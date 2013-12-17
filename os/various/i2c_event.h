/**
 * @file    i2c_event.h
 * @brief   I2C Driver Slave Event Service Thread API

 * Library to support an I2C Slave Event Service Thread
 * added by brent@mbari.org
 *
 * @addtogroup I2C
 * @{
 */

/*
   The core ISR callback API must process events as they occur,
   however, an event handling thread may not always be waiting
   when an I2C event occurs.  I2C events must therefore be queued
   for the servicing thread, along with all their associated meta-data.

   We assume that the low-level driver will be able to stretch
   the I2C clock when data is being transferred to or fram the slave device.
   Therefore, there is no need to queue message data.

   However...

   At least for the STM32, messages ended with an I2C "repeated start"
   are not recognized as complete until after the next message is begun.
   In this case, two I2C events are generated in one interrupt service.
   (The end of the previous followed by the begin of the current message)
   Therefore, the depth of the event queue for STM32 I2C interfaces must
   be at least two.

   In general, if your slave must respond to 0-byte SMBbus style quick
   commands, increase the queue depth by 2 or more, as a master sending these
   cannot be slowed by stretching the I2C clock.  The number of SMBus quick
   commands that must be queued is a roughly proportional to the I2C clock rate
   times the maximum latency of the event processing thread.
*/

#include "ch.h"
#include "hal.h"

#if !HAL_USE_I2C || !HAL_USE_I2C_SLAVE
#error  Both HAL_USE_I2C and HAL_USE_I2C_SLAVE must be enabled
#endif

#include "i2cslave.h"

#define ignore  I2CSlaveDummyCB

typedef uint16_t i2cQindex;

typedef enum {
  i2cMessage,       /* message received from bus master */
  i2cQuery,         /* bus master is waiting for our reply to its query */
  i2cReplied,       /* reply accepted by bus master */
  i2cError          /* operation aborted due to error */
} i2cEventType;

typedef struct i2cEvent {
  i2cEventType  type;       /* event type */
  i2caddr_t     targetAdr;  /* target i2c address */
  i2cflags_t    flags;      /* associated error mask -- zero for timeout */
  size_t        bytes;      /* # of bytes actually transferred */
} i2cEvent;

typedef struct i2cEventQbody {  /* event queue body */
  Thread    *thread;          /* thread waiting to process next event */
  unsigned   dropped;         /* # of events dropped due to full queue */
  i2cQindex  newest, oldest;  /* index of newest and oldest events in queue */
  i2cEvent   event[1];        /* event queue placeholder */
} i2cEventQbody;              /*   (actual array size is 1+depth elements) */

typedef struct i2cEventQ {  /* event queue descriptor */
  i2cEventQbody *body;      /* event queue */
  i2cQindex      depth;     /* max # of events that may be queued */
} i2cEventQ;

/* extended I2CConfig associates event queue qith low-level driver */
typedef struct i2cEventConfig {
  I2CConfig  cfg;    /* base driver configuration */
  i2cEventQ  queue;  /* descriptor of associated event queue */
}

/*
  the size of an i2cEventQ as a function of its (maximum) depth
*/
#define i2cEventSizeofQ(depth) (sizeof(i2cEventQbody)+depth*sizeof(i2cEvent))

/*
  statically allocate an i2cEventQ of given name and depth
*/
#define i2cEventStaticQ(name, depth) union { \
  {i2cEventQbody q; char space[i2cEventSizeofQ];} name

/*
  pointer to i2cEventQbody allocated with i2cEventStaticQ() above
*/
#define i2cEventStaticQbody(name) ((name).q)

/*
   Example of allocating a queue foo statically and referencing it

static i2cEventStaticQ(foo, 4);    //named foo with max depth 4 (w/5 elements)

     &i2cEventStaticQbody(foo)     // pointer to body of queue allocated above
*/


/*===========================================================================*/
/* Local functions.                                                          */
/*===========================================================================*/

static void queueCurrentEvent(
  I2CDriver *i2cp, i2cEventType type, i2caddr_t targetAdr)
/*
  add the current i2c event type specified to q associated with device i2cp
  targetAdr parameter is i2cp->targetAdr or i2cp->nextTargetAdr as appropriate
*/
{
  const i2cEventQ *i2cq = &(const i2cEventConfig *)(i2cp->config).queue;
  i2cEventQbody *body = i2cq->body;
  i2cQindex newest = body->newest+1;
  if (newest >= i2cq->depth)
    newest = 0;
  /*
  if (newest != body->oldest) {  /* not full */
    i2cEventQ *next = body->event + newest;
    next->type = type;
    next->targetAdr = targetAdr;
    next->flags = i2c_lld_get_slaveErrors(i2cp);
    next->bytes = i2c_lld_get_slaveBytes(i2cp);
    body->newest = newest;
  }else /* we've nowhere to put this event :-( */
    i2cq->dropped++;

  i2cp->slaveNextRx = i2cp->slaveNextReply = I2CSlaveLockOnMsg;

  chSysLockFromIsr();
  Thread *tp = body->thread;
  if (tp != NULL) {
    body->thread = NULL;
    chSchReadyI(tp);
  }
  chSysUnlockFromIsr();
}


#define qEmpty(body) ((body).oldest == (body).newest))



static void wakeOnRx(I2CDriver *i2cp)
{
  queueCurrentEvent(i2cp, i2cMessage, i2c_lld_get_slaveTargetAdr(i2cp));
}

static void wakeOnQuery(I2CDriver *i2cp)
{
  queueCurrentEvent(i2cp, i2cQuery, i2c_lld_get_slaveMatchedAdr(i2cp));
}

static void wakeWhenSent(I2CDriver *i2cp)
{
  queueCurrentEvent(i2cp, i2cReplied, i2c_lld_get_slaveTargetAdr(i2cp));
}

static void wakeOnError(I2CDriver *i2cp)
{
  queueCurrentEvent(i2cp, i2cError, i2c_lld_get_slaveMatchedAdr(i2cp));
}


/*===========================================================================*/
/* Exported functions.                                                       */
/*===========================================================================*/


/**
 * @brief   Initialize I2C event queue associated with its descriptor
 *
 * @param[in] i2cEventQ  queue descriptor
 *
 * @details  This is necessary only if the queue body is allocated on the heap
 *
 * @api
 **/
static INLINE
  void i2cEventQinit(const i2cEventQ *i2cq)
{
  i2cEventQ *body = i2cq->body;
  body->oldest = body->newest = body->next = 0;
  body->dropped = 0;
  body->thread = NULL;
}


/**
 * @brief   Await next I2C message or event
 *
 * @param[in] i2cq          pointer to the @p I2CDriver object
 * @param[out] inputBuffer  pointer to where to store received message body
 * @param[in] size          size of inputBuffer
 *
 * @return              pointer to i2cEvent struct
 *
 * @details The returned pointer remains valid only until the next
 *          call to this function or slaveAnswer() described below.
 *
 * @api
 **/
const i2cEvent  *i2cAwaitEvent(const I2CDriver *i2cp,
                        uint8_t *inputBuffer, size_t size)
{
  chDbgCheck((i2cp!=NULL && inputBuffer!=NULL && size>0), "i2cAwaitEvent");
  static const uint8_t dummyReplyBody = 0;
  const I2CSlaveMsg rx = {size, inputBuffer, ignore, wakeOnRx, wakeOnError},
  reply = {1, (uint8_t *)&dummyReplyBody, wakeOnQuery, ignore, wakeOnError};
  chSysLock();
#if CH_DBG_SYSTEM_STATE_CHECK
  if (i2cq->thread)
    chDbgPanic("i2cAwaitEvent reentry");
#endif
  const i2cEventQ *i2cq = &(const i2cEventConfig *)(i2cp->config).queue;
  i2cEventQbody *body = i2cq->body;
  if (qEmpty(body)
    goto empty;
  if (++body->oldest >= i2q->depth)  /* dequeue last event returned */
    body->oldest = 0;
  if (qEmpty(body)) {
empty:
    i2c_lld_slaveReceive(i2cp, &rx);
    i2c_lld_slaveReply(i2cp, &reply);
    i2cq->thread = chThdSelf();
    chSchGoSleepS(THD_STATE_SUSPENDED);
  }
  const i2cEvent *event = body->event + body->oldest;
  chSysUnlock();
  return event;
}



/**
 * @brief   Reply to latest I2C query
 *
 * @param[in] i2cq          pointer to the @p I2CDriver object
 * @param[in] replyBuffer   pointer to body of reply
 * @param[in] size          size of replyBuffer
 *
 * @return              pointer to i2cEvent
 *
 * @details This function is to called directly after i2cSlaveAwaitEvent()
 *          indicates that a master is awaiting a response to a query.
 *
 *          Will return a pointer to an i2cReplied or i2cError event.
 *          The returned pointer remains valid only until the next
 *          call to awaitEvent() described above.
 *
 * @notapi
 **/
const i2cEvent *i2cAnswer(const I2CDriver *i2cp,
                      const uint8_t *replyBuffer, size_t size)
{
  chDbgCheck((i2cp!=NULL && replyBuffer!=NULL && size>0), "i2cAnswer");
  const I2CSlaveMsg answer =
    {size, (uint8_t *)replyBuffer, ignore, wakeWhenSent, wakeOnError};
  chSysLock();
#if CH_DBG_SYSTEM_STATE_CHECK
  if (i2cq->thread)
    chDbgPanic("i2cAnswer reentry");
#endif
  const i2cEventQ *i2cq = &(const i2cEventConfig *)(i2cp->config).queue;
  i2cEventQbody *body = i2cq->body;
  if (qEmpty(body)
    goto empty;
  if (++body->oldest >= i2q->depth)  /* dequeue last event returned */
    body->oldest = 0;
  if (qEmpty(body)) {
empty:
    i2c_lld_slaveReply(i2cp, &answer);
    i2cq->thread = chThdSelf();
    chSchGoSleepS(THD_STATE_SUSPENDED);
  }
  const i2cEvent *event = body->event + body->oldest;
  chSysUnlock();
  return event;
}

#endif  /* I2C_SLAVE_EVENTQ_SIZE > 0 */

#endif /* HAL_USE_I2C_SLAVE */

/** @} */
