/**
 * @file    i2c_event.c
 * @brief   I2C Driver Slave Event Service Thread API

 * Library to support an I2C Slave Event Service Thread
 * contributed by Brent Roman (brent@mbari.org)
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
   writes, increase the queue depth by 2 or more, as a master sending these
   will not be slowed by stretching the I2C clock.  The number of SMBus quick
   writes that must be queued is a roughly proportional to the I2C clock rate
   times the maximum latency of the event processing thread.
*/

#include "i2c_event.h"

#define ignore  I2CSlaveDummyCB


/*===========================================================================*/
/* Exported variables.                                                       */
/*===========================================================================*/

static I2CSlaveMsgCB  wakeOnRx, wakeOnQuery, wakeWhenSent, wakeOnError;

const I2CSlaveMsg i2cQrx = {0, NULL, ignore, wakeOnRx, wakeOnError},
            i2cQreply = {0, NULL, wakeOnQuery, ignore, wakeOnError};


/*===========================================================================*/
/* Local functions.                                                          */
/*===========================================================================*/

#define i2cQempty(body) ((body)->oldest == (body)->newest)

static INLINE void i2cQdeq(i2cEventQbody *body, i2cQindex depth)
/*
  remove the oldest element from the queue
*/
{
  body->newest &= ~i2cQfull;
  if (++body->oldest >= depth)
    body->oldest = 0;
}


static void queueCurrentEvent(I2CDriver *i2cp, i2cEventType type)
/*
  add the current i2c event type specified to q associated with device i2cp
*/
{
  const i2cEventQ *i2cq = &((const i2cEventConfig *)i2cp->config)->queue;
  i2cEventQbody *body = i2cq->body;
  i2cQindex newest = body->newest;
  if (newest & i2cQfull)
    body->dropped++;  /* we've nowhere to put this event :-( */
  else{
    i2cEvent *next = body->event + newest;
    if (++newest >= i2cq->depth)
      newest = 0;
    if (newest == body->oldest)
      newest |= i2cQfull;  /* fifo just became full */
    body->newest = newest;
    next->type = type;
    next->targetAdr = i2c_lld_get_slaveTargetAdr(i2cp);
    next->flags = i2c_lld_get_slaveErrors(i2cp);
    next->bytes = i2c_lld_get_slaveBytes(i2cp);
  }
  chSysLockFromIsr();
  Thread *tp = body->thread;
  if (tp != NULL) {
    body->thread = NULL;
    chSchReadyI(tp);
  }
  chSysUnlockFromIsr();
}


static void wakeOnRx(I2CDriver *i2cp)
{
  queueCurrentEvent(i2cp, i2cMessage);
}

static void wakeOnQuery(I2CDriver *i2cp)
{
  queueCurrentEvent(i2cp, i2cQuery);
}

static void wakeWhenSent(I2CDriver *i2cp)
{
  queueCurrentEvent(i2cp, i2cReplied);
}

static void wakeOnError(I2CDriver *i2cp)
{
  queueCurrentEvent(i2cp, i2cError);
}


static const i2cEvent *eventsDropped(i2cEventQbody *body)
/*
  invoke when dropped > 0 with an empty queue
  reinitializes the event queue, returning a i2cDropped event
  (invoked with system locked, returns with it unlocked)
*/
{
  i2cEvent *dropped = &body->event[body->oldest = 0];
  body->newest = 1;
  dropped->bytes = body->dropped;
  body->dropped = 0;
  chSysUnlock();
  dropped->type = i2cDropped;
  dropped->flags = I2CD_OVERRUN;
  dropped->targetAdr = i2cInvalidAdr;
  return (const i2cEvent *)dropped;
}


/*===========================================================================*/
/* Exported functions.                                                       */
/*===========================================================================*/


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
const i2cEvent  *i2cAwaitEvent(I2CDriver *i2cp,
                        uint8_t *inputBuffer, size_t size)
{
  chDbgCheck((i2cp!=NULL && inputBuffer!=NULL && size>0), "i2cAwaitEvent");
  const I2CSlaveMsg rx = {size, inputBuffer, ignore, wakeOnRx, wakeOnError};
  const i2cEventQ *i2cq = &((i2cEventConfig *)i2cp->config)->queue;
  i2cEventQbody *body = i2cq->body;

  chSysLock();
#ifdef CH_DBG_SYSTEM_STATE_CHECK
  if (body->thread)
    chDbgPanic("i2cAwaitEvent reentry");
#endif
  if (i2cQempty(body))
    goto empty;
  i2cQdeq(body, i2cq->depth);  /* dequeue last event returned */
  if (i2cQempty(body)) {
empty:
    if (body->dropped > 0)
      return eventsDropped(body);
    i2c_lld_slaveReceive(i2cp, &rx);
    i2cp->slaveNextReply = &i2cQreply;
    body->thread = chThdSelf();
    chSchGoSleepS(THD_STATE_SUSPENDED);
    i2cp->slaveNextRx = &i2cQrx;
  }
  i2cQindex oldest = body->oldest;
  chSysUnlock();

  return body->event + oldest;
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
const i2cEvent *i2cAnswer(I2CDriver *i2cp,
                      const uint8_t *replyBuffer, size_t size)
{
  chDbgCheck((i2cp!=NULL && replyBuffer!=NULL && size>0), "i2cAnswer");
  const I2CSlaveMsg answer =
    {size, (uint8_t *)replyBuffer, ignore, wakeWhenSent, wakeOnError};
  const i2cEventQ *i2cq = &((i2cEventConfig *)i2cp->config)->queue;
  i2cEventQbody *body = i2cq->body;

  chSysLock();
#ifdef CH_DBG_SYSTEM_STATE_CHECK
  if (body->thread)
    chDbgPanic("i2cAnswer reentry");
#endif
  if (i2cQempty(body))
    goto empty;
  i2cQdeq(body, i2cq->depth);  /* dequeue last event returned */
  if (i2cQempty(body)) {
empty:
    if (body->dropped > 0)
      return eventsDropped(body);
    i2c_lld_slaveReply(i2cp, &answer);
    body->thread = chThdSelf();
    chSchGoSleepS(THD_STATE_SUSPENDED);
    i2cp->slaveNextReply = &i2cQreply;
  }
  i2cQindex oldest = body->oldest;
  chSysUnlock();

  return body->event + oldest;
}

/** @} */
