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
   The ISR callback API must process events as they occur,
   however, an event handling thread may not always be waiting
   when an I2C event occurs.  I2C events must therefore be queued
   for the servicing thread, along with all their associated meta-data.
   
   We assume that the low-level driver will be able to stretch
   the I2C clock when data is being transferred to or fram the slave device.
   Therefore, there is no need to queue message data.
   
   However...
   
   For the STM32, messages ended with an I2C "repeated start"
   are not recognized as complete until after the next message is begun.
   Therefore, two I2C events are generated in one interrupt service.
   The depth of the event queue for STM32 I2C interfaces must be at least two.
   
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

/*===========================================================================*/
/* Local definitions.                                                        */
/*===========================================================================*/

/*===========================================================================*/
/* Exported variables.                                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Local variables and types.                                                */
/*===========================================================================*/

typedef uint16_t i2cQindex;

typedef enum {
  i2cMessage,       /* message received from bus master */
  i2cQuery,         /* bus master is waiting for our reply to its query */
  i2cError          /* operation aborted due to error */
} i2cEventType;

typedef struct i2cEvent {
  i2cEventType  eventType;  /* event type */
  i2caddr_t     targetAdr;  /* target i2c address */
  i2cflags_t    flags;      /* associated error mask -- zero for timeout */
  size_t        bytes;      /* # of bytes actually transferred */
} i2cEvent;

typedef struct i2cEventQ {
  Thread    *thread;          /* thread waiting to process next event */
  i2cQindex  newest, oldest;  /* index of newest and oldest events in queue */
  i2cEvent   event[1];        /* event queue placeholder */
} i2cEventQ;                  /*   (actual array size is 1+depth elements) */

typedef struct i2cEventQcfg {  /* event queue descriptor */
  I2CDriver *bus;     /* low level i2c bus driver */
  i2cEventQ *q;       /* event queue */
  i2cQindex  depth;   /* max # of events that may be queued */
} i2cEventQcfg;

/*
  the size of an i2cEventQ as a function of its (maximum) depth
*/
#define i2cEventSizeofQ(depth) (sizeof(i2cEventQ)+depth*sizeof(i2cEvent))

/*
  statically allocate an i2cEventQ of given name and depth
*/
#define i2cEventStaticQ(name, depth) union { \
  {i2cEventQ q; char space[i2cEventSizeofQ];} name

/*
  pointer to i2cEventQ allocated with i2cEventStaticQ() above
*/
#define i2cEventStaticQptr(name) (&(name).q)

/* 
   Example of allocating a queue foo statically and referencing it

static i2cEventStaticQ(foo, 4);    //named foo with max depth 4 (w/5 elements)

       i2cEventStaticQptr(foo)     // pointer to above
*/
 
 
 
/*===========================================================================*/
/* Local functions.                                                          */
/*===========================================================================*/

    i2cp->slaveThread = NULL;

static void
  wakeOnRx(I2CDriver *i2cp)
{
  slaveBusy(i2cp);
  wakeup_isr(&i2cp->slaveThread, i2cp->slaveBytes);
}

static void
  wakeOnQuery(I2CDriver *i2cp)
{
  slaveBusy(i2cp);
  i2cp->targetAdr = i2cp->nextTargetAdr;
  wakeup_isr(&i2cp->slaveThread, I2C_QUERY);
}

static void
  wakeOnError(I2CDriver *i2cp)
{
  slaveBusy(i2cp);
  wakeup_isr(&i2cp->slaveThread, i2cp->slaveErrors ? I2C_ERROR : I2C_TIMEOUT);
}

/*===========================================================================*/
/* Exported functions.                                                       */
/*===========================================================================*/



/**
 * @brief   Await next I2C message or event
 *
 * @param[in] i2cp          pointer to the @p I2CDriver object
 * @param[out] inputBuffer  pointer to where to store received message body
 * @param[in] size          size of inputBuffer
 * @param[out] targetAdr    if non-NULL, store target address of message here
 *
 * @return              Length of message OR the type of event received
 * @retval >=0          Length message received
 * @retval I2C_QUERY    Master is awaiting a response via i2cSlaveReply()
 * @retval I2C_ERROR    if one or more I2C errors occurred, the errors can
 *                      be retrieved using @p i2cGetErrors().
 * @retval I2C_TIMEOUT  Did not call i2cSlaveReply() in time.   The I2C bus
 *                      had been locked too long and has been unlocked.
 * @notapi
 **/
msg_t  i2c_lld_slaveAwaitEvent(I2CDriver *i2cp,
                               uint8_t *inputBuffer,
                               size_t size,
                               i2caddr_t *targetAdr)
{
  static const uint8_t dummyReplyBody;
  const I2CSlaveMsg rx = {size, inputBuffer, NULL, wakeOnRx, wakeOnError},
  reply = {1, (uint8_t *)&dummyReplyBody, wakeOnQuery, NULL, wakeOnError};
#if CH_DBG_SYSTEM_STATE_CHECK
  if (i2cp->slaveThread)
    chDbgPanic("I2CawaitEvent reentry");
#endif
  i2c_lld_slaveReceive(i2cp, &rx);
  i2c_lld_slaveReply(i2cp, &reply);
  i2cp->slaveThread = chThdSelf();
  chSchGoSleepS(THD_STATE_SUSPENDED);
  if (targetAdr)
    *targetAdr = i2cp->targetAdr;
  return chThdSelf()->p_u.rdymsg;
}


static void
  wakeWhenSent(I2CDriver *i2cp)
{
  i2cp->slaveNextReply = NULL;
  wakeup_isr(&i2cp->slaveThread, i2cp->slaveBytes);
}


/**
 * @brief   Prepare to reply to I2C queries
 *
 * @param[in] i2cp          pointer to the @p I2CDriver object
 * @param[in] replyBuffer   pointer to body of reply
 * @param[in] size          size of replyBuffer
 *
 * @return              Length of message OR the type of event received
 * @retval >= 0         Length of reply transmitted (may be >size)
 * @retval I2C_QUERY    Master is awaiting a response via i2cSlaveReply()
 * @retval I2C_ERROR    if one or more I2C errors occurred, the errors can
 *                      be retrieved using @p i2cGetErrors().
 * @retval I2C_TIMEOUT  Did not call i2cSlaveReply() in time.   The I2C bus
 *                      had been locked too long and has been unlocked.
 * @notapi
 **/
msg_t i2c_lld_slaveAnswer(I2CDriver *i2cp,
                          const uint8_t *replyBuffer, size_t size)
{
  const I2CSlaveMsg answer =
    {size, (uint8_t *)replyBuffer, NULL, wakeWhenSent, wakeOnError};
#if CH_DBG_SYSTEM_STATE_CHECK
  if (i2cp->slaveThread)
    chDbgPanic("I2CslaveAnswer reentry");
#endif
  i2c_lld_slaveReply(i2cp, &answer);
  i2cp->slaveThread = chThdSelf();
  chSchGoSleepS(THD_STATE_SUSPENDED);
  return chThdSelf()->p_u.rdymsg;
}

#endif  /* I2C_SLAVE_EVENTQ_SIZE > 0 */

#endif /* HAL_USE_I2C_SLAVE */

/** @} */
