/**
 * @file    i2c_event.h
 * @brief   I2C Driver Slave Event Service Thread API

 * Library to support an I2C Slave Event Service Thread
 * contributed by Brent Roman (brent@mbari.org)
 *
 * @addtogroup I2C
 * @{
 */

#include "ch.h"
#include "hal.h"

#if !HAL_USE_I2C || !HAL_USE_I2C_SLAVE
#error  Both HAL_USE_I2C and HAL_USE_I2C_SLAVE must be enabled
#endif

#include "i2cslave.h"

/*===========================================================================*/
/* Exported types.                                                           */
/*===========================================================================*/

typedef uint16_t i2cQindex;
#define i2cQfull  (1<<15)    /* set MSB to indicate flag fifo is full */

typedef enum {
  i2cMessage,       /* message received from bus master */
  i2cQuery,         /* bus master is waiting for our reply to its query */
  i2cReplied,       /* reply accepted by bus master */
  i2cError,         /* operation aborted due to error */
  i2cDropped        /* missed (bytes) events due to queue overflow */
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
} i2cEventQbody;              /* (actual event array size is depth elements) */

typedef struct i2cEventQ {  /* event queue descriptor */
  i2cEventQbody *body;      /* event queue */
  i2cQindex      depth;     /* max # of events that may be queued */
} i2cEventQ;

/* extended I2CConfig associates event queue with the low-level driver */
typedef struct i2cEventConfig {
  I2CConfig  cfg;    /* base driver configuration */
  i2cEventQ  queue;  /* descriptor of associated event queue */
} i2cEventConfig;

/*===========================================================================*/
/* Definitions                                                               */
/*===========================================================================*/

/*
  the size of an i2cEventQ as a function of its (maximum) depth
*/
#define i2cEventSizeofQ(depth) \
  (sizeof(i2cEventQbody) + ((depth)-1)*sizeof(i2cEvent))

/*
  (statically) allocate an i2cEventQ of given name and depth
*/
#define i2cEventAllocateQ(depth) union { \
  i2cEventQbody body; char space[i2cEventSizeofQ(depth)];}


/*
  configuration for an i2c device with associated event queue
*/
#define i2cEventConfigQueue(qname, depth, ...) { \
  {__VA_ARGS__}, {&(qname).body, (depth)}}

/*
  complete configuration for queued i2c device channel
*/
#define i2cEventChannelCfg(channel, depth, ...) \
  static i2cEventAllocateQ(depth) channel##Q; \
  static const i2cEventConfig channel = \
    i2cEventConfigQueue(channel##Q, (depth), __VA_ARGS__)

/*
   Example of allocating a queued I2C channel with a 4 deep event queue

  //allocate i2c configuration and associated queue
  i2cEventChannel(myI2C, 4, OPMODE_I2C, 100000, STD_DUTY_CYCLE);
  ...
  i2cStart(&I2CD1, &myI2C.cfg);   //pass the driver its configuration struct
  ...
*/


/*===========================================================================*/
/* Exported variables.                                                       */
/*===========================================================================*/

extern const I2CSlaveMsg i2cQrx, i2cQreply;


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
  i2cEventQbody *body = i2cq->body;
  body->oldest = body->newest = 0;
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
 *          call to this function or i2cAnswer() described below.
 *
 * @api
 **/
const i2cEvent  *i2cAwaitEvent(I2CDriver *i2cp,
                               uint8_t *inputBuffer, size_t size);



/**
 * @brief   Reply to latest I2C query
 *
 * @param[in] i2cq          pointer to the @p I2CDriver object
 * @param[in] replyBuffer   pointer to body of reply
 * @param[in] size          size of replyBuffer
 *
 * @return              pointer to i2cEvent
 *
 * @details This function is called directly after i2cAwaitEvent()
 *          returns an i2cQuery event.  The next event will normally be
 *          i2cReplied.
 *
 *          Will return a pointer to an i2cReplied or i2cError event.
 *          The returned pointer remains valid only until the next
 *          call to i2cAwaitEvent() or i2cAnswer().
 *
 * @notapi
 **/
const i2cEvent *i2cAnswer(I2CDriver *i2cp,
                          const uint8_t *replyBuffer, size_t size);

/** @} */
