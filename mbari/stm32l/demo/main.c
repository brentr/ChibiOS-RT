#include <ch.h>
#include <hal.h>
#include <string.h>
#include <memstreams.h>
#include <chprintf.h>
#include <math.h>

#include "dccput.h"

/*
This code is an example of using the openocd debug message system.

Before the message output is seen in the debug window, the functionality
will need enabling:

** GDB **
From the gdb prompt: monitor target_request debugmsgs enable

** Telnet **
From the Telnet prompt: target_request debugmsgs enable

Spen
spen@spen-soft.co.uk
*/

/*
 * Working area for the debug output thread
 */
Thread *debugReader;
static WORKING_AREA(debugReaderArea, 128);

char debugOutput[500];  //debugging output awaiting transmission to host

static void resumeReader(void)
/*
  called whenever debug data is written
  to resume the queue reader
*/
{
  chSysLock();
  if(debugReader->p_state == THD_STATE_WTQUEUE)
    chSchWakeupS(debugReader, 0);
  chSysUnlock();
}


static uint8_t fetcher(void *outQ)
/*
  return next byte in queue
  block if empty
*/
{
  OutputQueue *q = outQ;
  msg_t b;
  chSysLock();
  while(TRUE) {
    b = chOQGetI(q);
    if (b != Q_EMPTY)
      break;
    chSchGoSleepS(THD_STATE_WTQUEUE);
  }
  chSysUnlock();
  return b;
}


static OUTPUTQUEUE_DECL(debugOutQ,
         debugOutput, sizeof(debugOutput), NULL, NULL);

static MUTEX_DECL(debugOutLock);


/*
 * This thread empties the debug output queue
 */
__attribute__((noreturn))
static msg_t debugReaderMain(void *arg)
{ 
  (void) arg;
  while (TRUE) {
    size_t len = fetcher(&debugOutQ);
    if (len)
      DCCputsQ(fetcher, &debugOutQ, len);
    else
      DCCputc(fetcher(&debugOutQ));
  }
}


int debugPutc(int c)
{
  chMtxLock(&debugOutLock);
  if (chOQGetEmptyI(&debugOutQ) > 1) {
    chOQPutTimeout( &debugOutQ, 0, TIME_IMMEDIATE);
    chOQPutTimeout( &debugOutQ, c, TIME_IMMEDIATE);
  }else
    c = -1;
  resumeReader();
  chMtxUnlock();
  return c;
}


size_t debugPut(const uint8_t *block, size_t n)
/*
  truncate any block > 255 bytes
*/
{
  if (chOQIsFullI(&debugOutQ))
    return -1;
  if (n) {
    chMtxLock(&debugOutLock);
    size_t space = chOQGetEmptyI(&debugOutQ);
    if (space) {
      if (n >= space)
        n = space - 1;
      if (n) {
        if (n > 255)
          n = 255;
        chOQPutTimeout( &debugOutQ, n, TIME_IMMEDIATE);
        chOQWriteTimeout( &debugOutQ, block, n, TIME_IMMEDIATE);
        resumeReader();
      }
    }
    chMtxUnlock();
  }else
    debugPutc('\n');
  return n;
}

size_t debugPuts(const char *str)
{
  return debugPut( (const uint8_t *)str, strlen(str) );
}

size_t debugPrint(const char *fmt, ...)
/*
  printf style debugging output
  outputs a trailing newline
*/
{
  uint8_t buf[255];
  MemoryStream stream;
  va_list ap;
  va_start(ap, fmt);  
  msObjectInit(&stream, buf, sizeof(buf), 0);
  chvprintf((BaseSequentialStream *) &stream, fmt, ap);
  va_end(ap);
  debugPut(buf, stream.eos);
  return stream.eos;
}


void logPanic(const char *panicTxt)
/*
  intended to be called from the SYSTEM_HALT_HOOK
*/
{
  if (!panicTxt)
    panicTxt = "<stack crash>";
  debugPuts(dbg_panic_msg);
  chSysLock(); 
  chSchGoSleepS(THD_STATE_FINAL);
}


int main(void) {
  unsigned count = 0;
  halInit();
  chSysInit();

  palSetPadMode(GPIOB, 7, PAL_MODE_OUTPUT_PUSHPULL);

  /* Start background debug output thread */
  debugReader = chThdCreateStatic(debugReaderArea, sizeof(debugReaderArea),
                          LOWPRIO, debugReaderMain, NULL);

  palSetPad(GPIOB, 7);
  chThdSleepMilliseconds(400);
  palClearPad(GPIOB, 7);
  chThdSleepMilliseconds(250);
  
  debugPuts("ChiDemo Blinky v1.1 -- 8/25/13 brent@mbari.org");

  while (1) {
    palSetPad(GPIOB, 7);
    debugPutc('+');
    chThdSleepMilliseconds(500);
    palClearPad(GPIOB, 7);
    debugPutc('-');
    chThdSleepMilliseconds(250);
    if (!(++count % 32))
      debugPutc('\n');
  }
}

