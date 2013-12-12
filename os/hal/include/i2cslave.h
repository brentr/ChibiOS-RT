/*
    ChibiOS/RT - Copyright (C) 2006,2007,2008,2009,2010,
                 2011,2012,2013 Giovanni Di Sirio.

    This file is part of ChibiOS/RT.

    ChibiOS/RT is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    (at your option) any later version.

    ChibiOS/RT is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

                                      ---

    A special exception to the GPL can be applied should you wish to distribute
    a combined work that includes ChibiOS/RT, without being obliged to provide
    the source code for any proprietary components. See the file exception.txt
    for full details of how and when the exception can be applied.
*/
/*
   Slave I2C support contributed by Brent Roman of the
    Monterey Bay Aquarium Research Institute
 */

/**
 * @file    i2cslave.h
 * @brief   Slave Mode for the I2C Driver.
 *
 * @addtogroup I2C
 * @{
 */
#ifndef _I2CSLAVE_H_
#define _I2CSLAVE_H_

#if HAL_USE_I2C_SLAVE || defined(__DOXYGEN__)

#include <i2c.h>

#ifdef __cplusplus
extern "C" {
#endif

int  i2cMatchAddress(I2CDriver *i2cp, i2caddr_t  i2cadr);
/*
    Respond to messages directed to the given i2cadr.
    MatchAddress calls are cumulative.
    Specify address zero to match I2C "all call"

    Returns non-zero if driver does not support matching the
    specified address in addition to those already being matched.
    Note that most drivers will only support matching a single nonzero address.
*/

void  i2cUnmatchAddress(I2CDriver *i2cp, i2caddr_t  i2cadr);
/*
    Do not match specified i2cadr.
    A message being transferred that has already matched the specified address,
    will continue being processed.
    Requests to unmatch an address that is not currently being matched
    are ignored.
*/

void  i2cUnmatchAll(I2CDriver *i2cp);
/*
    Clears all match addresses.  Causes all subsequent messages to be ignored.
    A message being transferred that has already matched a slave address
    will continue being processed.
*/


msg_t  i2cSlaveAwaitEvent(I2CDriver *i2cp,
           uint8_t *inputBuffer, size_t size, i2caddr_t *targetAdr);
/*
  Waits for a received message, query, error, or timeout.
  Incoming messages are received into inputBuffer,
  If non-NULL, *targetAdr is assigned the target address.

  If returned value is >=0, it is the number of bytes received
  otherwise, the return value:
    I2C_QUERY indicates master is awaiting a response from this slave and that
    one must call i2cSlaveAnswer() before next calling i2cSlaveAwaitMessage()

    I2C_ERROR indicates an error occured
    details can be retrieved via i2cGetErrors()

    I2C_TIMEOUT implies slave locked the bus too long and it has been unlocked.
*/

msg_t  i2cSlaveAnswer(I2CDriver *i2cp,
                      const uint8_t *replyBuffer, size_t size);
/*
  Blocks until replyBuffer is sent back to the requesting master.
  Invoke directly after i2cSlaveAwaitEvent() returns I2C_QUERY.

  If returned value is >=0, it is the number of bytes transmitted
  otherwise, the return value:
    I2C_ERROR indicates that an error occured
    details can be retrieved via i2cGetErrors()

    I2C_TIMEOUT implies slave locked the bus too long and it has been unlocked.
*/


static INLINE
  i2cflags_t i2cSlaveErrors(I2CDriver *i2cp)
/*
  returns mask of errors for last slave message (partially) received
*/
{
  chDbgCheck((i2cp != NULL), "i2cSlaveErrors");
  return i2c_lld_get_slaveErrors(i2cp);
}

static INLINE
  systime_t i2cSlaveTimeout(I2CDriver *i2cp)
/*
  returns the maximum number of ticks slave may stretch the I2C clock
  initialized to TIME_INFINITE (disabling slave mode bus lock timeouts)
*/
{
  chDbgCheck((i2cp != NULL), "i2cSlaveTimeout");
  return i2c_lld_get_slaveTimeout(i2cp);
}

static INLINE
  void i2cSlaveSetTimeout(I2CDriver *i2cp, systime_t ticks)
/*
  set the maximum number of ticks slave may stretch the I2C clock
  TIME_IMMEDIATE is invald
  TIME_INFINITE is disables slave mode bus lock timeouts
*/
{
  chDbgCheck((i2cp != NULL && ticks != TIME_IMMEDIATE), "i2cSlaveSetTimeout");
  i2c_lld_set_slaveTimeout(i2cp, ticks);
}



/*
  Advanced Usage with Asynchronous Callback functions:

  Asynchronous callback functions that are used internally to implement
  i2cSlaveAwaitMessage() and i2cSlaveAnswer() above are described below:

  i2cSlaveAwaitMessage() and i2cSlaveAnswer() will conflict with the
  functions defined below.

  Each callback function may alter the processing of subsequent I2C
  messages and read requests by calling i2cSlaveReceive() and
  i2cSlaveReply(), respectively.

  If an I2CSlaveMsg struct is in RAM, these callbacks may alter them.
  Such changes take immediate affect.  This facility can be used to
  avoid copying message buffers.

  If receive buffers become full or a reply to a read request cannot be
  immediately generated, an I2CSlaveMsg pointer may be set to NULL, via
  i2cSlaveLockOnReceive() or i2cSlaveLockOnReply(), as appropriate.
  This signals the master node to wait by holding the I2C clock signal
  low -- stretching the I2C clock -- on the next transaction to which
  that I2CSlaveMsg applies.
  The I2C bus is unlocked only after a i2cSlaveSetReceive() or SetReply() is
  called with a non-NULL I2CSlaveMsg pointer.

  All Receive and Reply are initially Locked.
*/

static INLINE
  size_t i2cSlaveBytes(I2CDriver *i2cp)
/*
  length of most recently received slave message
*/
{
  chDbgCheck((i2cp != NULL), "i2cSlaveBytes");
  return i2c_lld_get_slaveBytes(i2cp);
}


void i2cSlaveConfigure(I2CDriver *i2cp,
                   const I2CSlaveMsg *rxMsg, const I2CSlaveMsg *replyMsg);
/*
  Configure to receive and process I2C messages and reply to read requests.

  Notes:
      Must be called from a thread
      One must subsequently call i2cMatchAddress() to enable slave processing
      Enabling match addresses before installing handler callbacks can
      result in locking the I2C bus when a master accesses those slave addresses
*/

void i2cSlaveReceive(I2CDriver *i2cp, const I2CSlaveMsg *rxMsg);
/*
  Prepare to receive and process I2C messages according to
  the rxMsg configuration.

  Notes:
    Called from thread context
      Does not affect the processing of any message currently being received
*/

static INLINE
  const I2CSlaveMsg *i2cSlaveReceiveMsg(I2CDriver *i2cp)
/*
  processing descriptor for the next received message
*/
{
  chDbgCheck((i2cp != NULL), "i2cSlaveReceiveMsg");
  return i2c_lld_get_slaveReceive(i2cp);
}

static INLINE
  const I2CSlaveMsg *i2cSlaveLockReceive(I2CDriver *i2cp)
/*
  Lock the I2C bus on reception of next message
  until i2cSlaveReceive is called with a non-null rxMsg parameter
*/
{
  chDbgCheck((i2cp != NULL), "i2cSlaveLockReceive");
  return i2c_lld_lock_slaveReceive(i2cp);
}


void i2cSlaveReply(I2CDriver *i2cp, const I2CSlaveMsg *replyMsg);
/*
  Prepare to reply to subsequent I2C read requests from bus masters
  according to the replyMsg configuration.

  Notes:
    Called from thread context
      Does not affect the processing of any message reply being sent
*/

static INLINE
  const I2CSlaveMsg *i2cSlaveReplyMsg(I2CDriver *i2cp)
/*
  processing descriptor for the next reply message
*/
{
  chDbgCheck((i2cp != NULL), "i2cSlaveReplyMsg");
  return i2c_lld_get_slaveReply(i2cp);
}

static INLINE
  const I2CSlaveMsg *i2cSlaveLockReply(I2CDriver *i2cp)
/*
  Lock the I2C bus on next query
  until i2cSlaveReply is called with a non-null replyMsg parameter
*/
{
  chDbgCheck((i2cp != NULL), "i2cSlaveLockReply");
  return i2c_lld_lock_slaveReply(i2cp);
}

static INLINE
  i2caddr_t i2cSlaveTargetAdr(I2CDriver *i2cp)
/*
  target address of slave message being or last processed
*/
{
  chDbgCheck((i2cp != NULL), "i2cSlaveTargetAdr");
  return i2c_lld_get_slaveTargetAdr(i2cp);
}

static INLINE
  i2caddr_t i2cSlaveMatchedAdr(I2CDriver *i2cp)
/*
  target address of slave message just matched
*/
{
  chDbgCheck((i2cp != NULL), "i2cSlaveMatchedAdr");
  return i2c_lld_get_slaveMatchedAdr(i2cp);
}


static INLINE void
  i2cSlaveReceiveI(I2CDriver *i2cp, const I2CSlaveMsg *rxMsg)
/*
  Prepare to receive and process I2C messages according to
  the rxMsg configuration.

  Notes:
      Must be called from interrupt context
      Does not affect the processing of any message currently being received
*/
{
  chDbgCheck((i2cp != NULL), "i2cSlaveReceiveI");
  i2c_lld_slaveReceive(i2cp, rxMsg);
}

static INLINE void
  i2cSlaveReplyI(I2CDriver *i2cp, const I2CSlaveMsg *replyMsg)
/*
  Prepare to reply to subsequent I2C read requests from bus masters
  according to the replyMsg configuration.

  Notes:
      Must be called from interrupt context
      Does not affect the processing of any message reply being sent
*/
{
   chDbgCheck((i2cp != NULL), "i2cSlaveReplyI");
   i2c_lld_slaveReply(i2cp, replyMsg);
}


static INLINE msg_t
  i2cMatchAddressI(I2CDriver *i2cp, i2caddr_t  i2cadr)
/*
  Notes:
      Must be called from interrupt context
      Does not affect the processing of any message currently being received
*/
{
  chDbgCheck((i2cp != NULL), "i2cSlaveMatchAddressI");
  return i2c_lld_matchAddress(i2cp, i2cadr);
}

static INLINE void
  i2cUnmatchAddressI(I2CDriver *i2cp, i2caddr_t  i2cadr)
/*
  Notes:
      Must be called from interrupt context
      Does not affect the processing of any message currently being received
*/
{
  chDbgCheck((i2cp != NULL), "i2cSlaveUnmatchAddressI");
  i2c_lld_unmatchAddress(i2cp, i2cadr);
}

static INLINE void
  i2cUnmatchAllI(I2CDriver *i2cp)
/*
  Notes:
      Must be called from interrupt context
      Does not affect the processing of any message currently being received
*/
{
  chDbgCheck((i2cp != NULL), "i2cSlaveUnmatchAllI");
  i2c_lld_unmatchAll(i2cp);
}

#ifdef __cplusplus
}
#endif

#endif  /* HAL_USE_I2C_SLAVE */

#endif  /* _I2CSLAVE_H_ */
