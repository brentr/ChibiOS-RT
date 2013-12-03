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

msg_t  i2cMatchAddress(I2CDriver *i2cp, i2caddr_t  i2cadr);
/*
    Respond to messages directed to the given i2cadr.
    MatchAddress calls are cumulative.
    Specify address zero to match I2C "all call"

    Returns error if driver does not support matching the specified address in
    addition to those already being matched.
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

void i2cSlaveStart(I2CDriver *i2cp,
                   const I2CSlaveMsg *rxMsg, const I2CSlaveMsg *replyMsg);
/*
  Prepare to receive and process I2C messages and reply to read requests.

  Notes:
      Must be called from a thread
      One must subsequently call i2cMatchAddress() to enable slave processing
      Enabling match addresses before calling i2cSlaveStart() will
      result in locking the I2C bus when a master accesses those slave addresses
*/



/*
  Advanced Usage:

  Each I2CSlaveMsgCB function may alter the processing of subsequent I2C
  messages and read requests by calling i2cSlaveReceiveI() and
  i2cSlaveReplyI(), respectively.

  If an I2CSlaveMsg struct is in RAM, these callbacks may alter them.
  Such changes take immediate affect.  This facility can be used to
  avoid copying message buffers.

  If receive buffers become full or a reply to a read request cannot be
  immediately generated, a I2CSlaveMsg pointer may be set to NULL, via
  i2cSlaveReceive() or i2cSlaveReply(), as appropriate.
  This signals the master node to wait by holding the I2C clock signal
  low -- stretching the I2C clock -- on the next transaction to which
  that I2CSlaveMsg applies.
  The I2C bus is unlocked only after a i2cSlaveReceive() or Reply() is
  called with a non-NULL I2CSlaveMsg pointer.

  Note that I2CSlaveMsg pointers are NULL before i2cSlaveStart() is called.
*/


void i2cSlaveReceive(I2CDriver *i2cp, const I2CSlaveMsg *rxMsg);
/*
  Prepare to receive and process I2C messages according to
  the rxMsg configuration.

  Notes:
    Called from thread context
      Does not affect the processing of any message currently being received
*/

#define i2cSlaveReceiveMsg(i2cp)  ((i2cp)->slaveNextRx)

void i2cSlaveReply(I2CDriver *i2cp, const I2CSlaveMsg *replyMsg);
/*
  Prepare to reply to subsequent I2C read requests from bus masters
  according to the replyMsg configuration.

  Notes:
    Called from thread context
      Does not affect the processing of any message reply being sent
*/

#define i2cSlaveReplyMsg(i2cp)  ((i2cp)->slaveNextReply)

/*
  most recently matched target i2c address
*/
#define i2cTargetAdr(i2cp) ((i2cp)->targetAdr)


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
  i2c_lld_unmatchAll(i2cp);
}

#endif  /* HAL_USE_I2C_SLAVE */

#endif  /* _I2CSLAVE_H_ */
