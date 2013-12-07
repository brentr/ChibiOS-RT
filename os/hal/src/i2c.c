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
   Concepts and parts of this file have been contributed by Uladzimir Pylinsky
   aka barthess.
 */

/**
 * @file    i2c.c
 * @brief   I2C Driver code.
 *
 * @addtogroup I2C
 * @{
 */
#include "ch.h"
#include "hal.h"

#if HAL_USE_I2C || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   I2C Driver initialization.
 * @note    This function is implicitly invoked by @p halInit(), there is
 *          no need to explicitly initialize the driver.
 *
 * @init
 */
void i2cInit(void) {
  i2c_lld_init();
}

/**
 * @brief   Initializes the standard part of a @p I2CDriver structure.
 *
 * @param[out] i2cp     pointer to the @p I2CDriver object
 *
 * @init
 */
void i2cObjectInit(I2CDriver *i2cp) {

  i2cp->state  = I2C_STOP;
  i2cp->config = NULL;

#if I2C_USE_MUTUAL_EXCLUSION
#if CH_USE_MUTEXES
  chMtxInit(&i2cp->mutex);
#else
  chSemInit(&i2cp->semaphore, 1);
#endif /* CH_USE_MUTEXES */
#endif /* I2C_USE_MUTUAL_EXCLUSION */

#if defined(I2C_DRIVER_EXT_INIT_HOOK)
  I2C_DRIVER_EXT_INIT_HOOK(i2cp);
#endif
}

/**
 * @brief   Configures and activates the I2C peripheral.
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 * @param[in] config    pointer to the @p I2CConfig object
 *
 * @api
 */
void i2cStart(I2CDriver *i2cp, const I2CConfig *config) {

  chDbgCheck((i2cp != NULL) && (config != NULL), "i2cStart");
  chDbgAssert((i2cp->state == I2C_STOP) || (i2cp->state == I2C_READY) ||
              (i2cp->state == I2C_LOCKED),
              "i2cStart(), #1",
              "invalid state");

  chSysLock();
  i2cp->config = config;
  i2c_lld_start(i2cp);
  i2cp->state = I2C_READY;
  chSysUnlock();
}

/**
 * @brief   Deactivates the I2C peripheral.
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 *
 * @api
 */
void i2cStop(I2CDriver *i2cp) {

  chDbgCheck(i2cp != NULL, "i2cStop");
  chDbgAssert((i2cp->state == I2C_STOP) || (i2cp->state == I2C_READY) ||
              (i2cp->state == I2C_LOCKED),
              "i2cStop(), #1",
              "invalid state");

  chSysLock();
  i2c_lld_stop(i2cp);
  i2cp->state = I2C_STOP;
  chSysUnlock();
}

/**
 * @brief   Returns the errors mask associated to the previous operation.
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 * @return              The errors mask.
 *
 * @api
 */
i2cflags_t i2cGetErrors(I2CDriver *i2cp) {

  chDbgCheck(i2cp != NULL, "i2cGetErrors");

  return i2c_lld_get_errors(i2cp);
}

/**
 * @brief   Sends data via the I2C bus.
 * @details Function designed to realize "read-through-write" transfer
 *          paradigm. If you want transmit data without any further read,
 *          than set @b rxbytes field to 0.
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 * @param[in] addr      slave device address (7 bits) without R/W bit
 * @param[in] txbuf     pointer to transmit buffer
 * @param[in] txbytes   number of bytes to be transmitted
 * @param[out] rxbuf    pointer to receive buffer
 * @param[in] rxbytes   number of bytes to be received, set it to 0 if
 *                      you want transmit only
 * @param[in] timeout   the number of ticks before the operation timeouts,
 *                      the following special values are allowed:
 *                      - @a TIME_INFINITE no timeout.
 *                      .
 *
 * @return              The operation status.
 * @retval RDY_OK       if the function succeeded.
 * @retval RDY_RESET    if one or more I2C errors occurred, the errors can
 *                      be retrieved using @p i2cGetErrors().
 * @retval RDY_TIMEOUT  if a timeout occurred before operation end.
 *
 * @api
 */
msg_t i2cMasterTransmitTimeout(I2CDriver *i2cp,
                               i2caddr_t addr,
                               const uint8_t *txbuf,
                               size_t txbytes,
                               uint8_t *rxbuf,
                               size_t rxbytes,
                               systime_t timeout) {
  msg_t rdymsg;

  chDbgCheck((i2cp != NULL) && (addr != 0) &&
             (txbytes > 0) && (txbuf != NULL) &&
             ((rxbytes == 0) || ((rxbytes > 0) && (rxbuf != NULL))) &&
             (timeout != TIME_IMMEDIATE),
             "i2cMasterTransmitTimeout");

  chDbgAssert(i2cp->state == I2C_READY,
              "i2cMasterTransmitTimeout(), #1", "not ready");

  chSysLock();
  i2cp->errors = I2CD_NO_ERROR;
  i2cp->state = I2C_ACTIVE_TX;
  rdymsg = i2c_lld_master_transmit_timeout(i2cp, addr, txbuf, txbytes,
                                           rxbuf, rxbytes, timeout);
  i2cp->state = rdymsg == RDY_TIMEOUT ? I2C_LOCKED : I2C_READY;
  chSysUnlock();
  return rdymsg;
}

/**
 * @brief   Receives data from the I2C bus.
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 * @param[in] addr      slave device address (7 bits) without R/W bit
 * @param[out] rxbuf    pointer to receive buffer
 * @param[in] rxbytes   number of bytes to be received
 * @param[in] timeout   the number of ticks before the operation timeouts,
 *                      the following special values are allowed:
 *                      - @a TIME_INFINITE no timeout.
 *                      .
 *
 * @return              The operation status.
 * @retval RDY_OK       if the function succeeded.
 * @retval RDY_RESET    if one or more I2C errors occurred, the errors can
 *                      be retrieved using @p i2cGetErrors().
 * @retval RDY_TIMEOUT  if a timeout occurred before operation end.
 *
 * @api
 */
msg_t i2cMasterReceiveTimeout(I2CDriver *i2cp,
                              i2caddr_t addr,
                              uint8_t *rxbuf,
                              size_t rxbytes,
                              systime_t timeout){

  msg_t rdymsg;

  chDbgCheck((i2cp != NULL) && (addr != 0) &&
             (rxbytes > 0) && (rxbuf != NULL) &&
             (timeout != TIME_IMMEDIATE),
             "i2cMasterReceiveTimeout");

  chDbgAssert(i2cp->state == I2C_READY,
              "i2cMasterReceive(), #1", "not ready");

  chSysLock();
  i2cp->errors = I2CD_NO_ERROR;
  i2cp->state = I2C_ACTIVE_RX;
  rdymsg = i2c_lld_master_receive_timeout(i2cp, addr, rxbuf, rxbytes, timeout);
  i2cp->state = rdymsg == RDY_TIMEOUT ? I2C_LOCKED : I2C_READY;
  chSysUnlock();
  return rdymsg;
}


#if HAL_USE_I2C_SLAVE   /* I2C slave mode support */

msg_t i2cMatchAddress(I2CDriver *i2cp, i2caddr_t  i2cadr)
/*
    Respond to messages directed to the given i2cadr.
    MatchAddress calls are cumulative.
    Specify address zero to match I2C "all call"

    Does not support 10-bit addressing.
*/
{
  chDbgCheck((i2cp != NULL), "i2cMatchAddress");
  msg_t result;
  chSysLock();
  result = i2c_lld_matchAddress(i2cp, i2cadr);
  chSysUnlock();
  return result;
}


void i2cUnmatchAddress(I2CDriver *i2cp, i2caddr_t  i2cadr)
/*
    Do not match specified i2cadr.
    A message being transferred that has already matched the specified address
    will continue being processed.
    Requests to unmatch an address that is not currently being matched
    are ignored.
*/
{
  chDbgCheck((i2cp != NULL), "i2cUnmatchAddress");
  chSysLock();
  i2c_lld_unmatchAddress(i2cp, i2cadr);
  chSysUnlock();
}


void i2cUnmatchAll(I2CDriver *i2cp)
/*
    Clears all match addresses.  Causes all subsequent messages to be ignored.
    A message being transferred that has already matched a slave address
    will continue being processed.
*/
{
  chDbgCheck((i2cp != NULL), "i2cUnmatchAll");
  chSysLock();
  i2c_lld_unmatchAll(i2cp);
  chSysUnlock();
}


msg_t  i2cSlaveAwaitEvent(I2CDriver *i2cp,
                          uint8_t *inputBuffer,
                          size_t size,
                          i2caddr_t *targetAdr)
/*
  Waits for a received message, query, error, or timeout.
  Incoming messages are received into inputBuffer,
  If non-NULL, *targetAdr is assigned the target address.

  If returned value is >=0, it is the number of bytes received
  otherwise, the return value:
    I2C_QUERY implies master is awaiting a response from this slave
    One should call i2cSlaveReply() before calling i2cSalveAwaitMessage() again

    I2C_ERROR implies an error occured
    details can be retrieved via i2cGetErrors()

    I2C_TIMEOUT implies the bus remained locked too long and has been unlocked.
*/
{
  chDbgCheck((i2cp != NULL), "i2cSlaveAwaitEvent");
  chSysLock();
  msg_t result = i2c_lld_slaveAwaitEvent(i2cp, inputBuffer, size, targetAdr);
  chSysUnlock();
  return result;
}


msg_t  i2cSlaveAnswer(I2CDriver *i2cp,
                      const uint8_t *replyBuffer, size_t size)
/*
  Blocks until replyBuffer is sent back to the requesting master.
  Invoke directly after i2cSlaveAwaitEvent() returns I2C_QUERY.

  Returned value is one of:
    I2C_OK indicates success

    I2C_ERROR indicates that an error occured
    details can be retrieved via i2cGetErrors()

    I2C_TIMEOUT implies the bus remained locked too long and has been unlocked.
*/
{
  chDbgCheck((i2cp != NULL) && (replyBuffer != NULL) && (size > 0),
             "i2cSlaveAnswer");
  chSysLock();
  msg_t result = i2c_lld_slaveAnswer(i2cp, replyBuffer, size);
  chSysUnlock();
  return result;
}


/*  Support for "advanced" asychronous API  */

void i2cSlaveConfigure(I2CDriver *i2cp,
                   const I2CSlaveMsg *rxMsg, const I2CSlaveMsg *replyMsg)
/*
  Configure to receive and process I2C messages and reply to read requests.

  Notes:
      Must be called from a thread
      One must subsequently call i2cMatchAddress() to enable slave processing
      Enabling match addresses before calling i2cSlaveConfigure() will
      result in locking the I2C bus when a master accesses those slave addresses
*/
{
  chDbgCheck((i2cp != NULL), "i2cSlaveConfigure");
  chSysLock();
  i2c_lld_slaveReceive(i2cp, rxMsg);
  i2c_lld_slaveReply(i2cp, replyMsg);
  chSysUnlock();
}

void i2cSlaveReceive(I2CDriver *i2cp, const I2CSlaveMsg *rxMsg)
/*
  Prepare to receive and process I2C messages according to
  the rxMsg configuration.

  Notes:
      Does not affect the processing of any message currently being received
*/
{
  chDbgCheck((i2cp != NULL), "i2cSlaveReceive");
  chSysLock();
  i2c_lld_slaveReceive(i2cp, rxMsg);
  chSysUnlock();
}

void i2cSlaveReply(I2CDriver *i2cp, const I2CSlaveMsg *replyMsg)
/*
  Prepare to reply to subsequent I2C read requests from bus masters
  according to the replyMsg configuration.

  Notes:
      Does not affect the processing of any message reply being sent
*/
{
  chDbgCheck((i2cp != NULL), "i2cSlaveReply");
  chSysLock();
  i2c_lld_slaveReply(i2cp, replyMsg);
  chSysUnlock();
}

#endif /* HAL_USE_I2C_SLAVE */


#if I2C_USE_MUTUAL_EXCLUSION || defined(__DOXYGEN__)
/**
 * @brief   Gains exclusive access to the I2C bus.
 * @details This function tries to gain ownership to the SPI bus, if the bus
 *          is already being used then the invoking thread is queued.
 * @pre     In order to use this function the option @p I2C_USE_MUTUAL_EXCLUSION
 *          must be enabled.
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 *
 * @api
 */
void i2cAcquireBus(I2CDriver *i2cp) {

  chDbgCheck(i2cp != NULL, "i2cAcquireBus");

#if CH_USE_MUTEXES
  chMtxLock(&i2cp->mutex);
#elif CH_USE_SEMAPHORES
  chSemWait(&i2cp->semaphore);
#endif
}

/**
 * @brief   Releases exclusive access to the I2C bus.
 * @pre     In order to use this function the option @p I2C_USE_MUTUAL_EXCLUSION
 *          must be enabled.
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 *
 * @api
 */
void i2cReleaseBus(I2CDriver *i2cp) {

  chDbgCheck(i2cp != NULL, "i2cReleaseBus");

#if CH_USE_MUTEXES
  chMtxUnlock();
#elif CH_USE_SEMAPHORES
  chSemSignal(&i2cp->semaphore);
#endif
}
#endif /* I2C_USE_MUTUAL_EXCLUSION */

#endif /* HAL_USE_I2C */

/** @} */
