/*
    ChibiOS/RT - Copyright (C) 2006-2013 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

/**
 * @file    templates/i2c_lld.c
 * @brief   I2C Driver subsystem low level driver source template.
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

/**
 * @brief   I2C1 driver identifier.
 */
#if PLATFORM_I2C_USE_I2C1 || defined(__DOXYGEN__)
I2CDriver I2CD1;
#endif

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level I2C driver initialization.
 *
 * @notapi
 */
void i2c_lld_init(void) {

#if PLATFORM_I2C_USE_I2C1
  i2cObjectInit(&I2CD1);
#endif /* PLATFORM_I2C_USE_I2C1 */
}

/**
 * @brief   Configures and activates the I2C peripheral.
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 *
 * @notapi
 */
void i2c_lld_start(I2CDriver *i2cp) {

  if (i2cp->state == I2C_STOP) {
    /* Enables the peripheral.*/
#if PLATFORM_I2C_USE_I2C1
    if (&I2CD1 == i2cp) {

    }
#endif /* PLATFORM_I2C_USE_I2C1 */
  }
  /* Configures the peripheral.*/

}

/**
 * @brief   Deactivates the I2C peripheral.
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 *
 * @notapi
 */
void i2c_lld_stop(I2CDriver *i2cp) {

  if (i2cp->state != I2C_STOP) {
    /* Resets the peripheral.*/

    /* Disables the peripheral.*/
#if PLATFORM_I2C_USE_I2C1
    if (&I2CD1 == i2cp) {

    }
#endif /* PLATFORM_I2C_USE_I2C1 */
  }
}

/**
 * @brief   Receives data via the I2C bus as master.
 * @details Number of receiving bytes must be more than 1 on STM32F1x. This is
 *          hardware restriction.
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 * @param[in] addr      slave device address
 * @param[out] rxbuf    pointer to the receive buffer
 * @param[in] rxbytes   number of bytes to be received
 * @param[in] timeout   the number of ticks before the operation timeouts,
 *                      the following special values are allowed:
 *                      - @a TIME_INFINITE no timeout.
 *                      .
 * @return              The operation status.
 * @retval RDY_OK       if the function succeeded.
 * @retval RDY_RESET    if one or more I2C errors occurred, the errors can
 *                      be retrieved using @p i2cGetErrors().
 * @retval RDY_TIMEOUT  if a timeout occurred before operation end. <b>After a
 *                      timeout the driver must be stopped and restarted
 *                      because the bus is in an uncertain state</b>.
 *
 * @notapi
 */
msg_t i2c_lld_master_receive_timeout(I2CDriver *i2cp, i2caddr_t addr,
                                     uint8_t *rxbuf, size_t rxbytes,
                                     systime_t timeout) {

  (void)i2cp;
  (void)addr;
  (void)rxbuf;
  (void)rxbytes;
  (void)timeout;

  return RDY_OK;
}

/**
 * @brief   Transmits data via the I2C bus as master.
 * @details Number of receiving bytes must be 0 or more than 1 on STM32F1x.
 *          This is hardware restriction.
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 * @param[in] addr      slave device address
 * @param[in] txbuf     pointer to the transmit buffer
 * @param[in] txbytes   number of bytes to be transmitted
 * @param[out] rxbuf    pointer to the receive buffer
 * @param[in] rxbytes   number of bytes to be received
 * @param[in] timeout   the number of ticks before the operation timeouts,
 *                      the following special values are allowed:
 *                      - @a TIME_INFINITE no timeout.
 *                      .
 * @return              The operation status.
 * @retval RDY_OK       if the function succeeded.
 * @retval RDY_RESET    if one or more I2C errors occurred, the errors can
 *                      be retrieved using @p i2cGetErrors().
 * @retval RDY_TIMEOUT  if a timeout occurred before operation end. <b>After a
 *                      timeout the driver must be stopped and restarted
 *                      because the bus is in an uncertain state</b>.
 *
 * @notapi
 */
msg_t i2c_lld_master_transmit_timeout(I2CDriver *i2cp, i2caddr_t addr,
                                      const uint8_t *txbuf, size_t txbytes,
                                      uint8_t *rxbuf, size_t rxbytes,
                                      systime_t timeout) {

  (void)i2cp;
  (void)addr;
  (void)txbuf;
  (void)txbytes;
  (void)rxbuf;
  (void)rxbytes;
  (void)timeout;

  return RDY_OK;
}


#if HAL_USE_I2C_LOCK    /* I2C slave mode support */

/**
 * @brief   Lock I2C bus at the beginning of the next message sent
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 * @param[in] lockDuration   max number of ticks to hold bus locked
 *                      - @a TIME_INFINITE no timeout.
 *                      - @a TIME_IMMEDIATE unlock the bus immediately
 *                      .
 *
 * @notapi
 */
void i2c_lld_lock(I2CDriver *i2cp, systime_t lockDuration)
{
  (void)i2cp;
  (void)lockDuration;
}

#endif


#if HAL_USE_I2C_SLAVE   /* I2C slave mode support */

/**
 * @brief   Reconfigure I2C channel to respond to indicated address
 *          in addition to those already matched
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 * @param[in] i2cadr    I2C network address
 *
 * @return              Length of message OR the type of event received
 * @retval I2C_OK       Success
 * @retval I2C_ERROR    Cannot match address in addition of those already
 *
 * @details             MatchAddress calls are cumulative.
 *                      Specify address zero to match I2C "all call"
 *                      Does not support 10-bit addressing.
 *
 * @notapi
 **/
msg_t i2c_lld_matchAddress(I2CDriver *i2cp, i2caddr_t  i2cadr)
{
  (void)i2cp;
  (void)i2cadr;
  return I2C_OK;  /* or I2C_ERROR */
}


/**
 * @brief   Configure to ignore messages directed to the given i2cadr
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 * @param[in] i2cadr    I2C bus address
 *                      - @a 0 matches "all call"
 *                      .
 * @details A message being transferred that has already matched the
 *          specified address will continue being processed.
 *          Requests to unmatch an address that is not currently being matched
 *          are ignored.
 *
 * @notapi
 */
void i2cUnmatchAddress(I2CDriver *i2cp, i2caddr_t  i2cadr)
{
  (void)i2cp;
  (void)i2cadr;
}


/**
 * @brief   Reconfigure I2C channel to no longer match any address
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 *
 * @details   Causes all subsequent messages to be ignored.
 *            A message being transferred that has already matched a
 *            slave address will continue being processed.
 *
 * @notapi
 **/
void i2cUnmatchAll(I2CDriver *i2cp)
{
  (void)i2cp;
}


/**
 * @brief   Configure callbacks & buffers for message reception & query reply
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 * @param[in] rxMsg     @p I2CSlaveMsg struct for processing subsequent messages
 * @param[in] replyMsg  @p I2CSlaveMsg struct for processing subsequent queries
 *
 * @details             Call i2cMatchAddress() after this to start processing
 *     Enabling match addresses before installing handler callbacks can
 *     result in locking the I2C bus when a master accesses those
 *     unconfigured slave addresses
 *
 * @notapi
 */
void i2cSlaveConfigure(I2CDriver *i2cp,
                   const I2CSlaveMsg *rxMsg, const I2CSlaveMsg *replyMsg)
{
  (void)i2cp;
  (void)rxMsg;
  (void)replyMsg;
}


/**
 * @brief   Configure callbacks & buffers for query reply
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 * @param[in] replyMsg  @p I2CSlaveMsg struct for processing subsequent queries
 *
 * @details             Call i2cMatchAddress() after this to start processing
 *     Enabling match addresses before installing handler callbacks can
 *     result in locking the I2C bus when a master accesses those
 *     unconfigured slave addresses
 *
 * @notapi
 */
void i2cSlaveReceive(I2CDriver *i2cp, const I2CSlaveMsg *rxMsg)
{
  (void)i2cp;
  (void)rxMsg;
}


/**
 * @brief   Configure callbacks & buffers for query reply
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 * @param[in] replyMsg  @p I2CSlaveMsg struct for processing subsequent queries
 *
 * @details             Call i2cMatchAddress() after this to start processing
 *     Enabling match addresses before installing handler callbacks can
 *     result in locking the I2C bus when a master accesses those
 *     unconfigured slave addresses
 *
 * @notapi
 */
void i2cSlaveReply(I2CDriver *i2cp, const I2CSlaveMsg *replyMsg)
{
  (void)i2cp;
  (void)replyMsg;
}

#endif /* HAL_USE_I2C_SLAVE */

#endif /* HAL_USE_I2C */

/** @} */
