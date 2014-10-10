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
/*
   Concepts and parts of this file have been contributed by Uladzimir Pylinsky
   aka barthess.

   I2C Slave mode support added by Brent Roman (brent@mbari.org)
 */

/**
 * @file    STM32/I2Cv1/i2c_lld.c
 * @brief   STM32 I2C subsystem low level driver source.
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

#define I2C1_RX_DMA_CHANNEL                                                 \
  STM32_DMA_GETCHANNEL(STM32_I2C_I2C1_RX_DMA_STREAM,                        \
                       STM32_I2C1_RX_DMA_CHN)

#define I2C1_TX_DMA_CHANNEL                                                 \
  STM32_DMA_GETCHANNEL(STM32_I2C_I2C1_TX_DMA_STREAM,                        \
                       STM32_I2C1_TX_DMA_CHN)

#define I2C2_RX_DMA_CHANNEL                                                 \
  STM32_DMA_GETCHANNEL(STM32_I2C_I2C2_RX_DMA_STREAM,                        \
                       STM32_I2C2_RX_DMA_CHN)

#define I2C2_TX_DMA_CHANNEL                                                 \
  STM32_DMA_GETCHANNEL(STM32_I2C_I2C2_TX_DMA_STREAM,                        \
                       STM32_I2C2_TX_DMA_CHN)

#define I2C3_RX_DMA_CHANNEL                                                 \
  STM32_DMA_GETCHANNEL(STM32_I2C_I2C3_RX_DMA_STREAM,                        \
                       STM32_I2C3_RX_DMA_CHN)

#define I2C3_TX_DMA_CHANNEL                                                 \
  STM32_DMA_GETCHANNEL(STM32_I2C_I2C3_TX_DMA_STREAM,                        \
                       STM32_I2C3_TX_DMA_CHN)

/*===========================================================================*/
/* Driver constants -- see ST document RM0038 figure 208                     */
/*===========================================================================*/

#define I2C_EV5_MASTER_MODE_SELECT   \
  (((uint32_t)I2C_SR2_MSL << 16) | I2C_SR1_SB)

#define I2C_EV6_MASTER_TRA_MODE_SELECTED  \
  (((uint32_t)(I2C_SR2_MSL | I2C_SR2_TRA) << 16) | I2C_SR1_ADDR)

#define I2C_EV6_MASTER_REC_MODE_SELECTED  \
  (((uint32_t)I2C_SR2_MSL << 16) | I2C_SR1_ADDR)

#define I2C_EV8_2_MASTER_BYTE_TRANSMITTED \
  (((uint32_t)(I2C_SR2_MSL | I2C_SR2_TRA) << 16) | I2C_SR1_BTF)

#if HAL_USE_I2C_SLAVE
#define I2C_EV1_SLAVE_RXADRMATCH  \
  ((uint32_t)I2C_SR1_ADDR)

#define I2C_EV1_SLAVE_TXADRMATCH  \
  (((uint32_t)I2C_SR2_TRA << 16) | I2C_SR1_ADDR)

#define I2C_EV2_SLAVE_RXSTOP \
  (I2C_SR1_STOPF)
#endif

#define I2C_EV_MASK (  \
  ((uint32_t)(I2C_SR2_MSL|I2C_SR2_TRA)<<16) |           \
    (I2C_SR1_SB|I2C_SR1_ADDR|I2C_SR1_STOPF|I2C_SR1_BTF))

#define I2C_ERROR_MASK \
  (I2C_SR1_BERR | I2C_SR1_ARLO | I2C_SR1_AF | I2C_SR1_OVR |      \
     I2C_SR1_PECERR | I2C_SR1_TIMEOUT | I2C_SR1_SMBALERT)

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/** @brief I2C1 driver identifier.*/
#if STM32_I2C_USE_I2C1 || defined(__DOXYGEN__)
I2CDriver I2CD1;
#endif

/** @brief I2C2 driver identifier.*/
#if STM32_I2C_USE_I2C2 || defined(__DOXYGEN__)
I2CDriver I2CD2;
#endif

/** @brief I2C3 driver identifier.*/
#if STM32_I2C_USE_I2C3 || defined(__DOXYGEN__)
I2CDriver I2CD3;
#endif

/* quick and dirty queue to record event interrupts */
#define QEVENTS 32
#if QEVENTS > 0
uint16_t i2cQ[QEVENTS];
unsigned i2cI = QEVENTS;
#define qEvt(code) {if (++i2cI >= QEVENTS) i2cI = 0; i2cQ[i2cI]=(code);}
#else
#define qEvt(code)
#endif


#if HAL_USE_I2C_SLAVE   /* I2C slave mode support */

void I2CSlaveDummyCB(I2CDriver *i2cp)
/*
  dummy callback -- placeholder to ignore event
*/
{(void)i2cp;}

  /* lock bus on receive or reply message */
const I2CSlaveMsg I2CSlaveLockOnMsg = {
  0, NULL, I2CSlaveDummyCB, I2CSlaveDummyCB, I2CSlaveDummyCB
};

#endif

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/**
 * @brief   Resets and disables the I2C channel
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 *
 * @notapi
 */
static void i2cReset(I2CDriver *i2cp) {
  I2C_TypeDef *dp = i2cp->i2c;
  /* reset the I2C registers.*/
  dp->CR1 = I2C_CR1_SWRST;
  dp->CR1 = 0;

  /* Stop the associated DMA streams.*/
  dmaStreamDisable(i2cp->dmatx);
  dmaStreamDisable(i2cp->dmarx);
}

/**
 * @brief   Aborts an I2C transaction.
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 *
 * @notapi
 */
static void i2cAbortOperation(I2CDriver *i2cp) {
  /* save control registers */
  I2C_TypeDef *dp = i2cp->i2c;
  uint16_t cr2 = dp->CR2, cr1 = dp->CR1;
#if HAL_USE_I2C_SLAVE
  uint16_t oar1 = dp->OAR1, oar2 = dp->OAR2;
#endif
  uint16_t ccr = dp->CCR, trise = dp->TRISE;

  /* reset the I2C channel */
  i2cReset(i2cp);

  /* restore control registers */
  dp->TRISE = trise;  dp->CCR = ccr;
#if HAL_USE_I2C_SLAVE
  /* restore address mataching */
  dp->OAR1 = oar1; dp->OAR2 = oar2;
#endif

  /* Enable interrrupts */
  dp->CR2 = (cr2 & 0x3F) | I2C_CR2_ITERREN | I2C_CR2_DMAEN | I2C_CR2_ITEVTEN;

  /* Finish restoring and enable pheripheral */
  dp->CR1 = I2C_CR1_ACK | I2C_CR1_PE | (cr1 & (I2C_CR1_SMBUS | I2C_CR1_SMBTYPE))
#if HAL_USE_I2C_SLAVE
            | (cr1 & I2C_CR1_ENGC);
#endif
                                  ;
}


#if HAL_USE_I2C_SLAVE || HAL_USE_I2C_LOCK
/**
 * @brief   stop transaction timeout countdown
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 *
 * @notapi
 */
static INLINE void stopTimer(I2CDriver *i2cp)
{
  if (chVTIsArmedI(&i2cp->timer))
    chVTResetI(&i2cp->timer);
}
#else
#define stopTimer(ignored)  {}
#endif


#if HAL_USE_I2C_SLAVE   /* I2C slave mode support */

/**
 * @brief   return the address matched
 *
 * @param[in] dp      pointer to the @p I2C registers object
 * @param[in] sr2     I2C SR2 register contents
 *
 * @notapi
 *   Only supports 7-bit addressing for now
 */
static INLINE i2caddr_t matchedAdr(I2C_TypeDef *dp, uint32_t sr2) {
  if (sr2 & I2C_SR2_GENCALL)
    return 0;
  if (sr2 & I2C_SR2_DUALF)
    return (dp->OAR2>>1) & 0x7f;
  return (dp->OAR1>>1) & 0x7f;
}


/**
 * @brief   report error via slave exception callback
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 *
 * @note     moves back to the idle mode
 * @notapi
 */
static INLINE void reportSlaveError(I2CDriver *i2cp) {
  const I2CSlaveMsg *xfer = i2cp->mode >= i2cSlaveReplying ?
                                          i2cp->slaveReply : i2cp->slaveRx;
  xfer->exception(i2cp);  /* in this case, i2cp->slaveErrors == 0 */
  i2cp->mode = i2cIdle;
  i2cp->targetAdr = i2cInvalidAdr;
#if HAL_USE_I2C_STARTFIX
  i2cp->config->disarmStartDetect();
#endif
}


/**
 * @brief   Handling of stalled slave mode I2C transactions.
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 *
 * @notapi
 */
static void slaveTimeExpired(void *i2cv) {
  I2CDriver *i2cp = i2cv;

  i2cAbortOperation(i2cp);
  reportSlaveError(i2cp);
}


/**
 * @brief   start or restart slave mode transaction
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 *
 * @notapi
 */
static INLINE void startSlaveAction(I2CDriver *i2cp, i2caddr_t targetAdr)
{
  stopTimer(i2cp);
  i2cp->targetAdr = targetAdr;
  i2cp->slaveBytes = 0;
  i2cp->slaveErrors = 0;
  if (i2cp->slaveTimeout != TIME_INFINITE)
    chVTSetI(&i2cp->timer, i2cp->slaveTimeout, slaveTimeExpired, i2cp);
}

/**
 * @brief   end slave receive message DMA
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 *
 * @notapi
 */
static INLINE void endSlaveRxDMA(I2CDriver *i2cp)
{
  size_t bytesRemaining = dmaStreamGetTransactionSize(i2cp->dmarx);
  if (i2cp->slaveBytes)
    i2cp->slaveBytes += 0xffff - bytesRemaining;
  else
    i2cp->slaveBytes = i2cp->slaveRx->size - bytesRemaining;
  dmaStreamDisable(i2cp->dmarx);
}

/**
 * @brief   end slave transmit DMA
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 * @param[in] bytesRemaining  bytes lost in output queue
 *
 * @notapi
 */
static INLINE void endSlaveReplyDMA(I2CDriver *i2cp, size_t bytesRemaining)
{
  bytesRemaining += dmaStreamGetTransactionSize(i2cp->dmatx);
  if (i2cp->slaveBytes)
    i2cp->slaveBytes += 0xffff - bytesRemaining;
  else
    i2cp->slaveBytes = i2cp->slaveReply->size - bytesRemaining;
  dmaStreamDisable(i2cp->dmatx);
}

#endif


/**
 * @brief   Wakes up a waiting thread.
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 * @param[in] msg       wakeup message
 *
 * @notapi
 */
static INLINE void wakeup_isr(I2CDriver *i2cp, msg_t msg)
{
  chSysLockFromIsr();
  Thread *tp = i2cp->thread;
  if (tp != NULL) {
    i2cp->thread = NULL;
    tp->p_u.rdymsg = msg;
    chSchReadyI(tp);
  }
  chSysUnlockFromIsr();
}


/**
 * @brief   Set clock speed.
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 *
 * @notapi
 */
static void i2c_lld_set_clock(I2CDriver *i2cp) {
  I2C_TypeDef *dp = i2cp->i2c;
  uint16_t regCCR, clock_div;
  int32_t clock_speed = i2cp->config->clock_speed;
  i2cdutycycle_t duty = i2cp->config->duty_cycle;

  chDbgCheck((i2cp != NULL) && (clock_speed > 0) && (clock_speed <= 4000000),
             "i2c_lld_set_clock");

  /* CR2 Configuration.*/
  dp->CR2 &= ~I2C_CR2_FREQ;
  dp->CR2 |= I2C_CLK_FREQ;

  /* CCR Configuration.*/
  regCCR = 0;
  clock_div = I2C_CCR_CCR;

  switch (duty) {
    case STD_DUTY_CYCLE:
      /* Standard mode clock_div calculate: Tlow/Thigh = 1/1.*/
#ifndef STM32_I2C_LOOSE_CLK
      /* Configure clock_div in standard mode.*/
      chDbgAssert(clock_speed <= 100000,
                  "i2c_lld_set_clock(), #1",
                  "STD_DUTY_CYCLE limited to 100khz");
      chDbgAssert(STM32_PCLK1 % (clock_speed * 2) == 0,
                  "i2c_lld_set_clock(), #2",
                  "PCLK1 not divisible by 2*I2Cclk");
#endif
      clock_div = STM32_PCLK1 / (clock_speed * 2);

      chDbgAssert(clock_div >= 4,
                  "i2c_lld_set_clock(), #3",
                  "Clock divider < 4 not allowed");
      regCCR |= (clock_div & I2C_CCR_CCR);

      /* Sets the Maximum Rise Time for standard mode.*/
      dp->TRISE = I2C_CLK_FREQ + 1;
      break;

    case FAST_DUTY_CYCLE_2:
    case FAST_DUTY_CYCLE_16_9:
      /* Configure clock_div in fast mode.*/
#ifndef STM32_I2C_LOOSE_CLK
      chDbgAssert(clock_speed > 100000 && clock_speed <= 400000,
                  "i2c_lld_set_clock(), #4",
                  "I2Cclk out of range for FAST_DUTY_CYCLE");
#endif
      if (duty == FAST_DUTY_CYCLE_2) {
        /* Fast mode clock_div calculate: Tlow/Thigh = 2/1.*/
#ifndef STM32_I2C_LOOSE_CLK
        chDbgAssert((STM32_PCLK1 % (clock_speed * 3)) == 0,
                    "i2c_lld_set_clock(), #6",
                    "PCLK1 not divisible by 3*I2Cclk");
#endif
        clock_div = STM32_PCLK1 / (clock_speed * 3);
      }else{ /* FAST_DUTY_CYCLE_16_9 */
        /* Fast mode clock_div calculate: Tlow/Thigh = 16/9.*/
#ifndef STM32_I2C_LOOSE_CLK
        chDbgAssert(STM32_PCLK1 % (clock_speed * 25) == 0,
                    "i2c_lld_set_clock(), #7",
                    "PCLK1 not divisible by 25*I2Cclk");
#endif
        clock_div = STM32_PCLK1 / (clock_speed * 25);
        regCCR |= I2C_CCR_DUTY;
      }
      chDbgAssert(clock_div >= 1,
                      "i2c_lld_set_clock(), #8",
                      "Clock divider less than 4 not allowed");
      regCCR |= (I2C_CCR_FS | (clock_div & I2C_CCR_CCR));

      /* Sets the Maximum Rise Time for fast mode.*/
      dp->TRISE = (I2C_CLK_FREQ * 300 / 1000) + 1;
      break;

    default:
      chDbgPanic("Invalid I2C duty_cycle");
  }
  chDbgAssert((clock_div <= I2C_CCR_CCR),
              "i2c_lld_set_clock(), #9", "I2Cclk is too slow");
  dp->CCR = regCCR;
}


/**
 * @brief   Set operation mode of I2C hardware.
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 *
 * @notapi
 */
static void i2c_lld_set_opmode(I2CDriver *i2cp) {
  I2C_TypeDef *dp = i2cp->i2c;
  i2copmode_t opmode = i2cp->config->op_mode;
  uint16_t regCR1;

  regCR1 = dp->CR1;
  switch (opmode) {
    case OPMODE_I2C:
      regCR1 &= ~(I2C_CR1_SMBUS|I2C_CR1_SMBTYPE);
      break;
    case OPMODE_SMBUS_DEVICE:
      regCR1 |= I2C_CR1_SMBUS;
      regCR1 &= ~(I2C_CR1_SMBTYPE);
      break;
    case OPMODE_SMBUS_HOST:
      regCR1 |= I2C_CR1_SMBUS|I2C_CR1_SMBTYPE;
  }
  dp->CR1 = regCR1;
}


/**
 * @brief   Handling of stalled master mode I2C transactions.
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 *
 * @notapi
 */
static void i2c_lld_safety_timeout(void *p) {
  I2CDriver *i2cp = p;

  stopTimer(i2cp);
#if HAL_USE_I2C_SLAVE             /* abort any slave operation in progress */
  if (!(i2cp->i2c->SR2 & I2C_SR2_MSL))
    slaveTimeExpired(i2cp);  /* in case slave preventing master bus access */
  else
#endif
  {
    i2cAbortOperation(i2cp);
    i2cp->mode = i2cIdle;
  }
  wakeup_isr(i2cp, I2C_TIMEOUT);
}


#if HAL_USE_I2C_LOCK    /* I2C bus locking support */

/**
 * @brief   Handling of expired master bus lock timer
 *
 * @param[in] i2cv      pointer to the @p I2CDriver object
 *
 * @notapi
 */
static void lockExpired(void *i2cv) {
  I2CDriver *i2cp = i2cv;

  if (i2cp->mode == i2cIsMaster && !i2cp->thread) {  /* between transactions */
    i2cp->i2c->CR1 |= I2C_CR1_STOP | I2C_CR1_ACK;
    i2cp->mode = i2cIdle;
  }
  i2cp->lockDuration = TIME_IMMEDIATE;
}


/**
 * @brief   Lock I2C bus at the beginning of the next message
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 * @param[in] lockDuration   max number of ticks to hold bus locked
 *                      - @a TIME_INFINITE no timeout.
 *                      - @a TIME_IMMEDIATE unlock the bus immediately
 *                      .
 *
 *  Lock I2C bus at the beginning of the next message sent
 *  for a maximum of lockDuration ticks.  No other I2C masters will
 *  be allowed to interrupt until i2cUnlock() is called.
 *
 * @notapi
 **/
void i2c_lld_lock(I2CDriver *i2cp, systime_t lockDuration)
{
  i2cp->lockDuration = lockDuration;
  if (i2cp->mode >= i2cIsMaster) {
    stopTimer(i2cp);
    if (lockDuration == TIME_IMMEDIATE)
      lockExpired(i2cp);
    else if (lockDuration != TIME_INFINITE)
      chVTSetI(&i2cp->timer, lockDuration, lockExpired, i2cp);
  }
}

#endif



/**
 * @brief   report error waking thread or invoking callback as appropriate
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 * @param[in] errCode   error bit mask or code
 *
 * @notapi
 */
static void reportErrs(I2CDriver *i2cp, i2cflags_t errCode)
{
qEvt(0xee00 | errCode);
  I2C_TypeDef *dp = i2cp->i2c;
  if (i2cp->mode <= i2cIdle)  /* failing to master bus */
    i2cAbortOperation(i2cp);
  else if (dp->SR2 & I2C_SR2_MSL) {
#if HAL_USE_I2C_LOCK    /* I2C bus locking support */
    i2cp->mode = i2cIsMaster;
    switch (i2cp->lockDuration) {
      case TIME_INFINITE:
        break;
      case TIME_IMMEDIATE:
        stopTimer(i2cp);
      default:
        if (!chVTIsArmedI(&i2cp->timer)) {
          dp->CR1 |= I2C_CR1_STOP | I2C_CR1_ACK;
          i2cp->mode = i2cIdle;
          i2cp->lockDuration = TIME_IMMEDIATE;
        }
    }
#else  /* signal stop condition on any error */
    dp->CR1 |= I2C_CR1_STOP | I2C_CR1_ACK;
    i2cp->mode = i2cIdle;
#endif
  }
#if HAL_USE_I2C_SLAVE
  else if (i2cp->mode < i2cIsMaster) {
    i2cp->slaveErrors = errCode;
    i2cAbortOperation(i2cp);
    stopTimer(i2cp);
    reportSlaveError(i2cp);
    return;
  }
#endif
  /* wake any waiting master mode handling thread. */
  i2cp->errors = errCode;
  wakeup_isr(i2cp, I2C_ERROR);
}


/**
 * @brief   I2C error handler.
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 * @param[in]  sr       content of the SR1 register to be decoded
 *
 * @notapi
 */
static void i2c_lld_serve_error_interrupt(I2CDriver *i2cp, uint16_t sr) {
#if HAL_USE_I2C_SLAVE
/* NACK of last byte transmitted in slave response is NORMAL -- not an error! */
  if (i2cp->mode == i2cSlaveReplying && (sr & I2C_ERROR_MASK) == I2C_SR1_AF) {
qEvt(0xcccc);
    endSlaveReplyDMA(i2cp, 1);
    i2cp->slaveReply->processMsg(i2cp);
    i2cp->targetAdr = i2cInvalidAdr;
    stopTimer(i2cp);
    i2cp->mode = i2cIdle;
#if HAL_USE_I2C_STARTFIX
    i2cp->config->disarmStartDetect();
#endif
    return;
  }
#endif
  i2cflags_t errs = 0;

  if (sr & I2C_SR1_BERR)                            /* Bus error.           */
    errs = I2CD_BUS_ERROR;

  if (sr & I2C_SR1_ARLO)                            /* Arbitration lost.    */
    errs |= I2CD_ARBITRATION_LOST;

  if (sr & I2C_SR1_AF)                              /* Acknowledge fail.    */
    errs |= I2CD_ACK_FAILURE;

  if (sr & I2C_SR1_OVR)                             /* Overrun.             */
    errs |= I2CD_OVERRUN;

  if (sr & I2C_SR1_TIMEOUT)                         /* SMBus Timeout.       */
    errs |= I2CD_TIMEOUT;

  if (sr & I2C_SR1_PECERR)                          /* PEC error.           */
    errs |= I2CD_PEC_ERROR;

  if (sr & I2C_SR1_SMBALERT)                        /* SMBus alert.         */
    errs |= I2CD_SMB_ALERT;

  if (!errs)
    errs = I2CD_UNKNOWN_ERROR;

  /* Disable any active DMA */
  dmaStreamDisable(i2cp->dmatx);
  dmaStreamDisable(i2cp->dmarx);

  reportErrs(i2cp, errs);
}


static INLINE void endMasterAction(I2CDriver *i2cp, uint32_t regCR1)
{
#if HAL_USE_I2C_LOCK
  if (i2cp->lockDuration != TIME_IMMEDIATE && (
      chVTIsArmedI(&i2cp->timer) || i2cp->lockDuration == TIME_INFINITE)) {
    i2cp->mode = i2cIsMaster;
    return;
  }
  stopTimer(i2cp);
#endif
  i2cp->i2c->CR1 = regCR1 | I2C_CR1_STOP | I2C_CR1_ACK;
  i2cp->mode = i2cIdle;
}


/**
 * @brief   I2C event handler ISR
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 *
 * @notapi
 */
static void i2c_lld_serve_event_interrupt(I2CDriver *i2cp)
{
  I2C_TypeDef *dp = i2cp->i2c;
  uint16_t regCR1 = dp->CR1;
  uint32_t regSR2 = dp->SR2;
  uint32_t event = dp->SR1 | (regSR2 << 16);

  switch (event & I2C_EV_MASK) {

#define chkTransition(expectedMode) { \
   if (i2cp->mode != (expectedMode)) goto invalidTransition;}

   invalidTransition:   /* error on known event out of expected sequence */
    i2cAbortOperation(i2cp);        /* reset and reinit */
    reportErrs(i2cp, I2CD_UNKNOWN_ERROR + i2cp->mode);
    break;

#if HAL_USE_I2C_SLAVE
   case I2C_EV1_SLAVE_RXADRMATCH:
qEvt(0x1111);
   {
     i2caddr_t targetAdr = matchedAdr(dp, regSR2);
     switch (i2cp->mode) {
       case i2cIdle:
#if HAL_USE_I2C_STARTFIX
         i2cp->config->armStartDetect();
#endif
         break;
       case i2cSlaveRxing:
         endSlaveRxDMA(i2cp);
         i2cp->slaveRx->processMsg(i2cp);
         break;
       case i2cSlaveReplying:   /* Master did not NACK last transmitted byte */
         endSlaveReplyDMA(i2cp, 2);
         i2cp->slaveReply->processMsg(i2cp);
         break;
       default:
         goto invalidTransition;
     }
     startSlaveAction(i2cp, targetAdr);
   }
   {
      const I2CSlaveMsg *rx = i2cp->slaveNextRx;
      rx->adrMatched(i2cp);
      rx = i2cp->slaveRx = i2cp->slaveNextRx;
      if (rx->body && rx->size) {
        (void)dp->SR2;  /* clear I2C_SR1_ADDR */
         /* slave RX DMA setup.*/
        dmaStreamSetMode(i2cp->dmarx, i2cp->rxdmamode);
        dmaStreamSetMemory0(i2cp->dmarx, rx->body);
        dmaStreamSetTransactionSize(i2cp->dmarx, rx->size);
        dmaStreamEnable(i2cp->dmarx);
        i2cp->mode = i2cSlaveRxing;
      }else{
        dp->CR2 &= ~I2C_CR2_ITEVTEN;
        i2cp->mode = i2cLockedRxing;
      }
    }
    break;

   case I2C_EV2_SLAVE_RXSTOP:
qEvt(0x2222);
    dp->CR1 = regCR1;            /* clear STOPF */
    i2cp->slaveErrors = I2CD_STOPPED; /* indicate that bus has been released */
    switch (i2cp->mode) {
      case i2cSlaveRxing:
        endSlaveRxDMA(i2cp);
        i2cp->slaveRx->processMsg(i2cp);
        break;
      case i2cSlaveReplying:    /* Master did not NACK last transmitted byte */
        endSlaveReplyDMA(i2cp, 2);
        i2cp->slaveReply->processMsg(i2cp);
        break;
      default:
        goto invalidTransition;
    }
    i2cp->targetAdr = i2cInvalidAdr;
    stopTimer(i2cp);
    i2cp->mode = i2cIdle;
#if HAL_USE_I2C_STARTFIX
    i2cp->config->disarmStartDetect();
#endif
    break;

   case I2C_EV1_SLAVE_TXADRMATCH:
qEvt(0x3333);
   {
      i2caddr_t targetAdr = matchedAdr(dp, regSR2);
      (void)dp->SR2;  /* clear I2C_SR1_ADDR */
      switch (i2cp->mode) {
        case i2cIdle:
#if HAL_USE_I2C_STARTFIX
          i2cp->config->armStartDetect();
#endif
          break;
        case i2cSlaveRxing:
          endSlaveRxDMA(i2cp);
          i2cp->slaveRx->processMsg(i2cp);
          break;
        default:
          goto invalidTransition;
      }
      startSlaveAction(i2cp, targetAdr);
   }
   {
      const I2CSlaveMsg *reply = i2cp->slaveNextReply;
      reply->adrMatched(i2cp);
      reply = i2cp->slaveReply = i2cp->slaveNextReply;
      if (reply->body && reply->size) {
        /* slave TX DMA setup.*/
        dmaStreamSetMode(i2cp->dmatx, i2cp->txdmamode);
        dmaStreamSetMemory0(i2cp->dmatx, reply->body);
        dmaStreamSetTransactionSize(i2cp->dmatx, reply->size);
        dmaStreamEnable(i2cp->dmatx);
        i2cp->mode = i2cSlaveReplying;
      }else{
        dp->CR2 &= ~I2C_CR2_ITEVTEN;
        i2cp->mode = i2cLockedReplying;
      }
    }
    break;
#endif  /* HAL_USE_I2C_SLAVE */

   case I2C_EV5_MASTER_MODE_SELECT:
qEvt(0x5555);
    dp->DR = i2cp->addr;
    switch (i2cp->mode) {
      case i2cIdle:
#if HAL_USE_I2C_LOCK
      {
        systime_t lockDuration = i2cp->lockDuration;
        if (lockDuration != TIME_IMMEDIATE && lockDuration != TIME_INFINITE)
          chVTSetI(&i2cp->timer, lockDuration, lockExpired, i2cp);
      }
#endif
        break;
      case i2cIsMaster:
      case i2cMasterStarted:
        break;
      default:
        goto invalidTransition;
    }
    i2cp->mode = i2cMasterSelecting;
    break;

   case I2C_EV6_MASTER_REC_MODE_SELECTED:
qEvt(0x6666);
    chkTransition(i2cMasterSelecting);
    if (!i2cp->masterRxbytes) {  /* 0-length SMBus style quick read */
      endMasterAction(i2cp, regCR1);
      (void)dp->SR2;  /* clear I2C_SR1_ADDR */
      wakeup_isr(i2cp, I2C_OK);
      break;
    }
    (void)dp->SR2;  /* clear I2C_SR1_ADDR */
    /* RX DMA setup.*/
    dmaStreamSetMode(i2cp->dmarx, i2cp->rxdmamode);
    dmaStreamSetMemory0(i2cp->dmarx, i2cp->masterRxbuf);
    dmaStreamSetTransactionSize(i2cp->dmarx, i2cp->masterRxbytes);
    dmaStreamEnable(i2cp->dmarx);
    dp->CR2 |= I2C_CR2_LAST;                 /* Needed in receiver mode. */
    if (i2cp->masterRxbytes < 2)
      dp->CR1 = regCR1 & ~I2C_CR1_ACK;
    i2cp->mode = i2cMasterRxing;
    break;

   case I2C_EV6_MASTER_TRA_MODE_SELECTED:
qEvt(0x7777);
    (void)dp->SR2;  /* clear I2C_SR1_ADDR */
    chkTransition(i2cMasterSelecting);
    switch (i2cp->masterTxbytes) {
      case 0:
        goto doneWriting;
      case 1:
        dp->DR = i2cp->masterTxbuf[0];
        break;
      case 2:
        dp->DR = i2cp->masterTxbuf[0];
        dp->DR = i2cp->masterTxbuf[1];
        break;
      default:
        /* TX DMA setup.*/
        dmaStreamSetMode(i2cp->dmatx, i2cp->txdmamode);
        dmaStreamSetMemory0(i2cp->dmatx, i2cp->masterTxbuf);
        dmaStreamSetTransactionSize(i2cp->dmatx, i2cp->masterTxbytes);
        dmaStreamEnable(i2cp->dmatx);
    }
    i2cp->mode = i2cMasterTxing;
    break;

   case I2C_EV8_2_MASTER_BYTE_TRANSMITTED:
qEvt(0x8888);
    /* Catches BTF event after the end of transmission.*/
    (void)dp->DR;  /* clears BTF flag */
    chkTransition(i2cMasterTxing);
doneWriting:
    if (i2cp->masterRxbuf) {
      /* Starts "read after write" operation, LSB = 1 -> receive.*/
      dp->CR1 = regCR1 | I2C_CR1_START | I2C_CR1_ACK;
      i2cp->addr |= 1;
      i2cp->mode = i2cMasterStarted;
    }else{
      endMasterAction(i2cp, regCR1);
      wakeup_isr(i2cp, I2C_OK);
    }
    break;

   default:  /* unhandled event -- abort transaction, flag unknown err */
qEvt(0x9999);
    i2cAbortOperation(i2cp);
    i2c_lld_serve_error_interrupt(i2cp, event);
  }
}


#if HAL_USE_I2C_STARTFIX
/**
 * @brief   external device detected start condition
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 *
 * @details invoked from ISR if a START CONDITION detected during the time
 *          the startDetector is armed.
 *          This is a workaround for the STM32's lack of a repeated start event
 *
 * @notapi
 */
void  i2c_lld_startDetected(I2CDriver *i2cp)
{
qEvt(0xdddd);
  switch (i2cp->mode) {
    case i2cSlaveRxing:
      endSlaveRxDMA(i2cp);
      i2cp->slaveRx->processMsg(i2cp);
      break;
    case i2cSlaveReplying:    /* Master did not NACK last transmitted byte */
      endSlaveReplyDMA(i2cp, 2);
      i2cp->slaveReply->processMsg(i2cp);
      break;
    default:
      i2cAbortOperation(i2cp);        /* reset and reinit */
      reportErrs(i2cp, I2CD_UNKNOWN_ERROR + i2cp->mode);
      return;
  }
  i2cp->targetAdr = i2cInvalidAdr;
  stopTimer(i2cp);
  i2cp->mode = i2cIdle;
}

/**
 * @brief   dummy placeholder for armStartDetector() and disarmStartDetector()
 *
 * @details *MUST* be placed in the configuration struct in cases where
 *          HAL_USE_I2C_STARTFIX is configured and no startDetector is defined,
 *          otherwise NULL pointers will be called for these functions!
 *
 * @notapi
 */
void  i2c_lld_noStartDetector(void) {}
#endif


/**
 * @brief   DMA RX end IRQ handler.
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 * @param[in] flags     pre-shifted content of the ISR register
 *
 * @notapi
 */
static void i2c_lld_serve_rx_end_irq(I2CDriver *i2cp, uint32_t flags) {
  I2C_TypeDef *dp = i2cp->i2c;

qEvt(0xaaaa);
  /* DMA errors handling.*/
#if defined(STM32_I2C_DMA_ERROR_HOOK)
  if ((flags & (STM32_DMA_ISR_TEIF | STM32_DMA_ISR_DMEIF)) != 0) {
    STM32_I2C_DMA_ERROR_HOOK(i2cp);
  }
#else
  (void)flags;
#endif

  dmaStreamDisable(i2cp->dmarx);

#if HAL_USE_I2C_SLAVE
  if (i2cp->mode < i2cIsMaster) {
    static uint8_t bitbucket;
    if (i2cp->slaveBytes)
      i2cp->slaveBytes += 0xffff;
    else
      i2cp->slaveBytes = i2cp->slaveRx->size;
    /* discard data overrunning available rx buffer, but record total length */
    dmaStreamSetMode(i2cp->dmarx, i2cp->rxdmamode & ~STM32_DMA_CR_MINC);
    dmaStreamSetMemory0(i2cp->dmarx, &bitbucket);
    dmaStreamSetTransactionSize(i2cp->dmarx, 0xffff);
    dmaStreamEnable(i2cp->dmarx);
    return;
  }
#endif

  dp->CR2 &= ~I2C_CR2_LAST;
  endMasterAction(i2cp, dp->CR1);
  wakeup_isr(i2cp, I2C_OK);
}

/**
 * @brief    DMA TX end IRQ handler.
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 *
 * @notapi
 */
static void i2c_lld_serve_tx_end_irq(I2CDriver *i2cp, uint32_t flags) {
qEvt(0xbbbb);
  /* DMA errors handling.*/
#if defined(STM32_I2C_DMA_ERROR_HOOK)
  if ((flags & (STM32_DMA_ISR_TEIF | STM32_DMA_ISR_DMEIF)) != 0) {
    STM32_I2C_DMA_ERROR_HOOK(i2cp);
  }
#else
  (void)flags;
#endif
  dmaStreamDisable(i2cp->dmatx);

#if HAL_USE_I2C_SLAVE
  if (i2cp->mode < i2cIsMaster) {
    const I2CSlaveMsg *reply = i2cp->slaveReply;
    if (i2cp->slaveBytes)
      i2cp->slaveBytes += 0xffff;
    else
      i2cp->slaveBytes = reply->size;
    /* repeat the last byte in the reply */
    dmaStreamSetMode(i2cp->dmatx, i2cp->txdmamode & ~STM32_DMA_CR_MINC);
    dmaStreamSetMemory0(i2cp->dmatx, reply->body+reply->size-1);
    dmaStreamSetTransactionSize(i2cp->dmatx, 0xffff);
    dmaStreamEnable(i2cp->dmatx);
  }
#endif
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#if STM32_I2C_USE_I2C1 || defined(__DOXYGEN__)
/**
 * @brief   I2C1 event interrupt handler.
 *
 * @notapi
 */
CH_IRQ_HANDLER(I2C1_EV_IRQHandler) {

  CH_IRQ_PROLOGUE();

  i2c_lld_serve_event_interrupt(&I2CD1);

  CH_IRQ_EPILOGUE();
}

/**
 * @brief   I2C1 error interrupt handler.
 */
CH_IRQ_HANDLER(I2C1_ER_IRQHandler) {
  uint16_t sr = I2CD1.i2c->SR1;

  CH_IRQ_PROLOGUE();

  I2CD1.i2c->SR1 = ~(sr & I2C_ERROR_MASK);
  i2c_lld_serve_error_interrupt(&I2CD1, sr);

  CH_IRQ_EPILOGUE();
}
#endif /* STM32_I2C_USE_I2C1 */

#if STM32_I2C_USE_I2C2 || defined(__DOXYGEN__)
/**
 * @brief   I2C2 event interrupt handler.
 *
 * @notapi
 */
CH_IRQ_HANDLER(I2C2_EV_IRQHandler) {

  CH_IRQ_PROLOGUE();

  i2c_lld_serve_event_interrupt(&I2CD2);

  CH_IRQ_EPILOGUE();
}

/**
 * @brief   I2C2 error interrupt handler.
 *
 * @notapi
 */
CH_IRQ_HANDLER(I2C2_ER_IRQHandler) {
  uint16_t sr = I2CD2.i2c->SR1;

  CH_IRQ_PROLOGUE();

  I2CD2.i2c->SR1 = ~(sr & I2C_ERROR_MASK);
  i2c_lld_serve_error_interrupt(&I2CD2, sr);

  CH_IRQ_EPILOGUE();
}
#endif /* STM32_I2C_USE_I2C2 */

#if STM32_I2C_USE_I2C3 || defined(__DOXYGEN__)
/**
 * @brief   I2C3 event interrupt handler.
 *
 * @notapi
 */
CH_IRQ_HANDLER(I2C3_EV_IRQHandler) {

  CH_IRQ_PROLOGUE();

  i2c_lld_serve_event_interrupt(&I2CD3);

  CH_IRQ_EPILOGUE();
}

/**
 * @brief   I2C3 error interrupt handler.
 *
 * @notapi
 */
CH_IRQ_HANDLER(I2C3_ER_IRQHandler) {
  uint16_t sr = I2CD3.i2c->SR1;

  CH_IRQ_PROLOGUE();

  I2CD3.i2c->SR1 = ~(sr & I2C_ERROR_MASK);
  i2c_lld_serve_error_interrupt(&I2CD3, sr);

  CH_IRQ_EPILOGUE();
}
#endif /* STM32_I2C_USE_I2C3 */

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level I2C driver initialization.
 *
 * @notapi
 */
void i2c_lld_init(void) {

#if STM32_I2C_USE_I2C1
  i2cObjectInit(&I2CD1);
  I2CD1.thread = NULL;
  I2CD1.i2c    = I2C1;
  I2CD1.dmarx  = STM32_DMA_STREAM(STM32_I2C_I2C1_RX_DMA_STREAM);
  I2CD1.dmatx  = STM32_DMA_STREAM(STM32_I2C_I2C1_TX_DMA_STREAM);
#if HAL_USE_I2C_LOCK || HAL_USE_I2C_SLAVE
  chVTInit(&I2CD1.timer);
#endif
#endif /* STM32_I2C_USE_I2C1 */

#if STM32_I2C_USE_I2C2
  i2cObjectInit(&I2CD2);
  I2CD2.thread = NULL;
  I2CD2.i2c    = I2C2;
  I2CD2.dmarx  = STM32_DMA_STREAM(STM32_I2C_I2C2_RX_DMA_STREAM);
  I2CD2.dmatx  = STM32_DMA_STREAM(STM32_I2C_I2C2_TX_DMA_STREAM);
#if HAL_USE_I2C_LOCK || HAL_USE_I2C_SLAVE
  chVTInit(&I2CD2.timer);
#endif
#endif /* STM32_I2C_USE_I2C2 */

#if STM32_I2C_USE_I2C3
  i2cObjectInit(&I2CD3);
  I2CD3.thread = NULL;
  I2CD3.i2c    = I2C3;
  I2CD3.dmarx  = STM32_DMA_STREAM(STM32_I2C_I2C3_RX_DMA_STREAM);
  I2CD3.dmatx  = STM32_DMA_STREAM(STM32_I2C_I2C3_TX_DMA_STREAM);
#if HAL_USE_I2C_LOCK || HAL_USE_I2C_SLAVE
  chVTInit(&I2CD3.timer);
#endif
#endif /* STM32_I2C_USE_I2C3 */
}

/**
 * @brief   Configures and activates the I2C peripheral.
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 *
 * @notapi
 */
void i2c_lld_start(I2CDriver *i2cp) {
  I2C_TypeDef *dp = i2cp->i2c;

  /* If in stopped state then enable the I2C and DMA clocks.*/
  if (i2cp->state == I2C_STOP) {
    i2cp->txdmamode = STM32_DMA_CR_PSIZE_BYTE | STM32_DMA_CR_MSIZE_BYTE |
                      STM32_DMA_CR_MINC       | STM32_DMA_CR_DMEIE |
                      STM32_DMA_CR_TEIE       | STM32_DMA_CR_TCIE |
                      STM32_DMA_CR_DIR_M2P;
    i2cp->rxdmamode = STM32_DMA_CR_PSIZE_BYTE | STM32_DMA_CR_MSIZE_BYTE |
                      STM32_DMA_CR_MINC       | STM32_DMA_CR_DMEIE |
                      STM32_DMA_CR_TEIE       | STM32_DMA_CR_TCIE |
                      STM32_DMA_CR_DIR_P2M;

#if STM32_I2C_USE_I2C1
    if (&I2CD1 == i2cp) {
      bool_t b = dmaStreamAllocate(i2cp->dmarx, STM32_I2C_I2C1_IRQ_PRIORITY,
                            (stm32_dmaisr_t)i2c_lld_serve_rx_end_irq, i2cp);
      chDbgAssert(!b, "i2c_lld_start(), #1", "stream already allocated");
      b = dmaStreamAllocate(i2cp->dmatx, STM32_I2C_I2C1_IRQ_PRIORITY,
                            (stm32_dmaisr_t)i2c_lld_serve_tx_end_irq, i2cp);
      chDbgAssert(!b, "i2c_lld_start(), #2", "stream already allocated");
      rccEnableI2C1(FALSE);
      nvicEnableVector(I2C1_EV_IRQn,
          CORTEX_PRIORITY_MASK(STM32_I2C_I2C1_IRQ_PRIORITY));
      nvicEnableVector(I2C1_ER_IRQn,
          CORTEX_PRIORITY_MASK(STM32_I2C_I2C1_IRQ_PRIORITY));

      i2cp->rxdmamode |= STM32_DMA_CR_CHSEL(I2C1_RX_DMA_CHANNEL) |
                       STM32_DMA_CR_PL(STM32_I2C_I2C1_DMA_PRIORITY);
      i2cp->txdmamode |= STM32_DMA_CR_CHSEL(I2C1_TX_DMA_CHANNEL) |
                       STM32_DMA_CR_PL(STM32_I2C_I2C1_DMA_PRIORITY);
    }
#endif /* STM32_I2C_USE_I2C1 */

#if STM32_I2C_USE_I2C2
    if (&I2CD2 == i2cp) {
      bool_t b = dmaStreamAllocate(i2cp->dmarx, STM32_I2C_I2C2_IRQ_PRIORITY,
                            (stm32_dmaisr_t)i2c_lld_serve_rx_end_irq, i2cp);
      chDbgAssert(!b, "i2c_lld_start(), #3", "stream already allocated");
      b = dmaStreamAllocate(i2cp->dmatx, STM32_I2C_I2C2_IRQ_PRIORITY,
                            (stm32_dmaisr_t)i2c_lld_serve_tx_end_irq, i2cp);
      chDbgAssert(!b, "i2c_lld_start(), #4", "stream already allocated");
      rccEnableI2C2(FALSE);
      nvicEnableVector(I2C2_EV_IRQn,
          CORTEX_PRIORITY_MASK(STM32_I2C_I2C2_IRQ_PRIORITY));
      nvicEnableVector(I2C2_ER_IRQn,
          CORTEX_PRIORITY_MASK(STM32_I2C_I2C2_IRQ_PRIORITY));

      i2cp->rxdmamode |= STM32_DMA_CR_CHSEL(I2C2_RX_DMA_CHANNEL) |
                       STM32_DMA_CR_PL(STM32_I2C_I2C2_DMA_PRIORITY);
      i2cp->txdmamode |= STM32_DMA_CR_CHSEL(I2C2_TX_DMA_CHANNEL) |
                       STM32_DMA_CR_PL(STM32_I2C_I2C2_DMA_PRIORITY);
    }
#endif /* STM32_I2C_USE_I2C2 */

#if STM32_I2C_USE_I2C3
    if (&I2CD3 == i2cp) {
      bool_t b = dmaStreamAllocate(i2cp->dmarx, STM32_I2C_I2C3_IRQ_PRIORITY,
                            (stm32_dmaisr_t)i2c_lld_serve_rx_end_irq, i2cp);
      chDbgAssert(!b, "i2c_lld_start(), #5", "stream already allocated");
      b = dmaStreamAllocate(i2cp->dmatx, STM32_I2C_I2C3_IRQ_PRIORITY,
                            (stm32_dmaisr_t)i2c_lld_serve_tx_end_irq, i2cp);
      chDbgAssert(!b, "i2c_lld_start(), #6", "stream already allocated");
      rccEnableI2C3(FALSE);
      nvicEnableVector(I2C3_EV_IRQn,
          CORTEX_PRIORITY_MASK(STM32_I2C_I2C3_IRQ_PRIORITY));
      nvicEnableVector(I2C3_ER_IRQn,
          CORTEX_PRIORITY_MASK(STM32_I2C_I2C3_IRQ_PRIORITY));

      i2cp->rxdmamode |= STM32_DMA_CR_CHSEL(I2C3_RX_DMA_CHANNEL) |
                       STM32_DMA_CR_PL(STM32_I2C_I2C3_DMA_PRIORITY);
      i2cp->txdmamode |= STM32_DMA_CR_CHSEL(I2C3_TX_DMA_CHANNEL) |
                       STM32_DMA_CR_PL(STM32_I2C_I2C3_DMA_PRIORITY);
    }
#endif /* STM32_I2C_USE_I2C3 */

    /* I2C registers pointed by the DMA.*/
    dmaStreamSetPeripheral(i2cp->dmarx, &dp->DR);
    dmaStreamSetPeripheral(i2cp->dmatx, &dp->DR);

    i2cp->mode = i2cIdle;
#if HAL_USE_I2C_LOCK
    i2cp->lockDuration = TIME_IMMEDIATE;
#endif
#if HAL_USE_I2C_SLAVE   /* I2C slave mode support */
    i2cp->slaveNextReply = i2cp->slaveNextRx = &I2CSlaveLockOnMsg;
    i2cp->targetAdr = i2cInvalidAdr;
    i2cp->slaveTimeout = TIME_INFINITE;
#endif

    /* Setup I2C parameters.*/
    i2c_lld_set_clock(i2cp);
    i2c_lld_set_opmode(i2cp);

    /* Enable interrrupts */
    dp->CR2 |= I2C_CR2_ITERREN | I2C_CR2_DMAEN | I2C_CR2_ITEVTEN;
    /* Ready to go.*/
    dp->CR1 |= I2C_CR1_PE | I2C_CR1_ACK;
  }  /* ignore start if state is not stopped */
}

/**
 * @brief   Deactivates the I2C peripheral.
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 *
 * @notapi
 */
void i2c_lld_stop(I2CDriver *i2cp) {

  /* If not in stopped state, then disable the I2C clock.*/
  if (i2cp->state != I2C_STOP) {

    /* I2C disable.*/
    i2cReset(i2cp);
    dmaStreamRelease(i2cp->dmatx);
    dmaStreamRelease(i2cp->dmarx);

#if STM32_I2C_USE_I2C1
    if (&I2CD1 == i2cp) {
      nvicDisableVector(I2C1_EV_IRQn);
      nvicDisableVector(I2C1_ER_IRQn);
      rccDisableI2C1(FALSE);
    }
#endif

#if STM32_I2C_USE_I2C2
    if (&I2CD2 == i2cp) {
      nvicDisableVector(I2C2_EV_IRQn);
      nvicDisableVector(I2C2_ER_IRQn);
      rccDisableI2C2(FALSE);
    }
#endif

#if STM32_I2C_USE_I2C3
    if (&I2CD3 == i2cp) {
      nvicDisableVector(I2C3_EV_IRQn);
      nvicDisableVector(I2C3_ER_IRQn);
      rccDisableI2C3(FALSE);
    }
#endif

    reportErrs(i2cp, I2CD_STOPPED);  /* wake any blocked thread */
#if HAL_USE_I2C_SLAVE   /* I2C slave mode support */
    reportErrs(i2cp, I2CD_STOPPED);  /* ensure both master and slave notified */
#endif
  }
}


/**
 * @brief   Starts an I2C bus master transaction.
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 * @param[in] timeout   the number of ticks before the operation timeouts,
 *                      the following special values are allowed:
 *                      - @a TIME_INFINITE no timeout.
 *                      .
 * @return              The operation status.
 * @retval I2C_OK       if the function succeeded.
 * @retval I2C_ERROR    if one or more I2C errors occurred, the errors can
 *                      be retrieved using @p i2cGetErrors().
 * @retval I2C_TIMEOUT  A timeout occurred before operation end. <b>After a
 *                      timeout the driver should be stopped and restarted
 *                      because the bus may in an uncertain state</b>.
 * Drivers that support slave mode require only the the driver be restarted
 * after a timeout, as stopping and restarting may result in missed events.
 *
 * @notapi
 */
static msg_t startMasterAction(I2CDriver *i2cp, systime_t timeout)
{
  chDbgAssert((i2cp->mode <= i2cIsMaster), "startMasterAction()", "busy");

  i2cp->i2c->CR1 |= I2C_CR1_START | I2C_CR1_ACK;

  VirtualTimer vt;  /* Global timeout for the whole operation.*/
  chVTInit(&vt);

  if (timeout != TIME_INFINITE)
    chVTSetI(&vt, timeout, i2c_lld_safety_timeout, i2cp);
  i2cp->errors = 0;
  i2cp->thread = chThdSelf();
  chSchGoSleepS(THD_STATE_SUSPENDED);
  if (chVTIsArmedI(&vt))
    chVTResetI(&vt);
  return chThdSelf()->p_u.rdymsg;
}


/**
 * @brief   Receives data via the I2C bus as master.
 * @details Number of receiving bytes must be 0 or more than 1 on STM32F1x.
 *          This is hardware restriction.
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
 * @retval I2C_OK       if the function succeeded.
 * @retval I2C_ERROR    if one or more I2C errors occurred, the errors can
 *                      be retrieved using @p i2cGetErrors().
 * @retval I2C_TIMEOUT  A timeout occurred before operation end. <b>After a
 *                      timeout the driver should be stopped and restarted
 *                      because the bus may in an uncertain state</b>.
 * Drivers that support slave mode require only the the driver be restarted
 * after a timeout, as stopping and restarting may result in missed events.
 *
 * @notapi
 */
msg_t i2c_lld_master_receive_timeout(I2CDriver *i2cp, i2caddr_t addr,
                                     uint8_t *rxbuf, size_t rxbytes,
                                     systime_t timeout) {
  chDbgAssert((rxbytes < (1<<16)),
                "i2c_lld_master_receive_timeout(),#1", ">64Kbytes");
#if defined(STM32F1XX_I2C)
  chDbgAssert((rxbytes != 1),
              "i2c_lld_master_receive_timeout(),#2","rxbytes==1");
#endif
  chDbgAssert((i2cp->thread==NULL),
              "i2c_lld_master_receive_timeout(),#3","reentry");

  /* Initializes driver fields, LSB = 1 -> receive.*/
  i2cp->addr = (addr << 1) | 1;

  /* store away DMA info for later activation in event ISR */
  i2cp->masterRxbuf = rxbuf;
  i2cp->masterRxbytes = (uint16_t) rxbytes;
  return startMasterAction(i2cp, timeout);
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
 * @retval I2C_OK       if the function succeeded.
 * @retval I2C_ERROR    if one or more I2C errors occurred, the errors can
 *                      be retrieved using @p i2cGetErrors().
 * @retval I2C_TIMEOUT  A timeout occurred before operation end. <b>After a
 *                      timeout the driver should be stopped and restarted
 *                      because the bus is in an uncertain state</b>.
 * Drivers that support slave mode require only the the driver be restarted
 * after a timeout, as stopping and restarting may result in missed events.
 *
 * @notapi
 */
msg_t i2c_lld_master_transmit_timeout(I2CDriver *i2cp, i2caddr_t addr,
                                      const uint8_t *txbuf, size_t txbytes,
                                      uint8_t *rxbuf, size_t rxbytes,
                                      systime_t timeout) {
  if (rxbuf == NULL)
    rxbytes = 0;
  chDbgAssert(((rxbytes | txbytes) < (1<<16)),
                "i2c_lld_master_transmit_timeout(),#1", ">64Kbytes")
#if defined(STM32F1XX_I2C)
  chDbgAssert((rxbytes != 1),
              "i2c_lld_master_transmit_timeout(),#2","rxbytes==1");
#endif
  chDbgAssert((i2cp->thread==NULL),
              "i2c_lld_master_transmit_timeout(),#3","reentry");

  /* Initializes driver fields, LSB = 0 -> write.*/
  i2cp->addr = addr << 1;
  /* store away DMA info for later activation in event ISR */
  i2cp->masterTxbuf = txbuf;
  i2cp->masterTxbytes = (uint16_t) txbytes;
  i2cp->masterRxbuf = rxbuf;
  i2cp->masterRxbytes = (uint16_t) rxbytes;
  return startMasterAction(i2cp, timeout);
}


#if HAL_USE_I2C_SLAVE   /* I2C slave mode support */

/* These bits are undocumented, but used in STM32l1xx I2C example driver */
#define I2C_OAR1_Ack_7bit     (0x4000)  /*enable  7 bit address acknowledge*/
#define I2C_OAR1_Ack_10bit    (0xC000)  /*enable 10 bit address acknowledge*/

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
  I2C_TypeDef *dp = i2cp->i2c;
  if (i2cadr == 0) {
    dp->CR1 |= I2C_CR1_ENGC;
    dp->OAR1 |= I2C_OAR1_Ack_7bit;
  }else{
    uint16_t adr = i2cadr << 1;
    uint16_t ownAdr = dp->OAR1 & (0x7f<<1);
    if (ownAdr == 0 || ownAdr == adr)
      dp->OAR1 = adr | I2C_OAR1_Ack_7bit;
    else if (!(dp->OAR2 & I2C_OAR2_ENDUAL))
      dp->OAR2 = adr | I2C_OAR2_ENDUAL;
    else if ((dp->OAR2 & (0x7f<<1)) != adr)
      return I2C_ERROR;    /* cannot add this address to set of those matched */
  }
  return I2C_OK;
}


/**
 * @brief   Reconfigure I2C channel to no longer match specified address
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 * @param[in] i2cadr    I2C network address
 *
 * @details   A message being transferred that has already matched the
 *            specified address will continue being processed.
 *  Requests to unmatch an address that is not currently being matched
 *  are ignored.
 *  Does not support 10-bit addressing.
 *
 * @notapi
 **/
void i2c_lld_unmatchAddress(I2CDriver *i2cp, i2caddr_t  i2cadr)
{
  I2C_TypeDef *dp = i2cp->i2c;
  if (i2cadr == 0) {
    dp->CR1 &= ~I2C_CR1_ENGC;
    if ((dp->OAR1 & (0x7f<<1)) == 0)
      dp->OAR1 = 0;
  }else{
    uint16_t adr = i2cadr << 1;
    if ((dp->OAR1 & (0x7f<<1)) == adr) {
      if (dp->OAR2 & I2C_OAR2_ENDUAL)
        dp->OAR1 = (dp->OAR2 & (0x7f<<1)) | I2C_OAR1_Ack_7bit;
      else
        dp->OAR1 = dp->CR1 & I2C_CR1_ENGC ? I2C_OAR1_Ack_7bit : 0;
    }else if (dp->OAR2 & I2C_OAR2_ENDUAL && (dp->OAR2 & (0x7f<<1)) == adr)
      dp->OAR2 &= ~I2C_OAR2_ENDUAL;
  }
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
void i2c_lld_unmatchAll(I2CDriver *i2cp)
{
  I2C_TypeDef *dp = i2cp->i2c;
  dp->CR1 &= ~I2C_CR1_ENGC;
  dp->OAR1 = 0;
  dp->OAR2 = 0;
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
void i2c_lld_slaveReceive(I2CDriver *i2cp, const I2CSlaveMsg *rxMsg)
{
  chDbgCheck((rxMsg && rxMsg->size <= 0xffff), "i2c_lld_slaveReceive");
  i2cp->slaveNextRx = rxMsg;
  if (i2cp->mode == i2cLockedRxing && rxMsg->body && rxMsg->size) {
    /* We can receive now! */
    I2C_TypeDef *dp = i2cp->i2c;
    (void)dp->SR1, dp->SR2;  /* clear I2C_SR1_ADDR */
    i2cp->slaveRx = rxMsg;
    /* slave RX DMA setup */
    dmaStreamSetMode(i2cp->dmarx, i2cp->rxdmamode);
    dmaStreamSetMemory0(i2cp->dmarx, rxMsg->body);
    dmaStreamSetTransactionSize(i2cp->dmarx, rxMsg->size);
    dmaStreamEnable(i2cp->dmarx);
    i2cp->mode = i2cSlaveRxing;
    dp->CR2 |= I2C_CR2_ITEVTEN;
  }
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
void i2c_lld_slaveReply(I2CDriver *i2cp, const I2CSlaveMsg *replyMsg)
{
  chDbgCheck((replyMsg && replyMsg->size <= 0xffff), "i2c_lld_slaveReply");
  i2cp->slaveNextReply = replyMsg;
  if (i2cp->mode == i2cLockedReplying && replyMsg->body && replyMsg->size) {
    i2cp->slaveReply = replyMsg;
    /* slave TX DMA setup -- we can reply now! */
    dmaStreamSetMode(i2cp->dmatx, i2cp->txdmamode);
    dmaStreamSetMemory0(i2cp->dmatx, replyMsg->body);
    dmaStreamSetTransactionSize(i2cp->dmatx, replyMsg->size);
    dmaStreamEnable(i2cp->dmatx);
    i2cp->mode = i2cSlaveReplying;
    i2cp->i2c->CR2 |= I2C_CR2_ITEVTEN;
  }
}

#endif /* HAL_USE_I2C_SLAVE */

#endif /* HAL_USE_I2C */

/** @} */
