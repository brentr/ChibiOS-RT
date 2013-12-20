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

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

static void i2c_lld_abort_operation(I2CDriver *i2cp);

/**
 * @brief   return the address matched
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 * @param[in] sr2       I2C SR2 register contents
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
 * @brief   Handling of stalled slave mode I2C transactions.
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 *
 * @notapi
 */
static void timeExpired(void *i2cv) {
  I2CDriver *i2cp = (I2CDriver *)i2cv;

  i2c_lld_abort_operation(i2cp);
  const I2CSlaveMsg *xfer = i2cp->mode >= i2cSlaveReplying ?
                                          i2cp->slaveReply : i2cp->slaveRx;
  xfer->exception(i2cp);  /* in this case, i2cp->slaveErrors == 0 */
  i2cp->mode = i2cIsSlave;
}


/**
 * @brief   stop slave mode transaction timeout countdown
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 *
 * @notapi
 */
static INLINE void stopAction(I2CDriver *i2cp)
{
  if (chVTIsArmedI(&i2cp->slaveTimer))
    chVTResetI(&i2cp->slaveTimer);
}

/**
 * @brief   start or restart slave mode transaction
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 *
 * @notapi
 */
static INLINE void startAction(I2CDriver *i2cp, i2caddr_t targetAdr)
{
  stopAction(i2cp);
  i2cp->targetAdr = targetAdr;
  i2cp->slaveBytes = 0;
  i2cp->slaveErrors = 0;
  if (i2cp->slaveTimeout != TIME_INFINITE)
    chVTSetI(&i2cp->slaveTimer, i2cp->slaveTimeout, timeExpired, i2cp);
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
  dmaStreamDisable(i2cp->dmarx);
  if (i2cp->slaveBytes)
    i2cp->slaveBytes += 0xffff - bytesRemaining;
  else
    i2cp->slaveBytes = i2cp->slaveRx->size - bytesRemaining;
}

/**
 * @brief   process slave receive message
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 *
 * @notapi
 */
static INLINE void processSlaveRx(I2CDriver *i2cp)
{
  i2cp->slaveRx->processMsg(i2cp);
  i2cp->targetAdr = i2cInvalidAdr;
  stopAction(i2cp);
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
  dmaStreamDisable(i2cp->dmatx);
  if (i2cp->slaveBytes)
    i2cp->slaveBytes += 0xffff - bytesRemaining;
  else
    i2cp->slaveBytes = i2cp->slaveReply->size - bytesRemaining;
}

/**
 * @brief   process slave transmit message
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 *
 * @notapi
 */
static INLINE void processSlaveReply(I2CDriver *i2cp)
{
  i2cp->slaveReply->processMsg(i2cp);
  i2cp->targetAdr = i2cInvalidAdr;
  stopAction(i2cp);
}


#define setSlaveMode(i2cp, newMode)  {i2cp->mode=(newMode);}
#else   /* ! HAL_USE_I2C_SLAVE */
#define setSlaveMode(ignored, too)   {}
#define stopAction(ignored)  {}
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

  if (clock_speed <= 100000) {
    /* Configure clock_div in standard mode.*/
    chDbgAssert(duty == STD_DUTY_CYCLE,
                "i2c_lld_set_clock(), #1",
                "Invalid standard mode duty cycle");

    /* Standard mode clock_div calculate: Tlow/Thigh = 1/1.*/
#ifndef STM32_I2C_LOOSE_CLK
    chDbgAssert((STM32_PCLK1 % (clock_speed * 2)) == 0,
                "i2c_lld_set_clock(), #2",
                "PCLK1 not divisible by 2*I2Cclk");
#endif
    clock_div = STM32_PCLK1 / (clock_speed * 2);

    chDbgAssert(clock_div >= 0x04,
                "i2c_lld_set_clock(), #3",
                "Clock divider < 4 not allowed");
    regCCR |= (clock_div & I2C_CCR_CCR);

    /* Sets the Maximum Rise Time for standard mode.*/
    dp->TRISE = I2C_CLK_FREQ + 1;
  }
  else {
    chDbgAssert((clock_speed <= 400000),
                "i2c_lld_set_clock(), #4",
                "I2Cclk too fast");
    /* Configure clock_div in fast mode.*/
    chDbgAssert((duty == FAST_DUTY_CYCLE_2) || (duty == FAST_DUTY_CYCLE_16_9),
                "i2c_lld_set_clock(), #5",
                "Invalid fast mode duty cycle");

    if (duty == FAST_DUTY_CYCLE_2) {
      /* Fast mode clock_div calculate: Tlow/Thigh = 2/1.*/
#ifndef STM32_I2C_LOOSE_CLK
      chDbgAssert((STM32_PCLK1 % (clock_speed * 3)) == 0,
                  "i2c_lld_set_clock(), #6",
                  "PCLK1 not divisible by 3*I2Cclk");
#endif
      clock_div = STM32_PCLK1 / (clock_speed * 3);
    }
    else if (duty == FAST_DUTY_CYCLE_16_9) {
      /* Fast mode clock_div calculate: Tlow/Thigh = 16/9.*/
#ifndef STM32_I2C_LOOSE_CLK
      chDbgAssert((STM32_PCLK1 % (clock_speed * 25)) == 0,
                  "i2c_lld_set_clock(), #7",
                  "PCLK1 not divisible by 25*I2Cclk");
#endif
      clock_div = STM32_PCLK1 / (clock_speed * 25);
      regCCR |= I2C_CCR_DUTY;
    }

    chDbgAssert(clock_div >= 0x01,
                    "i2c_lld_set_clock(), #8",
                    "Clock divider less then 0x04 not allowed");
    regCCR |= (I2C_CCR_FS | (clock_div & I2C_CCR_CCR));

    /* Sets the Maximum Rise Time for fast mode.*/
    dp->TRISE = (I2C_CLK_FREQ * 300 / 1000) + 1;
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
 * @brief   Resets and disables the I2C channel
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 *
 * @notapi
 */
static void i2c_lld_disable(I2CDriver *i2cp) {
  I2C_TypeDef *dp = i2cp->i2c;
  /* reset the I2C registers.*/
  dp->CR1 = I2C_CR1_SWRST;
  stopAction(i2cp);
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
static void i2c_lld_abort_operation(I2CDriver *i2cp) {
  I2C_TypeDef *dp = i2cp->i2c;

  /* save control registers */
  uint16_t cr2 = dp->CR2, cr1 = dp->CR1;
#if HAL_USE_I2C_SLAVE
  uint16_t oar1 = dp->OAR1, oar2 = dp->OAR2;
#endif
  uint16_t ccr = dp->CCR, trise = dp->TRISE;

  /* reset the I2C channel */
  i2c_lld_disable(i2cp);

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
  i2cp->targetAdr = i2cInvalidAdr
#endif
                                  ;
}


/**
 * @brief   Handling of stalled master mode I2C transactions.
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 *
 * @notapi
 */
static void i2c_lld_safety_timeout(void *p) {
  I2CDriver *i2cp = (I2CDriver *)p;

  i2c_lld_abort_operation(i2cp);
  wakeup_isr(i2cp, I2C_TIMEOUT);
}


/**
 * @brief   report error waking thread or invoking callback as appropriate
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 * @param[in] errCode   error bit mask or code
 *
 * @notapi
 */
void reportErrs(I2CDriver *i2cp, i2cflags_t errCode)
{
  /* Disable any active DMA */
  dmaStreamDisable(i2cp->dmatx);
  dmaStreamDisable(i2cp->dmarx);
#if HAL_USE_I2C_SLAVE
  if (i2cp->mode < i2cIsMaster) {
    i2cp->slaveErrors = errCode;
    const I2CSlaveMsg *xfer =
      i2cp->mode >= i2cSlaveReplying ? i2cp->slaveReply : i2cp->slaveRx;
    xfer->exception(i2cp);
    i2cp->targetAdr = i2cInvalidAdr;
  }else{
    i2cp->errors = errCode;
    /* wake any master mode handling thread.*/
    wakeup_isr(i2cp, I2C_ERROR);
  }
  i2cp->mode = i2cIsSlave;
#else
  i2cp->errors = errCode;
  /* wake any master mode handling thread.*/
  wakeup_isr(i2cp, I2C_ERROR);
#endif
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
  if (i2cp->mode == i2cSlaveReplying && (sr & I2C_SR1_AF)) {
    endSlaveReplyDMA(i2cp, 1);
    processSlaveReply(i2cp);
    i2cp->mode = i2cIsSlave;
    return;
  }
  stopAction(i2cp);
#endif
  i2cflags_t errs = 0;

  if (sr & I2C_SR1_ARLO)                            /* Arbitration lost.    */
    errs |= I2CD_ARBITRATION_LOST;

  if (sr & I2C_SR1_BERR)                            /* Bus error.           */
    errs |= I2CD_BUS_ERROR;

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

  reportErrs(i2cp, errs);
}


/* quick and dirty queue to record event interrupts */
#define QEVENTS 0
#if QEVENTS > 0
uint32_t i2cQ[QEVENTS];
unsigned i2cI = QEVENTS;
#define qEvt(code) {if (++i2cI >= QEVENTS) i2cI = 0; i2cQ[i2cI]=(code);}
#else
#define qEvt(code)
#endif


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
  i2caddr_t  targetAdr;

  qEvt(event);
  switch (event & I2C_EV_MASK) {

#if !HAL_USE_I2C_SLAVE
#define chkTransition(expectedMode) {}
#else
#define chkTransition(expectedMode) { \
   if (i2cp->mode != (expectedMode)) goto invalidTransition;}

   invalidTransition:   /* error on known event out of expected sequence */
    i2c_lld_abort_operation(i2cp);        /* reset and reinit */
    reportErrs(i2cp, I2CD_UNKNOWN_ERROR + i2cp->mode);
   break;

   case I2C_EV1_SLAVE_RXADRMATCH:
    targetAdr = matchedAdr(dp, regSR2);
    (void)dp->SR2;  /* clear I2C_SR1_ADDR */
    switch (i2cp->mode) {
      case i2cIsSlave:
        break;
      case i2cSlaveRxing:
        endSlaveRxDMA(i2cp);
      case i2cLockedRxing:
        processSlaveRx(i2cp);
        break;
      default:
        goto invalidTransition;
    }
    startAction(i2cp, targetAdr);
    {
      const I2CSlaveMsg *rx = i2cp->slaveNextRx;
      rx->adrMatched(i2cp);
      rx = i2cp->slaveRx = i2cp->slaveNextRx;
      if (rx->body && rx->size) {
         /* slave RX DMA setup.*/
        dmaStreamSetMode(i2cp->dmarx, i2cp->rxdmamode);
        dmaStreamSetMemory0(i2cp->dmarx, rx->body);
        dmaStreamSetTransactionSize(i2cp->dmarx, rx->size);
        dmaStreamEnable(i2cp->dmarx);
        i2cp->mode = i2cSlaveRxing;
      }else
        i2cp->mode = i2cLockedRxing;
    }
    break;

   case I2C_EV2_SLAVE_RXSTOP:
    dp->CR1 = regCR1;            /* clear STOPF */
    switch (i2cp->mode) {
      case i2cSlaveRxing:
        endSlaveRxDMA(i2cp);
      case i2cLockedRxing:       /* for SMBus 0-byte Quick Write Command */
        processSlaveRx(i2cp);
        break;
      case i2cLockedReplying:
        endSlaveReplyDMA(i2cp, 2);  /* did not NACK last transmitted byte */
      case i2cSlaveReplying:     /* for SMBus 0-byte Quick Read Command */
        processSlaveReply(i2cp);
        break;
      default:
        goto invalidTransition;
    }
    i2cp->mode = i2cIsSlave;
    break;

   case I2C_EV1_SLAVE_TXADRMATCH:
    targetAdr = matchedAdr(dp, regSR2);
    (void)dp->SR2;  /* clear I2C_SR1_ADDR */
    switch (i2cp->mode) {
      case i2cIsSlave:
        break;
      case i2cSlaveRxing:
        endSlaveRxDMA(i2cp);
      case i2cLockedRxing:
        processSlaveRx(i2cp);
        break;
      default:
        goto invalidTransition;
    }
    startAction(i2cp, targetAdr);
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
        break;
      }
      i2cp->mode = i2cLockedReplying;
    }
    break;
#endif  /* HAL_USE_I2C_SLAVE */

   case I2C_EV5_MASTER_MODE_SELECT:
    dp->DR = i2cp->addr;
    chkTransition(i2cIsSlave);
    if (i2cp->addr&1 && !i2cp->masterRxbytes) {
      /* handle 0-length SMBus style quick read command */
      dp->CR1 = regCR1 | I2C_CR1_STOP;
    }
    setSlaveMode(i2cp, i2cIsMaster);
    break;

   case I2C_EV6_MASTER_REC_MODE_SELECTED:
    (void)dp->SR2;  /* clear I2C_SR1_ADDR */
    chkTransition(i2cIsMaster);
    if (!i2cp->masterRxbytes)         /* 0-length SMBus style quick read */
      goto done;
    /* RX DMA setup.*/
    dmaStreamSetMode(i2cp->dmarx, i2cp->rxdmamode);
    dmaStreamSetMemory0(i2cp->dmarx, i2cp->masterRxbuf);
    dmaStreamSetTransactionSize(i2cp->dmarx, i2cp->masterRxbytes);
    dmaStreamEnable(i2cp->dmarx);
    dp->CR2 |= I2C_CR2_LAST;                 /* Needed in receiver mode. */
    if (i2cp->masterRxbytes < 2)
      dp->CR1 = regCR1 & ~I2C_CR1_ACK;
    setSlaveMode(i2cp, i2cMasterRxing);
    break;

   case I2C_EV6_MASTER_TRA_MODE_SELECTED:
    (void)dp->SR2;  /* clear I2C_SR1_ADDR */
    chkTransition(i2cIsMaster);
    if (!i2cp->masterTxbytes)
      goto doneWriting;
    /* TX DMA setup.*/
    dmaStreamSetMode(i2cp->dmatx, i2cp->txdmamode);
    dmaStreamSetMemory0(i2cp->dmatx, i2cp->masterTxbuf);
    dmaStreamSetTransactionSize(i2cp->dmatx, i2cp->masterTxbytes);
    dmaStreamEnable(i2cp->dmatx);
    setSlaveMode(i2cp, i2cMasterTxing);
    break;

   case I2C_EV8_2_MASTER_BYTE_TRANSMITTED:
    chkTransition(i2cMasterTxing);
    /* Catches BTF event after the end of transmission.*/
doneWriting:
    if (i2cp->masterRxbuf) {
      /* Starts "read after write" operation, LSB = 1 -> receive.*/
      i2cp->addr |= 0x01;
      dp->CR1 = regCR1 | I2C_CR1_START | I2C_CR1_ACK;
      setSlaveMode(i2cp, i2cIsMaster);
    }else{
      dp->CR1 = regCR1 | I2C_CR1_STOP | I2C_CR1_ACK;
done:
      setSlaveMode(i2cp, i2cIsSlave);
      wakeup_isr(i2cp, I2C_OK);
    }
    break;

   default:  /* unhandled event -- abort transaction, flag unknown err */
    i2c_lld_abort_operation(i2cp);
    i2c_lld_serve_error_interrupt(i2cp, event);
  }
}

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
  dp->CR1 |= I2C_CR1_STOP | I2C_CR1_ACK;
  setSlaveMode(i2cp, i2cIsSlave);
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
#if HAL_USE_I2C_SLAVE
  chVTInit(&I2CD1.slaveTimer);
#endif
#endif /* STM32_I2C_USE_I2C1 */

#if STM32_I2C_USE_I2C2
  i2cObjectInit(&I2CD2);
  I2CD2.thread = NULL;
  I2CD2.i2c    = I2C2;
  I2CD2.dmarx  = STM32_DMA_STREAM(STM32_I2C_I2C2_RX_DMA_STREAM);
  I2CD2.dmatx  = STM32_DMA_STREAM(STM32_I2C_I2C2_TX_DMA_STREAM);
#if HAL_USE_I2C_SLAVE
  chVTInit(&I2CD2.slaveTimer);
#endif
#endif /* STM32_I2C_USE_I2C2 */

#if STM32_I2C_USE_I2C3
  i2cObjectInit(&I2CD3);
  I2CD3.thread = NULL;
  I2CD3.i2c    = I2C3;
  I2CD3.dmarx  = STM32_DMA_STREAM(STM32_I2C_I2C3_RX_DMA_STREAM);
  I2CD3.dmatx  = STM32_DMA_STREAM(STM32_I2C_I2C3_TX_DMA_STREAM);
#if HAL_USE_I2C_SLAVE
  chVTInit(&I2CD3.slaveTimer);
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
      bool_t b;

      b = dmaStreamAllocate(i2cp->dmarx,
                            STM32_I2C_I2C1_IRQ_PRIORITY,
                            (stm32_dmaisr_t)i2c_lld_serve_rx_end_irq,
                            (void *)i2cp);
      chDbgAssert(!b, "i2c_lld_start(), #1", "stream already allocated");
      b = dmaStreamAllocate(i2cp->dmatx,
                            STM32_I2C_I2C1_IRQ_PRIORITY,
                            (stm32_dmaisr_t)i2c_lld_serve_tx_end_irq,
                            (void *)i2cp);
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
      bool_t b;

      b = dmaStreamAllocate(i2cp->dmarx,
                            STM32_I2C_I2C2_IRQ_PRIORITY,
                            (stm32_dmaisr_t)i2c_lld_serve_rx_end_irq,
                            (void *)i2cp);
      chDbgAssert(!b, "i2c_lld_start(), #3", "stream already allocated");
      b = dmaStreamAllocate(i2cp->dmatx,
                            STM32_I2C_I2C2_IRQ_PRIORITY,
                            (stm32_dmaisr_t)i2c_lld_serve_tx_end_irq,
                            (void *)i2cp);
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
      bool_t b;

      b = dmaStreamAllocate(i2cp->dmarx,
                            STM32_I2C_I2C3_IRQ_PRIORITY,
                            (stm32_dmaisr_t)i2c_lld_serve_rx_end_irq,
                            (void *)i2cp);
      chDbgAssert(!b, "i2c_lld_start(), #5", "stream already allocated");
      b = dmaStreamAllocate(i2cp->dmatx,
                            STM32_I2C_I2C3_IRQ_PRIORITY,
                            (stm32_dmaisr_t)i2c_lld_serve_tx_end_irq,
                            (void *)i2cp);
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

#if HAL_USE_I2C_SLAVE   /* I2C slave mode support */
    i2cp->slaveNextReply = i2cp->slaveNextRx = &I2CSlaveLockOnMsg;
    i2cp->mode = i2cIsSlave;
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
    i2c_lld_disable(i2cp);
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
  I2C_TypeDef *dp = i2cp->i2c;
  VirtualTimer vt;

  chDbgAssert((rxbytes < (1<<16)),
                "i2c_lld_master_receive_timeout(), #1", ">64Kbytes");
#if defined(STM32F1XX_I2C)
  chDbgCheck((rxbytes > 1), "i2c_lld_master_receive_timeout");
#endif

#if CH_DBG_SYSTEM_STATE_CHECK
  if (i2cp->thread)
    chDbgPanic("I2C RX reentry");
#endif

  /* Global timeout for the whole operation.*/
  chVTInit(&vt);
  if (timeout != TIME_INFINITE)
    chVTSetI(&vt, timeout, i2c_lld_safety_timeout, (void *)i2cp);

  /* Initializes driver fields, LSB = 1 -> receive.*/
  i2cp->addr = (addr << 1) | 0x01;
  i2cp->errors = 0;

  /* store away DMA info for later activation in event ISR */
  i2cp->masterRxbuf = rxbuf;
  i2cp->masterRxbytes = (uint16_t) rxbytes;

  /* Starts the operation.*/
  dp->CR1 |= I2C_CR1_START | I2C_CR1_ACK;
  i2cp->thread = chThdSelf();
  chSchGoSleepS(THD_STATE_SUSPENDED);
  if (chVTIsArmedI(&vt))
    chVTResetI(&vt);
  return chThdSelf()->p_u.rdymsg;
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
  I2C_TypeDef *dp = i2cp->i2c;
  VirtualTimer vt;

  if (rxbuf == NULL)
    rxbytes = 0;
  chDbgAssert(((rxbytes | txbytes) < (1<<16)),
                "i2c_lld_master_transmit_timeout(), #1", ">64Kbytes")
#if defined(STM32F1XX_I2C)
  chDbgCheck((rxbytes > 1), "i2c_lld_master_transmit_timeout");
#endif

#if CH_DBG_SYSTEM_STATE_CHECK
  if (i2cp->thread)
    chDbgPanic("I2C TX reentry");
#endif

  /* Global timeout for the whole operation.*/
  chVTInit(&vt);
  if (timeout != TIME_INFINITE)
    chVTSetI(&vt, timeout, i2c_lld_safety_timeout, (void *)i2cp);

  /* Initializes driver fields, LSB = 0 -> write.*/
  i2cp->addr = addr << 1;
  i2cp->errors = 0;

  /* store away DMA info for later activation in event ISR */
  i2cp->masterTxbuf = txbuf;
  i2cp->masterTxbytes = (uint16_t) txbytes;
  i2cp->masterRxbuf = rxbuf;
  i2cp->masterRxbytes = (uint16_t) rxbytes;

  /* Starts the operation.*/
  dp->CR1 |= I2C_CR1_START | I2C_CR1_ACK;
  i2cp->thread = chThdSelf();
  chSchGoSleepS(THD_STATE_SUSPENDED);
  if (chVTIsArmedI(&vt))
    chVTResetI(&vt);
  return chThdSelf()->p_u.rdymsg;
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
 * @retval 0            Success
 * @retval <0           Cannot match address in addition of those already
 *
 * *notes
 *  MatchAddress calls are cumulative.
 *   Specify address zero to match I2C "all call"
 *
 *  Does not support 10-bit addressing.
 *
 * @notapi
 **/
int i2c_lld_matchAddress(I2CDriver *i2cp, i2caddr_t  i2cadr)
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
      return -1;    /* cannot add this address to set of those matched */
  }
  return 0;
}


/**
 * @brief   Reconfigure I2C channel to no longer match specified address
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 * @param[in] i2cadr    I2C network address
 *
 *  A message being transferred that has already matched the specified address
 *  will continue being processed.
 *  Requests to unmatch an address that is not currently being matched
 *  are ignored.
 *
 *   Does not support 10-bit addressing.
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
 *  Clears all match addresses.  Causes all subsequent messages to be ignored.
 *  A message being transferred that has already matched a slave address
 *  will continue being processed.
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
 * @brief   Prepare to receive and process I2C messages
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 * @param[in] rxMsg     pointer to the @p I2CSlaveMsg struct (or NULL)
 *
 *  A NULL rxMsg will cause the I2C bus to stall on the next write
 *  to this slave until this function is called with a non-NULL rxMsg
 *
 *    Does not affect the processing of a message currently being received
 *
 * @notapi
 **/
void i2c_lld_slaveReceive(I2CDriver *i2cp, const I2CSlaveMsg *rxMsg)
{
  chDbgCheck((rxMsg && rxMsg->size <= 0xffff), "i2c_lld_slaveReceive");
  i2cp->slaveNextRx = rxMsg;
  if (i2cp->mode == i2cLockedRxing && rxMsg->body && rxMsg->size) {
    i2cp->slaveRx = rxMsg;
    /* slave RX DMA setup -- we can receive now! */
    dmaStreamSetMode(i2cp->dmarx, i2cp->rxdmamode);
    dmaStreamSetMemory0(i2cp->dmarx, rxMsg->body);
    dmaStreamSetTransactionSize(i2cp->dmarx, rxMsg->size);
    dmaStreamEnable(i2cp->dmarx);
    i2cp->mode = i2cSlaveRxing;
  }
}


/**
 * @brief   Prepare to reply to I2C queries
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 * @param[in] rxMsg     pointer to the @p I2CSlaveMsg struct (or NULL)
 *
 *  A NULL rxMsg will cause the I2C bus to stall on the next write
 *  to this slave until this function is called with a non-NULL rxMsg
 *
 *    Does not affect the processing of a message currently being received
 *
 * @notapi
 **/
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
  }
}

#endif /* HAL_USE_I2C_SLAVE */

#endif /* HAL_USE_I2C */

/** @} */
