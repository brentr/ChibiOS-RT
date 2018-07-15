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

    ESP Elf Cartridge board
*/

#define slowstartMs   200   //milliseconds for logic supply to stabilize

#include "ch.h"
#include "hal.h"

#if HAL_USE_PAL || defined(__DOXYGEN__)
/**
 * @brief   PAL setup.
 * @details Digital I/O ports static configuration as defined in @p board.h.
 *          This variable is used by the HAL when initializing the PAL driver.
 */
const PALConfig pal_default_config =
{
  {VAL_GPIOA_MODER, VAL_GPIOA_OTYPER, VAL_GPIOA_OSPEEDR, VAL_GPIOA_PUPDR,
   VAL_GPIOA_ODR,   VAL_GPIOA_AFRL,   VAL_GPIOA_AFRH},
  {VAL_GPIOB_MODER, VAL_GPIOB_OTYPER, VAL_GPIOB_OSPEEDR, VAL_GPIOB_PUPDR,
   VAL_GPIOB_ODR,   VAL_GPIOB_AFRL,   VAL_GPIOB_AFRH},
  {VAL_GPIOC_MODER, VAL_GPIOC_OTYPER, VAL_GPIOC_OSPEEDR, VAL_GPIOC_PUPDR,
   VAL_GPIOC_ODR,   VAL_GPIOC_AFRL,   VAL_GPIOC_AFRH},
  {VAL_GPIOD_MODER, VAL_GPIOD_OTYPER, VAL_GPIOD_OSPEEDR, VAL_GPIOD_PUPDR,
   VAL_GPIOD_ODR,   VAL_GPIOD_AFRL,   VAL_GPIOD_AFRH},
  {VAL_GPIOE_MODER, VAL_GPIOE_OTYPER, VAL_GPIOE_OSPEEDR, VAL_GPIOE_PUPDR,
   VAL_GPIOE_ODR,   VAL_GPIOE_AFRL,   VAL_GPIOE_AFRH},
  {VAL_GPIOH_MODER, VAL_GPIOH_OTYPER, VAL_GPIOH_OSPEEDR, VAL_GPIOH_PUPDR,
   VAL_GPIOH_ODR,   VAL_GPIOH_AFRL,   VAL_GPIOH_AFRH}
};
#endif

/**
 * @brief   Early initialization code.
 * @details This initialization must be performed just after stack setup
 *          and before any other initialization.
 */
void __early_init(void) {
  //first we must get our power consumption down as low as possible
  //while waiting for supply voltage to ramp up before bypassing our
  //slowstart resistor.  (this should get us down to < 100uA)

  /* PWR clock enable.*/
  RCC->APB1ENR = RCC_APB1ENR_PWREN;

  //slow down the clock to 65kHz
  RCC->ICSCR = (RCC->ICSCR & ~STM32_MSIRANGE_MASK) | STM32_MSIRANGE_64K;
  while ((RCC->CR & RCC_CR_MSIRDY) == 0) ;  //await stable clock

  //switch to lowest voltage possible and make ready for low regulator power
  PWR->CR = STM32_VOS_1P2 | PWR_CR_LPSDSR;
  PWR->CR |= PWR_CR_LPRUN; //enter low power run mode
  //no need to await stable voltage -- let it fall during slowstart loop

  /* disable PWR clock to save a tiny bit of power during slowstart */
  RCC->APB1ENR = 0;

  //wait logic supply to ramp up
  //each iteration of loop below takes roughly 92 microseconds.
  volatile unsigned count = slowstartMs * 11;
  while(--count) ;

  //disable slowstart before enabling high-speed CPU operation
   _pal_lld_init(&pal_default_config);

  count = 2*11;
  while(--count) ;  //wait 2ms for FET to turn on after bypassing slow start.

  RCC->APB1ENR = RCC_APB1ENR_PWREN;
  PWR->CR &= ~PWR_CR_LPRUN;  //exit low-power run, enabling main power regulator
  stm32_clock_init();  //switch to high speed clock!
}

#if HAL_USE_SDC || defined(__DOXYGEN__)
/**
 * @brief   SDC card detection.
 */
bool_t sdc_lld_is_card_inserted(SDCDriver *sdcp) {

  (void)sdcp;
  /* TODO: Fill the implementation.*/
  return TRUE;
}

/**
 * @brief   SDC card write protection detection.
 */
bool_t sdc_lld_is_write_protected(SDCDriver *sdcp) {

  (void)sdcp;
  /* TODO: Fill the implementation.*/
  return FALSE;
}
#endif /* HAL_USE_SDC */

#if HAL_USE_MMC_SPI || defined(__DOXYGEN__)
/**
 * @brief   MMC_SPI card detection.
 */
bool_t mmc_lld_is_card_inserted(MMCDriver *mmcp) {

  (void)mmcp;
  /* TODO: Fill the implementation.*/
  return TRUE;
}

/**
 * @brief   MMC_SPI card write protection detection.
 */
bool_t mmc_lld_is_write_protected(MMCDriver *mmcp) {

  (void)mmcp;
  /* TODO: Fill the implementation.*/
  return FALSE;
}
#endif

/**
 * @brief   Board-specific initialization code.
 * @todo    Add your board-specific code, if any.
 */
void boardInit(void) {
}
