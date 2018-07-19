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
    At reset, power is delivered through a 1kOhm resistor
    Need to get power consumption down < 100uA quickly
    Immediately enter low-power run mode
    This gets to about 180uA
    Digital input mode Schmitt triggers consume 100uA
      Switch all possible GPIOs to Analog mode to reach 60-80uA
    After softstart delay, bypass the 1kOhm resistor
*/

#include "pads.h"

#define slowstartMs     50         //milliseconds for slow start
#define cartLogicStart  GPIOB, 2   //limit 3.3V current until driven high

//clock enables to reconfigure all GPIO ports
#define AHB_EN_MASK     (RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN |   \
                         RCC_AHBENR_GPIOCEN | RCC_AHBENR_GPIODEN |   \
                         RCC_AHBENR_GPIOEEN | RCC_AHBENR_GPIOHEN)

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



/**
 * @brief   Early initialization code.
 * @details This initialization must be performed just after stack setup
 *          and *before* writing stack pattern and any other initialization.
 */
void __early_init(void) {
  //first we must get our power consumption down as low as possible
  //while waiting for supply voltage to ramp up before bypassing our
  //slowstart resistor.  (this gets us down to ~ 70uA)

  //switch to lowest power GPIO configuration possible
  RCC->AHBENR |= AHB_EN_MASK;
  GPIOA->MODER = INITIAL_GPIOA_MODER;
  GPIOB->MODER = INITIAL_GPIOB_MODER;
  GPIOC->MODER = INITIAL_GPIOC_MODER;
  GPIOD->MODER = INITIAL_GPIOD_MODER;
  GPIOE->MODER = INITIAL_GPIOE_MODER;
  GPIOH->MODER = INITIAL_GPIOH_MODER;

  //slow down the clock to 65kHz
  RCC->APB1ENR = RCC_APB1ENR_PWREN;
  RCC->ICSCR = (RCC->ICSCR & ~STM32_MSIRANGE_MASK) | STM32_MSIRANGE_64K;
  while (!(RCC->CR & RCC_CR_MSIRDY)) ;  //await stable clock

  //set Vcore to 1.5V (range 2)
  //also disable voltage reference and main power regulator in low-power state
  PWR->CR = STM32_VOS_1P5 | PWR_CR_LPSDSR | PWR_CR_ULP;
  while ((PWR->CSR & PWR_CSR_VOSF)) ;   //wait until regulator is stable @1.5V

  PWR->CR |= PWR_CR_LPRUN;    //enter low power run mode
  // no need to wait for this.
//while(!(PWR->CSR & PWR_CSR_REGLPF)) ; //wait until regulator in low power mode

  //wait for logic supply to ramp up
  //each iteration of loop below takes roughly 100 microseconds w/65kHz clock.
  volatile unsigned count = slowstartMs * 10;
  while(--count) ;

  //bypass slowstart before enabling high-speed CPU operation
  setPad(cartLogicStart);
  configurePad(cartLogicStart, PAL_MODE_OUTPUT_PUSHPULL);

  PWR->CR &= ~PWR_CR_LPRUN;  //exit low-power run, enabling main power regulator
  while((PWR->CSR & PWR_CSR_REGLPF)) ;  //wait until regulator back in main mode

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
