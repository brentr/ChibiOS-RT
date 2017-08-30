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

#ifndef _BOARD_H_
#define _BOARD_H_

/*
 * Setup for MBARI ESP3 Cartridge Elf
 */

/*
 * Board identifier.
 */
#define BOARD_ST_STM32L_cartridge
#define BOARD_NAME                  "ST STM32L ESP Cartridge Elf"


/*
 * Board oscillators-related settings.
 * NOTE: HSE not fitted.
 */
#if !defined(STM32_LSECLK)
#define STM32_LSECLK                32768
#endif

#if !defined(STM32_HSECLK)
#define STM32_HSECLK                8000000
#endif

/*
 * MCU type as defined in the ST header file stm32l1xx.h.
 */
#define STM32L1XX_MD

/*
 * IO pins assignments.
 */
#define GPIOA_PIN0                  0
#define GPIOA_PIN1                  1
#define GPIOA_PIN2                  2
#define GPIOA_PIN3                  3
#define GPIOA_PIN4                  4
#define GPIOA_PIN5                  5
#define GPIOA_PIN6                  6
#define GPIOA_PIN7                  7
#define GPIOA_PIN8                  8
#define GPIOA_PIN9                  9
#define GPIOA_PIN10                 10
#define GPIOA_PIN11                 11
#define GPIOA_PIN12                 12
#define GPIOA_JTAG_TMS              13
#define GPIOA_JTAG_TCK              14
#define GPIOA_JTAG_TDI              15

#define GPIOB_PIN0                  0
#define GPIOB_PIN1                  1
#define GPIOB_BOOT1                 2
#define GPIOB_JTAG_TDO              3
#define GPIOB_JTAG_TRST             4
#define GPIOB_PIN5                  5
#define GPIOB_PIN6                  6
#define GPIOB_PIN7                  7
#define GPIOB_PIN8                  8
#define GPIOB_PIN9                  9
#define GPIOB_PIN10                 10
#define GPIOB_PIN11                 11
#define GPIOB_PIN12                 12
#define GPIOB_PIN13                 13
#define GPIOB_PIN14                 14
#define GPIOB_PIN15                 15

#define GPIOC_PIN0                  0
#define GPIOC_PIN1                  1
#define GPIOC_PIN2                  2
#define GPIOC_PIN3                  3
#define GPIOC_PIN4                  4
#define GPIOC_PIN5                  5
#define GPIOC_PIN6                  6
#define GPIOC_PIN7                  7
#define GPIOC_PIN8                  8
#define GPIOC_PIN9                  9
#define GPIOC_PIN10                 10
#define GPIOC_PIN11                 11
#define GPIOC_PIN12                 12
#define GPIOC_PIN13                 13
#define GPIOC_OSC32_IN              14
#define GPIOC_OSC32_OUT             15

#define GPIOD_PIN0                  0
#define GPIOD_PIN1                  1
#define GPIOD_PIN2                  2
#define GPIOD_PIN3                  3
#define GPIOD_PIN4                  4
#define GPIOD_PIN5                  5
#define GPIOD_PIN6                  6
#define GPIOD_PIN7                  7
#define GPIOD_PIN8                  8
#define GPIOD_PIN9                  9
#define GPIOD_PIN10                 10
#define GPIOD_PIN11                 11
#define GPIOD_PIN12                 12
#define GPIOD_PIN13                 13
#define GPIOD_PIN14                 14
#define GPIOD_PIN15                 15

#define GPIOE_PIN0                  0
#define GPIOE_PIN1                  1
#define GPIOE_PIN2                  2
#define GPIOE_PIN3                  3
#define GPIOE_PIN4                  4
#define GPIOE_PIN5                  5
#define GPIOE_PIN6                  6
#define GPIOE_PIN7                  7
#define GPIOE_PIN8                  8
#define GPIOE_PIN9                  9
#define GPIOE_PIN10                 10
#define GPIOE_PIN11                 11
#define GPIOE_PIN12                 12
#define GPIOE_PIN13                 13
#define GPIOE_PIN14                 14
#define GPIOE_PIN15                 15

#define GPIOH_OSC_IN                0
#define GPIOH_OSC_OUT               1
#define GPIOH_PIN2                  2
#define GPIOH_PIN3                  3
#define GPIOH_PIN4                  4
#define GPIOH_PIN5                  5
#define GPIOH_PIN6                  6
#define GPIOH_PIN7                  7
#define GPIOH_PIN8                  8
#define GPIOH_PIN9                  9
#define GPIOH_PIN10                 10
#define GPIOH_PIN11                 11
#define GPIOH_PIN12                 12
#define GPIOH_PIN13                 13
#define GPIOH_PIN14                 14
#define GPIOH_PIN15                 15

/*
 * I/O ports initial setup, this configuration is established soon after reset
 * in the initialization code.
 * Please refer to the STM32 Reference Manual for details.
 */
#define PIN_MODE_INPUT(n)           (0U << ((n) * 2))
#define PIN_MODE_OUTPUT(n)          (1U << ((n) * 2))
#define PIN_MODE_ALTERNATE(n)       (2U << ((n) * 2))
#define PIN_MODE_ANALOG(n)          (3U << ((n) * 2))
#define PIN_ODR_LOW(n)              (0U << (n))
#define PIN_ODR_HIGH(n)             (1U << (n))
#define PIN_OTYPE_PUSHPULL(n)       (0U << (n))
#define PIN_OTYPE_OPENDRAIN(n)      (1U << (n))
#define PIN_OSPEED_400K(n)          (0U << ((n) * 2))
#define PIN_OSPEED_2M(n)            (1U << ((n) * 2))
#define PIN_OSPEED_10M(n)           (2U << ((n) * 2))
#define PIN_OSPEED_40M(n)           (3U << ((n) * 2))
#define PIN_PUPDR_FLOATING(n)       (0U << ((n) * 2))
#define PIN_PUPDR_PULLUP(n)         (1U << ((n) * 2))
#define PIN_PUPDR_PULLDOWN(n)       (2U << ((n) * 2))
#define PIN_AFIO_AF(n, v)           ((v##U) << ((n % 8) * 4))

/*
 * GPIOA setup:
 *
 * PA0  - PIN0                      (input floating).
 * PA1  - PIN1                      (input floating).
 * PA2  - PIN2                      (input pulldown).
 * PA3  - PIN3                      (input pulldown).
 * PA4  - PIN4                      (input pulldown).
 * PA5  - PIN5                      (input pulldown).
 * PA6  - PIN6                      (input pulldown).
 * PA7  - PIN7                      (input pulldown).
 * PA8  - PIN8                      (input pulldown).
 * PA9  - PIN9                      (input pulldown).
 * PA10 - PIN10                     (input pulldown).
 * PA11 - PIN11                     (input pulldown).
 * PA12 - PIN12                     (input pulldown).
 * PA13 - JTAG_TMS                  (alternate 0).
 * PA14 - JTAG_TCK                  (alternate 0).
 * PA15 - JTAG_TDI                  (alternate 0).
 */
#define VAL_GPIOA_MODER             (PIN_MODE_INPUT(GPIOA_PIN0) |           \
                                     PIN_MODE_INPUT(GPIOA_PIN1) |           \
                                     PIN_MODE_INPUT(GPIOA_PIN2) |           \
                                     PIN_MODE_OUTPUT(GPIOA_PIN3) |          \
                                     PIN_MODE_INPUT(GPIOA_PIN4) |           \
                                     PIN_MODE_INPUT(GPIOA_PIN5) |           \
                                     PIN_MODE_INPUT(GPIOA_PIN6) |           \
                                     PIN_MODE_INPUT(GPIOA_PIN7) |           \
                                     PIN_MODE_INPUT(GPIOA_PIN8) |           \
                                     PIN_MODE_INPUT(GPIOA_PIN9) |           \
                                     PIN_MODE_INPUT(GPIOA_PIN10) |          \
                                     PIN_MODE_INPUT(GPIOA_PIN11) |          \
                                     PIN_MODE_INPUT(GPIOA_PIN12) |          \
                                     PIN_MODE_ALTERNATE(GPIOA_JTAG_TMS) |   \
                                     PIN_MODE_ALTERNATE(GPIOA_JTAG_TCK) |   \
                                     PIN_MODE_ALTERNATE(GPIOA_JTAG_TDI))
#define VAL_GPIOA_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOA_PIN0) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN1) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN2) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN3) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN4) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN5) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN6) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN7) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN8) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN9) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN10) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN11) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN12) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOA_JTAG_TMS) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOA_JTAG_TCK) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOA_JTAG_TDI))
#define VAL_GPIOA_OSPEEDR           (PIN_OSPEED_400K(GPIOA_PIN0) |        \
                                     PIN_OSPEED_400K(GPIOA_PIN1) |          \
                                     PIN_OSPEED_400K(GPIOA_PIN2) |          \
                                     PIN_OSPEED_400K(GPIOA_PIN3) |          \
                                     PIN_OSPEED_400K(GPIOA_PIN4) |          \
                                     PIN_OSPEED_400K(GPIOA_PIN5) |          \
                                     PIN_OSPEED_400K(GPIOA_PIN6) |          \
                                     PIN_OSPEED_400K(GPIOA_PIN7) |          \
                                     PIN_OSPEED_400K(GPIOA_PIN8) |          \
                                     PIN_OSPEED_400K(GPIOA_PIN9) |          \
                                     PIN_OSPEED_400K(GPIOA_PIN10) |         \
                                     PIN_OSPEED_400K(GPIOA_PIN11) |         \
                                     PIN_OSPEED_400K(GPIOA_PIN12) |         \
                                     PIN_OSPEED_40M(GPIOA_JTAG_TMS) |       \
                                     PIN_OSPEED_40M(GPIOA_JTAG_TCK) |       \
                                     PIN_OSPEED_40M(GPIOA_JTAG_TDI))
#define VAL_GPIOA_PUPDR             (PIN_PUPDR_FLOATING(GPIOA_PIN0) |       \
                                     PIN_PUPDR_FLOATING(GPIOA_PIN1) |       \
                                     PIN_PUPDR_PULLDOWN(GPIOA_PIN2) |         \
                                     PIN_PUPDR_FLOATING(GPIOA_PIN3) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOA_PIN4) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOA_PIN5) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOA_PIN6) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOA_PIN7) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOA_PIN8) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOA_PIN9) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOA_PIN10) |        \
                                     PIN_PUPDR_PULLDOWN(GPIOA_PIN11) |        \
                                     PIN_PUPDR_PULLDOWN(GPIOA_PIN12) |        \
                                     PIN_PUPDR_PULLDOWN(GPIOA_JTAG_TMS) |     \
                                     PIN_PUPDR_PULLDOWN(GPIOA_JTAG_TCK) |   \
                                     PIN_PUPDR_PULLDOWN(GPIOA_JTAG_TDI))
#define VAL_GPIOA_ODR               (PIN_ODR_HIGH(GPIOA_PIN0) |           \
                                     PIN_ODR_HIGH(GPIOA_PIN1) |             \
                                     PIN_ODR_HIGH(GPIOA_PIN2) |             \
                                     PIN_ODR_HIGH(GPIOA_PIN3) |             \
                                     PIN_ODR_HIGH(GPIOA_PIN4) |             \
                                     PIN_ODR_HIGH(GPIOA_PIN5) |             \
                                     PIN_ODR_HIGH(GPIOA_PIN6) |             \
                                     PIN_ODR_HIGH(GPIOA_PIN7) |             \
                                     PIN_ODR_HIGH(GPIOA_PIN8) |             \
                                     PIN_ODR_HIGH(GPIOA_PIN9) |             \
                                     PIN_ODR_HIGH(GPIOA_PIN10) |            \
                                     PIN_ODR_HIGH(GPIOA_PIN11) |            \
                                     PIN_ODR_HIGH(GPIOA_PIN12) |            \
                                     PIN_ODR_HIGH(GPIOA_JTAG_TMS) |         \
                                     PIN_ODR_HIGH(GPIOA_JTAG_TCK) |         \
                                     PIN_ODR_HIGH(GPIOA_JTAG_TDI))
#define VAL_GPIOA_AFRL              (PIN_AFIO_AF(GPIOA_PIN0, 0) |         \
                                     PIN_AFIO_AF(GPIOA_PIN1, 0) |           \
                                     PIN_AFIO_AF(GPIOA_PIN2, 0) |           \
                                     PIN_AFIO_AF(GPIOA_PIN3, 0) |           \
                                     PIN_AFIO_AF(GPIOA_PIN4, 0) |           \
                                     PIN_AFIO_AF(GPIOA_PIN5, 0) |           \
                                     PIN_AFIO_AF(GPIOA_PIN6, 0) |           \
                                     PIN_AFIO_AF(GPIOA_PIN7, 0))
#define VAL_GPIOA_AFRH              (PIN_AFIO_AF(GPIOA_PIN8, 0) |           \
                                     PIN_AFIO_AF(GPIOA_PIN9, 0) |           \
                                     PIN_AFIO_AF(GPIOA_PIN10, 0) |          \
                                     PIN_AFIO_AF(GPIOA_PIN11, 0) |          \
                                     PIN_AFIO_AF(GPIOA_PIN12, 0) |          \
                                     PIN_AFIO_AF(GPIOA_JTAG_TMS, 0) |       \
                                     PIN_AFIO_AF(GPIOA_JTAG_TCK, 0) |       \
                                     PIN_AFIO_AF(GPIOA_JTAG_TDI, 0))

/*
 * GPIOB setup:
 *
 * PB0  - PIN0                      (input pulldown).
 * PB1  - PIN1                      (input pulldown).
 * PB2  - BOOT1                     (output pushpull -- driven high to cancel slow start).
 * PB3  - JTAG_TDO                  (alternate 0).
 * PB4  - JTAG_TRST                 (alternate 0).
 * PB5  - PIN5                      (input pulldown).
 * PB6  - PIN6                      (input pulldown).
 * PB7  - PIN7                      (input pulldown).
 * PB8  - PIN8                      (input pulldown).
 * PB9  - PIN9                      (input pulldown).
 * PB10 - PIN10                     (input pulldown).
 * PB11 - PIN11                     (input pulldown).
 * PB12 - PIN12                     (input pulldown).
 * PB13 - PIN13                     (input pulldown).
 * PB14 - PIN14                     (input pulldown).
 * PB15 - PIN15                     (input pulldown).
 */
#define VAL_GPIOB_MODER             (PIN_MODE_INPUT(GPIOB_PIN0) |           \
                                     PIN_MODE_INPUT(GPIOB_PIN1) |           \
                                     PIN_MODE_OUTPUT(GPIOB_BOOT1) |         \
                                     PIN_MODE_ALTERNATE(GPIOB_JTAG_TDO) |   \
                                     PIN_MODE_ALTERNATE(GPIOB_JTAG_TRST) |  \
                                     PIN_MODE_INPUT(GPIOB_PIN5) |           \
                                     PIN_MODE_INPUT(GPIOB_PIN6) |          \
                                     PIN_MODE_INPUT(GPIOB_PIN7) |          \
                                     PIN_MODE_INPUT(GPIOB_PIN8) |           \
                                     PIN_MODE_INPUT(GPIOB_PIN9) |           \
                                     PIN_MODE_INPUT(GPIOB_PIN10) |          \
                                     PIN_MODE_INPUT(GPIOB_PIN11) |          \
                                     PIN_MODE_INPUT(GPIOB_PIN12) |          \
                                     PIN_MODE_INPUT(GPIOB_PIN13) |          \
                                     PIN_MODE_INPUT(GPIOB_PIN14) |          \
                                     PIN_MODE_INPUT(GPIOB_PIN15))
#define VAL_GPIOB_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOB_PIN0) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN1) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOB_BOOT1) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOB_JTAG_TDO) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOB_JTAG_TRST) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN5) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN6) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN7) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN8) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN9) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN10) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN11) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN12) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN13) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN14) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN15))
#define VAL_GPIOB_OSPEEDR           (PIN_OSPEED_400K(GPIOB_PIN0) |          \
                                     PIN_OSPEED_400K(GPIOB_PIN1) |          \
                                     PIN_OSPEED_400K(GPIOB_BOOT1) |         \
                                     PIN_OSPEED_40M(GPIOB_JTAG_TDO) |       \
                                     PIN_OSPEED_40M(GPIOB_JTAG_TRST) |      \
                                     PIN_OSPEED_400K(GPIOB_PIN5) |          \
                                     PIN_OSPEED_40M(GPIOB_PIN6) |           \
                                     PIN_OSPEED_40M(GPIOB_PIN7) |           \
                                     PIN_OSPEED_400K(GPIOB_PIN8) |          \
                                     PIN_OSPEED_400K(GPIOB_PIN9) |          \
                                     PIN_OSPEED_400K(GPIOB_PIN10) |         \
                                     PIN_OSPEED_400K(GPIOB_PIN11) |         \
                                     PIN_OSPEED_400K(GPIOB_PIN12) |         \
                                     PIN_OSPEED_400K(GPIOB_PIN13) |         \
                                     PIN_OSPEED_400K(GPIOB_PIN14) |         \
                                     PIN_OSPEED_400K(GPIOB_PIN15))
#define VAL_GPIOB_PUPDR             (PIN_PUPDR_PULLDOWN(GPIOB_PIN0) |       \
                                     PIN_PUPDR_PULLDOWN(GPIOB_PIN1) |       \
                                     PIN_PUPDR_FLOATING(GPIOB_BOOT1) |      \
                                     PIN_PUPDR_FLOATING(GPIOB_JTAG_TDO) |   \
                                     PIN_PUPDR_PULLDOWN(GPIOB_JTAG_TRST) |    \
                                     PIN_PUPDR_PULLDOWN(GPIOB_PIN5) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOB_PIN6) |       \
                                     PIN_PUPDR_PULLDOWN(GPIOB_PIN7) |       \
                                     PIN_PUPDR_PULLDOWN(GPIOB_PIN8) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOB_PIN9) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOB_PIN10) |        \
                                     PIN_PUPDR_PULLDOWN(GPIOB_PIN11) |        \
                                     PIN_PUPDR_PULLDOWN(GPIOB_PIN12) |        \
                                     PIN_PUPDR_PULLDOWN(GPIOB_PIN13) |        \
                                     PIN_PUPDR_PULLDOWN(GPIOB_PIN14) |        \
                                     PIN_PUPDR_PULLDOWN(GPIOB_PIN15))
#define VAL_GPIOB_ODR               (PIN_ODR_HIGH(GPIOB_PIN0) |             \
                                     PIN_ODR_HIGH(GPIOB_PIN1) |             \
                                     PIN_ODR_HIGH(GPIOB_BOOT1) |            \
                                     PIN_ODR_HIGH(GPIOB_JTAG_TDO) |         \
                                     PIN_ODR_HIGH(GPIOB_JTAG_TRST) |        \
                                     PIN_ODR_HIGH(GPIOB_PIN5) |             \
                                     PIN_ODR_LOW(GPIOB_PIN6) |              \
                                     PIN_ODR_LOW(GPIOB_PIN7) |              \
                                     PIN_ODR_HIGH(GPIOB_PIN8) |             \
                                     PIN_ODR_HIGH(GPIOB_PIN9) |             \
                                     PIN_ODR_HIGH(GPIOB_PIN10) |            \
                                     PIN_ODR_HIGH(GPIOB_PIN11) |            \
                                     PIN_ODR_HIGH(GPIOB_PIN12) |            \
                                     PIN_ODR_HIGH(GPIOB_PIN13) |            \
                                     PIN_ODR_HIGH(GPIOB_PIN14) |            \
                                     PIN_ODR_HIGH(GPIOB_PIN15))
#define VAL_GPIOB_AFRL              (PIN_AFIO_AF(GPIOB_PIN0, 0) |           \
                                     PIN_AFIO_AF(GPIOB_PIN1, 0) |           \
                                     PIN_AFIO_AF(GPIOB_BOOT1, 0) |          \
                                     PIN_AFIO_AF(GPIOB_JTAG_TDO, 0) |       \
                                     PIN_AFIO_AF(GPIOB_JTAG_TRST, 0) |      \
                                     PIN_AFIO_AF(GPIOB_PIN5, 0) |           \
                                     PIN_AFIO_AF(GPIOB_PIN6, 0) |           \
                                     PIN_AFIO_AF(GPIOB_PIN7, 0))
#define VAL_GPIOB_AFRH              (PIN_AFIO_AF(GPIOB_PIN8, 0) |           \
                                     PIN_AFIO_AF(GPIOB_PIN9, 0) |           \
                                     PIN_AFIO_AF(GPIOB_PIN10, 0) |          \
                                     PIN_AFIO_AF(GPIOB_PIN11, 0) |          \
                                     PIN_AFIO_AF(GPIOB_PIN12, 0) |          \
                                     PIN_AFIO_AF(GPIOB_PIN13, 0) |          \
                                     PIN_AFIO_AF(GPIOB_PIN14, 0) |          \
                                     PIN_AFIO_AF(GPIOB_PIN15, 0))

/*
 * GPIOC setup:
 *
 * PC0  - PIN0                      (input pulldown).
 * PC1  - PIN1                      (input pulldown).
 * PC2  - PIN2                      (input pulldown).
 * PC3  - PIN3                      (input pulldown).
 * PC4  - PIN4                      (input pulldown).
 * PC5  - PIN5                      (input pulldown).
 * PC6  - PIN6                      (input pulldown).
 * PC7  - PIN7                      (input pulldown).
 * PC8  - PIN8                      (input pulldown).
 * PC9  - PIN9                      (input pulldown).
 * PC10 - PIN10                     (input pulldown).
 * PC11 - PIN11                     (input pulldown).
 * PC12 - PIN12                     (input pulldown).
 * PC13 - PIN13                     (input pulldown).
 * PC14 - OSC32_IN                  (input floating).
 * PC15 - OSC32_OUT                 (input floating).
 */
#define VAL_GPIOC_MODER             (PIN_MODE_INPUT(GPIOC_PIN0) |           \
                                     PIN_MODE_INPUT(GPIOC_PIN1) |           \
                                     PIN_MODE_INPUT(GPIOC_PIN2) |           \
                                     PIN_MODE_INPUT(GPIOC_PIN3) |           \
                                     PIN_MODE_INPUT(GPIOC_PIN4) |           \
                                     PIN_MODE_INPUT(GPIOC_PIN5) |           \
                                     PIN_MODE_INPUT(GPIOC_PIN6) |           \
                                     PIN_MODE_INPUT(GPIOC_PIN7) |           \
                                     PIN_MODE_INPUT(GPIOC_PIN8) |           \
                                     PIN_MODE_INPUT(GPIOC_PIN9) |           \
                                     PIN_MODE_INPUT(GPIOC_PIN10) |          \
                                     PIN_MODE_INPUT(GPIOC_PIN11) |          \
                                     PIN_MODE_INPUT(GPIOC_PIN12) |          \
                                     PIN_MODE_INPUT(GPIOC_PIN13) |          \
                                     PIN_MODE_INPUT(GPIOC_OSC32_IN) |       \
                                     PIN_MODE_INPUT(GPIOC_OSC32_OUT))
#define VAL_GPIOC_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOC_PIN0) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN1) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN2) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN3) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN4) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN5) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN6) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN7) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN8) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN9) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN10) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN11) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN12) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN13) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOC_OSC32_IN) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOC_OSC32_OUT))
#define VAL_GPIOC_OSPEEDR           (PIN_OSPEED_400K(GPIOC_PIN0) |          \
                                     PIN_OSPEED_400K(GPIOC_PIN1) |          \
                                     PIN_OSPEED_400K(GPIOC_PIN2) |          \
                                     PIN_OSPEED_400K(GPIOC_PIN3) |          \
                                     PIN_OSPEED_400K(GPIOC_PIN4) |          \
                                     PIN_OSPEED_400K(GPIOC_PIN5) |          \
                                     PIN_OSPEED_400K(GPIOC_PIN6) |          \
                                     PIN_OSPEED_400K(GPIOC_PIN7) |          \
                                     PIN_OSPEED_400K(GPIOC_PIN8) |          \
                                     PIN_OSPEED_400K(GPIOC_PIN9) |          \
                                     PIN_OSPEED_400K(GPIOC_PIN10) |         \
                                     PIN_OSPEED_400K(GPIOC_PIN11) |         \
                                     PIN_OSPEED_400K(GPIOC_PIN12) |         \
                                     PIN_OSPEED_400K(GPIOC_PIN13) |         \
                                     PIN_OSPEED_40M(GPIOC_OSC32_IN) |       \
                                     PIN_OSPEED_40M(GPIOC_OSC32_OUT))
#define VAL_GPIOC_PUPDR             (PIN_PUPDR_PULLDOWN(GPIOC_PIN0) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOC_PIN1) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOC_PIN2) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOC_PIN3) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOC_PIN4) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOC_PIN5) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOC_PIN6) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOC_PIN7) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOC_PIN8) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOC_PIN9) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOC_PIN10) |        \
                                     PIN_PUPDR_PULLDOWN(GPIOC_PIN11) |        \
                                     PIN_PUPDR_PULLDOWN(GPIOC_PIN12) |        \
                                     PIN_PUPDR_PULLDOWN(GPIOC_PIN13) |        \
                                     PIN_PUPDR_FLOATING(GPIOC_OSC32_IN) |   \
                                     PIN_PUPDR_FLOATING(GPIOC_OSC32_OUT))
#define VAL_GPIOC_ODR               (PIN_ODR_HIGH(GPIOC_PIN0) |             \
                                     PIN_ODR_HIGH(GPIOC_PIN1) |             \
                                     PIN_ODR_HIGH(GPIOC_PIN2) |             \
                                     PIN_ODR_HIGH(GPIOC_PIN3) |             \
                                     PIN_ODR_HIGH(GPIOC_PIN4) |             \
                                     PIN_ODR_HIGH(GPIOC_PIN5) |             \
                                     PIN_ODR_HIGH(GPIOC_PIN6) |             \
                                     PIN_ODR_HIGH(GPIOC_PIN7) |             \
                                     PIN_ODR_HIGH(GPIOC_PIN8) |             \
                                     PIN_ODR_HIGH(GPIOC_PIN9) |             \
                                     PIN_ODR_HIGH(GPIOC_PIN10) |            \
                                     PIN_ODR_HIGH(GPIOC_PIN11) |            \
                                     PIN_ODR_HIGH(GPIOC_PIN12) |            \
                                     PIN_ODR_HIGH(GPIOC_PIN13) |            \
                                     PIN_ODR_HIGH(GPIOC_OSC32_IN) |         \
                                     PIN_ODR_HIGH(GPIOC_OSC32_OUT))
#define VAL_GPIOC_AFRL              (PIN_AFIO_AF(GPIOC_PIN0, 0) |           \
                                     PIN_AFIO_AF(GPIOC_PIN1, 0) |           \
                                     PIN_AFIO_AF(GPIOC_PIN2, 0) |           \
                                     PIN_AFIO_AF(GPIOC_PIN3, 0) |           \
                                     PIN_AFIO_AF(GPIOC_PIN4, 0) |           \
                                     PIN_AFIO_AF(GPIOC_PIN5, 0) |           \
                                     PIN_AFIO_AF(GPIOC_PIN6, 0) |           \
                                     PIN_AFIO_AF(GPIOC_PIN7, 0))
#define VAL_GPIOC_AFRH              (PIN_AFIO_AF(GPIOC_PIN8, 0) |           \
                                     PIN_AFIO_AF(GPIOC_PIN9, 0) |           \
                                     PIN_AFIO_AF(GPIOC_PIN10, 0) |          \
                                     PIN_AFIO_AF(GPIOC_PIN11, 0) |          \
                                     PIN_AFIO_AF(GPIOC_PIN12, 0) |          \
                                     PIN_AFIO_AF(GPIOC_PIN13, 0) |          \
                                     PIN_AFIO_AF(GPIOC_OSC32_IN, 0) |       \
                                     PIN_AFIO_AF(GPIOC_OSC32_OUT, 0))

/*
 * GPIOD setup:
 *
 * PD0  - PIN0                      (input pulldown).
 * PD1  - PIN1                      (input pulldown).
 * PD2  - PIN2                      (input pulldown).
 * PD3  - PIN3                      (input pulldown).
 * PD4  - PIN4                      (input pulldown).
 * PD5  - PIN5                      (input pulldown).
 * PD6  - PIN6                      (input pulldown).
 * PD7  - PIN7                      (input pulldown).
 * PD8  - PIN8                      (input pulldown).
 * PD9  - PIN9                      (input pulldown).
 * PD10 - PIN10                     (input pulldown).
 * PD11 - PIN11                     (input pulldown).
 * PD12 - PIN12                     (input pulldown).
 * PD13 - PIN13                     (input pulldown).
 * PD14 - PIN14                     (input pulldown).
 * PD15 - PIN15                     (input pulldown).
 */
#define VAL_GPIOD_MODER             (PIN_MODE_INPUT(GPIOD_PIN0) |           \
                                     PIN_MODE_INPUT(GPIOD_PIN1) |           \
                                     PIN_MODE_INPUT(GPIOD_PIN2) |           \
                                     PIN_MODE_INPUT(GPIOD_PIN3) |           \
                                     PIN_MODE_INPUT(GPIOD_PIN4) |           \
                                     PIN_MODE_INPUT(GPIOD_PIN5) |           \
                                     PIN_MODE_INPUT(GPIOD_PIN6) |           \
                                     PIN_MODE_INPUT(GPIOD_PIN7) |           \
                                     PIN_MODE_INPUT(GPIOD_PIN8) |           \
                                     PIN_MODE_INPUT(GPIOD_PIN9) |           \
                                     PIN_MODE_INPUT(GPIOD_PIN10) |          \
                                     PIN_MODE_INPUT(GPIOD_PIN11) |          \
                                     PIN_MODE_INPUT(GPIOD_PIN12) |          \
                                     PIN_MODE_INPUT(GPIOD_PIN13) |          \
                                     PIN_MODE_INPUT(GPIOD_PIN14) |          \
                                     PIN_MODE_INPUT(GPIOD_PIN15))
#define VAL_GPIOD_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOD_PIN0) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN1) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN2) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN3) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN4) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN5) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN6) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN7) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN8) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN9) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN10) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN11) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN12) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN13) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN14) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN15))
#define VAL_GPIOD_OSPEEDR           (PIN_OSPEED_400K(GPIOD_PIN0) |          \
                                     PIN_OSPEED_400K(GPIOD_PIN1) |          \
                                     PIN_OSPEED_400K(GPIOD_PIN2) |          \
                                     PIN_OSPEED_400K(GPIOD_PIN3) |          \
                                     PIN_OSPEED_400K(GPIOD_PIN4) |          \
                                     PIN_OSPEED_400K(GPIOD_PIN5) |          \
                                     PIN_OSPEED_400K(GPIOD_PIN6) |          \
                                     PIN_OSPEED_400K(GPIOD_PIN7) |          \
                                     PIN_OSPEED_400K(GPIOD_PIN8) |          \
                                     PIN_OSPEED_400K(GPIOD_PIN9) |          \
                                     PIN_OSPEED_400K(GPIOD_PIN10) |         \
                                     PIN_OSPEED_400K(GPIOD_PIN11) |         \
                                     PIN_OSPEED_400K(GPIOD_PIN12) |         \
                                     PIN_OSPEED_400K(GPIOD_PIN13) |         \
                                     PIN_OSPEED_400K(GPIOD_PIN14) |         \
                                     PIN_OSPEED_400K(GPIOD_PIN15))
#define VAL_GPIOD_PUPDR             (PIN_PUPDR_PULLDOWN(GPIOD_PIN0) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOD_PIN1) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOD_PIN2) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOD_PIN3) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOD_PIN4) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOD_PIN5) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOD_PIN6) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOD_PIN7) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOD_PIN8) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOD_PIN9) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOD_PIN10) |        \
                                     PIN_PUPDR_PULLDOWN(GPIOD_PIN11) |        \
                                     PIN_PUPDR_PULLDOWN(GPIOD_PIN12) |        \
                                     PIN_PUPDR_PULLDOWN(GPIOD_PIN13) |        \
                                     PIN_PUPDR_PULLDOWN(GPIOD_PIN14) |        \
                                     PIN_PUPDR_PULLDOWN(GPIOD_PIN15))
#define VAL_GPIOD_ODR               (PIN_ODR_HIGH(GPIOD_PIN0) |             \
                                     PIN_ODR_HIGH(GPIOD_PIN1) |             \
                                     PIN_ODR_HIGH(GPIOD_PIN2) |             \
                                     PIN_ODR_HIGH(GPIOD_PIN3) |             \
                                     PIN_ODR_HIGH(GPIOD_PIN4) |             \
                                     PIN_ODR_HIGH(GPIOD_PIN5) |             \
                                     PIN_ODR_HIGH(GPIOD_PIN6) |             \
                                     PIN_ODR_HIGH(GPIOD_PIN7) |             \
                                     PIN_ODR_HIGH(GPIOD_PIN8) |             \
                                     PIN_ODR_HIGH(GPIOD_PIN9) |             \
                                     PIN_ODR_HIGH(GPIOD_PIN10) |            \
                                     PIN_ODR_HIGH(GPIOD_PIN11) |            \
                                     PIN_ODR_HIGH(GPIOD_PIN12) |            \
                                     PIN_ODR_HIGH(GPIOD_PIN13) |            \
                                     PIN_ODR_HIGH(GPIOD_PIN14) |            \
                                     PIN_ODR_HIGH(GPIOD_PIN15))
#define VAL_GPIOD_AFRL              (PIN_AFIO_AF(GPIOD_PIN0, 0) |           \
                                     PIN_AFIO_AF(GPIOD_PIN1, 0) |           \
                                     PIN_AFIO_AF(GPIOD_PIN2, 0) |           \
                                     PIN_AFIO_AF(GPIOD_PIN3, 0) |           \
                                     PIN_AFIO_AF(GPIOD_PIN4, 0) |           \
                                     PIN_AFIO_AF(GPIOD_PIN5, 0) |           \
                                     PIN_AFIO_AF(GPIOD_PIN6, 0) |           \
                                     PIN_AFIO_AF(GPIOD_PIN7, 0))
#define VAL_GPIOD_AFRH              (PIN_AFIO_AF(GPIOD_PIN8, 0) |           \
                                     PIN_AFIO_AF(GPIOD_PIN9, 0) |           \
                                     PIN_AFIO_AF(GPIOD_PIN10, 0) |          \
                                     PIN_AFIO_AF(GPIOD_PIN11, 0) |          \
                                     PIN_AFIO_AF(GPIOD_PIN12, 0) |          \
                                     PIN_AFIO_AF(GPIOD_PIN13, 0) |          \
                                     PIN_AFIO_AF(GPIOD_PIN14, 0) |          \
                                     PIN_AFIO_AF(GPIOD_PIN15, 0))

/*
 * GPIOE setup:
 *
 * PE0  - PIN0                      (input pulldown).
 * PE1  - PIN1                      (input pulldown).
 * PE2  - PIN2                      (input pulldown).
 * PE3  - PIN3                      (input pulldown).
 * PE4  - PIN4                      (input pulldown).
 * PE5  - PIN5                      (input pulldown).
 * PE6  - PIN6                      (input pulldown).
 * PE7  - PIN7                      (input pulldown).
 * PE8  - PIN8                      (input pulldown).
 * PE9  - PIN9                      (input pulldown).
 * PE10 - PIN10                     (input pulldown).
 * PE11 - PIN11                     (input pulldown).
 * PE12 - PIN12                     (input pulldown).
 * PE13 - PIN13                     (input pulldown).
 * PE14 - PIN14                     (input pulldown).
 * PE15 - PIN15                     (input pulldown).
 */
#define VAL_GPIOE_MODER             (PIN_MODE_INPUT(GPIOE_PIN0) |           \
                                     PIN_MODE_INPUT(GPIOE_PIN1) |           \
                                     PIN_MODE_INPUT(GPIOE_PIN2) |           \
                                     PIN_MODE_INPUT(GPIOE_PIN3) |           \
                                     PIN_MODE_INPUT(GPIOE_PIN4) |           \
                                     PIN_MODE_INPUT(GPIOE_PIN5) |           \
                                     PIN_MODE_INPUT(GPIOE_PIN6) |           \
                                     PIN_MODE_INPUT(GPIOE_PIN7) |           \
                                     PIN_MODE_INPUT(GPIOE_PIN8) |           \
                                     PIN_MODE_INPUT(GPIOE_PIN9) |           \
                                     PIN_MODE_INPUT(GPIOE_PIN10) |          \
                                     PIN_MODE_INPUT(GPIOE_PIN11) |          \
                                     PIN_MODE_INPUT(GPIOE_PIN12) |          \
                                     PIN_MODE_INPUT(GPIOE_PIN13) |          \
                                     PIN_MODE_INPUT(GPIOE_PIN14) |          \
                                     PIN_MODE_INPUT(GPIOE_PIN15))
#define VAL_GPIOE_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOE_PIN0) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN1) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN2) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN3) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN4) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN5) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN6) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN7) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN8) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN9) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN10) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN11) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN12) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN13) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN14) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN15))
#define VAL_GPIOE_OSPEEDR           (PIN_OSPEED_400K(GPIOE_PIN0) |          \
                                     PIN_OSPEED_400K(GPIOE_PIN1) |          \
                                     PIN_OSPEED_400K(GPIOE_PIN2) |          \
                                     PIN_OSPEED_400K(GPIOE_PIN3) |          \
                                     PIN_OSPEED_400K(GPIOE_PIN4) |          \
                                     PIN_OSPEED_400K(GPIOE_PIN5) |          \
                                     PIN_OSPEED_400K(GPIOE_PIN6) |          \
                                     PIN_OSPEED_400K(GPIOE_PIN7) |          \
                                     PIN_OSPEED_400K(GPIOE_PIN8) |          \
                                     PIN_OSPEED_400K(GPIOE_PIN9) |          \
                                     PIN_OSPEED_400K(GPIOE_PIN10) |         \
                                     PIN_OSPEED_400K(GPIOE_PIN11) |         \
                                     PIN_OSPEED_400K(GPIOE_PIN12) |         \
                                     PIN_OSPEED_400K(GPIOE_PIN13) |         \
                                     PIN_OSPEED_400K(GPIOE_PIN14) |         \
                                     PIN_OSPEED_400K(GPIOE_PIN15))
#define VAL_GPIOE_PUPDR             (PIN_PUPDR_PULLDOWN(GPIOE_PIN0) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOE_PIN1) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOE_PIN2) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOE_PIN3) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOE_PIN4) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOE_PIN5) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOE_PIN6) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOE_PIN7) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOE_PIN8) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOE_PIN9) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOE_PIN10) |        \
                                     PIN_PUPDR_PULLDOWN(GPIOE_PIN11) |        \
                                     PIN_PUPDR_PULLDOWN(GPIOE_PIN12) |        \
                                     PIN_PUPDR_PULLDOWN(GPIOE_PIN13) |        \
                                     PIN_PUPDR_PULLDOWN(GPIOE_PIN14) |        \
                                     PIN_PUPDR_PULLDOWN(GPIOE_PIN15))
#define VAL_GPIOE_ODR               (PIN_ODR_HIGH(GPIOE_PIN0) |             \
                                     PIN_ODR_HIGH(GPIOE_PIN1) |             \
                                     PIN_ODR_HIGH(GPIOE_PIN2) |             \
                                     PIN_ODR_HIGH(GPIOE_PIN3) |             \
                                     PIN_ODR_HIGH(GPIOE_PIN4) |             \
                                     PIN_ODR_HIGH(GPIOE_PIN5) |             \
                                     PIN_ODR_HIGH(GPIOE_PIN6) |             \
                                     PIN_ODR_HIGH(GPIOE_PIN7) |             \
                                     PIN_ODR_HIGH(GPIOE_PIN8) |             \
                                     PIN_ODR_HIGH(GPIOE_PIN9) |             \
                                     PIN_ODR_HIGH(GPIOE_PIN10) |            \
                                     PIN_ODR_HIGH(GPIOE_PIN11) |            \
                                     PIN_ODR_HIGH(GPIOE_PIN12) |            \
                                     PIN_ODR_HIGH(GPIOE_PIN13) |            \
                                     PIN_ODR_HIGH(GPIOE_PIN14) |            \
                                     PIN_ODR_HIGH(GPIOE_PIN15))
#define VAL_GPIOE_AFRL              (PIN_AFIO_AF(GPIOE_PIN0, 0) |           \
                                     PIN_AFIO_AF(GPIOE_PIN1, 0) |           \
                                     PIN_AFIO_AF(GPIOE_PIN2, 0) |           \
                                     PIN_AFIO_AF(GPIOE_PIN3, 0) |           \
                                     PIN_AFIO_AF(GPIOE_PIN4, 0) |           \
                                     PIN_AFIO_AF(GPIOE_PIN5, 0) |           \
                                     PIN_AFIO_AF(GPIOE_PIN6, 0) |           \
                                     PIN_AFIO_AF(GPIOE_PIN7, 0))
#define VAL_GPIOE_AFRH              (PIN_AFIO_AF(GPIOE_PIN8, 0) |           \
                                     PIN_AFIO_AF(GPIOE_PIN9, 0) |           \
                                     PIN_AFIO_AF(GPIOE_PIN10, 0) |          \
                                     PIN_AFIO_AF(GPIOE_PIN11, 0) |          \
                                     PIN_AFIO_AF(GPIOE_PIN12, 0) |          \
                                     PIN_AFIO_AF(GPIOE_PIN13, 0) |          \
                                     PIN_AFIO_AF(GPIOE_PIN14, 0) |          \
                                     PIN_AFIO_AF(GPIOE_PIN15, 0))

/*
 * GPIOH setup:
 *
 * PH0  - OSC_IN                    (input floating).
 * PH1  - OSC_OUT                   (input floating).
 * PH2  - PIN2                      (input pulldown).
 * PH3  - PIN3                      (input pulldown).
 * PH4  - PIN4                      (input pulldown).
 * PH5  - PIN5                      (input pulldown).
 * PH6  - PIN6                      (input pulldown).
 * PH7  - PIN7                      (input pulldown).
 * PH8  - PIN8                      (input pulldown).
 * PH9  - PIN9                      (input pulldown).
 * PH10 - PIN10                     (input pulldown).
 * PH11 - PIN11                     (input pulldown).
 * PH12 - PIN12                     (input pulldown).
 * PH13 - PIN13                     (input pulldown).
 * PH14 - PIN14                     (input pulldown).
 * PH15 - PIN15                     (input pulldown).
 */
#define VAL_GPIOH_MODER             (PIN_MODE_INPUT(GPIOH_OSC_IN) |         \
                                     PIN_MODE_INPUT(GPIOH_OSC_OUT) |        \
                                     PIN_MODE_INPUT(GPIOH_PIN2) |           \
                                     PIN_MODE_INPUT(GPIOH_PIN3) |           \
                                     PIN_MODE_INPUT(GPIOH_PIN4) |           \
                                     PIN_MODE_INPUT(GPIOH_PIN5) |           \
                                     PIN_MODE_INPUT(GPIOH_PIN6) |           \
                                     PIN_MODE_INPUT(GPIOH_PIN7) |           \
                                     PIN_MODE_INPUT(GPIOH_PIN8) |           \
                                     PIN_MODE_INPUT(GPIOH_PIN9) |           \
                                     PIN_MODE_INPUT(GPIOH_PIN10) |          \
                                     PIN_MODE_INPUT(GPIOH_PIN11) |          \
                                     PIN_MODE_INPUT(GPIOH_PIN12) |          \
                                     PIN_MODE_INPUT(GPIOH_PIN13) |          \
                                     PIN_MODE_INPUT(GPIOH_PIN14) |          \
                                     PIN_MODE_INPUT(GPIOH_PIN15))
#define VAL_GPIOH_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOH_OSC_IN) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOH_OSC_OUT) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN2) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN3) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN4) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN5) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN6) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN7) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN8) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN9) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN10) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN11) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN12) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN13) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN14) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN15))
#define VAL_GPIOH_OSPEEDR           (PIN_OSPEED_40M(GPIOH_OSC_IN) |         \
                                     PIN_OSPEED_40M(GPIOH_OSC_OUT) |        \
                                     PIN_OSPEED_400K(GPIOH_PIN2) |          \
                                     PIN_OSPEED_400K(GPIOH_PIN3) |          \
                                     PIN_OSPEED_400K(GPIOH_PIN4) |          \
                                     PIN_OSPEED_400K(GPIOH_PIN5) |          \
                                     PIN_OSPEED_400K(GPIOH_PIN6) |          \
                                     PIN_OSPEED_400K(GPIOH_PIN7) |          \
                                     PIN_OSPEED_400K(GPIOH_PIN8) |          \
                                     PIN_OSPEED_400K(GPIOH_PIN9) |          \
                                     PIN_OSPEED_400K(GPIOH_PIN10) |         \
                                     PIN_OSPEED_400K(GPIOH_PIN11) |         \
                                     PIN_OSPEED_400K(GPIOH_PIN12) |         \
                                     PIN_OSPEED_400K(GPIOH_PIN13) |         \
                                     PIN_OSPEED_400K(GPIOH_PIN14) |         \
                                     PIN_OSPEED_400K(GPIOH_PIN15))
#define VAL_GPIOH_PUPDR             (PIN_PUPDR_FLOATING(GPIOH_OSC_IN) |     \
                                     PIN_PUPDR_FLOATING(GPIOH_OSC_OUT) |    \
                                     PIN_PUPDR_PULLDOWN(GPIOH_PIN2) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOH_PIN3) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOH_PIN4) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOH_PIN5) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOH_PIN6) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOH_PIN7) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOH_PIN8) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOH_PIN9) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOH_PIN10) |        \
                                     PIN_PUPDR_PULLDOWN(GPIOH_PIN11) |        \
                                     PIN_PUPDR_PULLDOWN(GPIOH_PIN12) |        \
                                     PIN_PUPDR_PULLDOWN(GPIOH_PIN13) |        \
                                     PIN_PUPDR_PULLDOWN(GPIOH_PIN14) |        \
                                     PIN_PUPDR_PULLDOWN(GPIOH_PIN15))
#define VAL_GPIOH_ODR               (PIN_ODR_HIGH(GPIOH_OSC_IN) |           \
                                     PIN_ODR_HIGH(GPIOH_OSC_OUT) |          \
                                     PIN_ODR_HIGH(GPIOH_PIN2) |             \
                                     PIN_ODR_HIGH(GPIOH_PIN3) |             \
                                     PIN_ODR_HIGH(GPIOH_PIN4) |             \
                                     PIN_ODR_HIGH(GPIOH_PIN5) |             \
                                     PIN_ODR_HIGH(GPIOH_PIN6) |             \
                                     PIN_ODR_HIGH(GPIOH_PIN7) |             \
                                     PIN_ODR_HIGH(GPIOH_PIN8) |             \
                                     PIN_ODR_HIGH(GPIOH_PIN9) |             \
                                     PIN_ODR_HIGH(GPIOH_PIN10) |            \
                                     PIN_ODR_HIGH(GPIOH_PIN11) |            \
                                     PIN_ODR_HIGH(GPIOH_PIN12) |            \
                                     PIN_ODR_HIGH(GPIOH_PIN13) |            \
                                     PIN_ODR_HIGH(GPIOH_PIN14) |            \
                                     PIN_ODR_HIGH(GPIOH_PIN15))
#define VAL_GPIOH_AFRL              (PIN_AFIO_AF(GPIOH_OSC_IN, 0) |         \
                                     PIN_AFIO_AF(GPIOH_OSC_OUT, 0) |        \
                                     PIN_AFIO_AF(GPIOH_PIN2, 0) |           \
                                     PIN_AFIO_AF(GPIOH_PIN3, 0) |           \
                                     PIN_AFIO_AF(GPIOH_PIN4, 0) |           \
                                     PIN_AFIO_AF(GPIOH_PIN5, 0) |           \
                                     PIN_AFIO_AF(GPIOH_PIN6, 0) |           \
                                     PIN_AFIO_AF(GPIOH_PIN7, 0))
#define VAL_GPIOH_AFRH              (PIN_AFIO_AF(GPIOH_PIN8, 0) |           \
                                     PIN_AFIO_AF(GPIOH_PIN9, 0) |           \
                                     PIN_AFIO_AF(GPIOH_PIN10, 0) |          \
                                     PIN_AFIO_AF(GPIOH_PIN11, 0) |          \
                                     PIN_AFIO_AF(GPIOH_PIN12, 0) |          \
                                     PIN_AFIO_AF(GPIOH_PIN13, 0) |          \
                                     PIN_AFIO_AF(GPIOH_PIN14, 0) |          \
                                     PIN_AFIO_AF(GPIOH_PIN15, 0))


#if !defined(_FROM_ASM_)
#ifdef __cplusplus
extern "C" {
#endif
  void boardInit(void);
#ifdef __cplusplus
}
#endif
#endif /* _FROM_ASM_ */

#endif /* _BOARD_H_ */
