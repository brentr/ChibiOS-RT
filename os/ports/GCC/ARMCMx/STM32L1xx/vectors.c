/*
    ChibiOS/RT - Copyright (C) 2006,2007,2008,2009,2010,
                 2011,2012,2013 Giovanni Di Sirio.

    revised:  11/12/15  brent@mbari.org
              Use of new unhandled() macro avoids much repetition
              Disable all interrupts when unhandled exception occurs
              Compile unique unhandled handlers if not optimizing to aid debug

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

/**
 * @file    GCC/ARMCMx/STM32L1xx/vectors.c
 * @brief   Interrupt vectors for the STM32 family.
 *
 * @defgroup ARMCMx_STM32L1xx_VECTORS STM32L1xx Interrupt Vectors
 * @ingroup ARMCMx_SPECIFIC
 * @details Interrupt vectors for the STM32L1xx family.
 * @{
 */

#include "ch.h"
#include "hal.h"

/**
* Executes the BKPT instruction that causes the debugger to stop.
* If no debugger is attached, this will be ignored
*/
#define bkpt() __asm volatile("BKPT #0\n")
#define crash() do {chSysDisable(); bkpt(); NVIC_SystemReset(); } while(0)

#if __OPTIMIZE__ || defined(__DOXYGEN__)

/**
 * @brief   Generic Unhandled exceptions handler.
 * @details Any undefined exception vector points to this function by default.
 *          This function simply stops the system into an infinite loop.
 *
 * @notapi
 */
#if !defined(__DOXYGEN__)
__attribute__ ((naked))
#endif
void _unhandled_exception(void) {crash();}

#define unhandled(exception)   \
  void exception(void)         \
    __attribute__((weak, alias("_unhandled_exception")))

#else  //if unoptimized compile, create unique exception handlers to aid debug

#define unhandled(exception)   \
  extern void exception(void); \
  __attribute__ ((naked))      \
    void _unhandled_##exception##_exception(void) {crash();}\
  void exception(void)         \
    __attribute__((weak, alias("_unhandled_" #exception "_exception")))

#endif


/**
 * @brief   Type of an IRQ vector.
 */
typedef void  (*irq_vector_t)(void);

/**
 * @brief   Type of a structure representing the whole vectors table.
 */
typedef struct {
  uint32_t      *init_stack;
  irq_vector_t  reset_vector;
  irq_vector_t  nmi_vector;
  irq_vector_t  hardfault_vector;
  irq_vector_t  memmanage_vector;
  irq_vector_t  busfault_vector;
  irq_vector_t  usagefault_vector;
  irq_vector_t  vector1c;
  irq_vector_t  vector20;
  irq_vector_t  vector24;
  irq_vector_t  vector28;
  irq_vector_t  svcall_vector;
  irq_vector_t  debugmonitor_vector;
  irq_vector_t  vector34;
  irq_vector_t  pendsv_vector;
  irq_vector_t  systick_vector;
  irq_vector_t  vectors[45];
} vectors_t;


#if !defined(__DOXYGEN__)
extern uint32_t __main_stack_end__;
extern void ResetHandler(void);
#endif /* !defined(__DOXYGEN__) */

unhandled(NMIVector);
unhandled(HardFaultVector);
unhandled(MemManageVector);
unhandled(BusFaultVector);
unhandled(UsageFaultVector);
unhandled(Vector1C);
unhandled(Vector20);
unhandled(Vector24);
unhandled(Vector28);
unhandled(SVCallVector);
unhandled(DebugMonitorVector);
unhandled(Vector34);
unhandled(PendSVVector);
unhandled(SysTickVector);
unhandled(Vector40);
unhandled(Vector44);
unhandled(Vector48);
unhandled(Vector4C);
unhandled(Vector50);
unhandled(Vector54);
unhandled(Vector58);
unhandled(Vector5C);
unhandled(Vector60);
unhandled(Vector64);
unhandled(Vector68);
unhandled(Vector6C);
unhandled(Vector70);
unhandled(Vector74);
unhandled(Vector78);
unhandled(Vector7C);
unhandled(Vector80);
unhandled(Vector84);
unhandled(Vector88);
unhandled(Vector8C);
unhandled(Vector90);
unhandled(Vector94);
unhandled(Vector98);
unhandled(Vector9C);
unhandled(VectorA0);
unhandled(VectorA4);
unhandled(VectorA8);
unhandled(VectorAC);
unhandled(VectorB0);
unhandled(VectorB4);
unhandled(VectorB8);
unhandled(VectorBC);
unhandled(VectorC0);
unhandled(VectorC4);
unhandled(VectorC8);
unhandled(VectorCC);
unhandled(VectorD0);
unhandled(VectorD4);
unhandled(VectorD8);
unhandled(VectorDC);
unhandled(VectorE0);
unhandled(VectorE4);
unhandled(VectorE8);
unhandled(VectorEC);
unhandled(VectorF0);

/**
 * @brief   STM32L1xx vectors table.
 */
#if !defined(__DOXYGEN__)
__attribute__ ((section("vectors")))
#endif
vectors_t _vectors = {
  &__main_stack_end__,ResetHandler,       NMIVector,          HardFaultVector,
  MemManageVector,    BusFaultVector,     UsageFaultVector,   Vector1C,
  Vector20,           Vector24,           Vector28,           SVCallVector,
  DebugMonitorVector, Vector34,           PendSVVector,       SysTickVector,
  {
    Vector40,           Vector44,           Vector48,           Vector4C,
    Vector50,           Vector54,           Vector58,           Vector5C,
    Vector60,           Vector64,           Vector68,           Vector6C,
    Vector70,           Vector74,           Vector78,           Vector7C,
    Vector80,           Vector84,           Vector88,           Vector8C,
    Vector90,           Vector94,           Vector98,           Vector9C,
    VectorA0,           VectorA4,           VectorA8,           VectorAC,
    VectorB0,           VectorB4,           VectorB8,           VectorBC,
    VectorC0,           VectorC4,           VectorC8,           VectorCC,
    VectorD0,           VectorD4,           VectorD8,           VectorDC,
    VectorE0,           VectorE4,           VectorE8,           VectorEC,
    VectorF0
  }
};

/** @} */
