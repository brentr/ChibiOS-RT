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

//from http://koti.kapsi.fi/jpa/stuff/other/stm32-hardfault-backtrace.html
//  restore process stack pointer so debugger can backtrace from hard faults
register void *stack_pointer asm("sp");

/**
* Executes the BKPT instruction that causes the debugger to stop.
* If no debugger is attached, this will be ignored
*/
#define bkpt() __asm volatile("BKPT #0\n")

static inline void crash(void)
{
  chSysDisable();   //ignore subsequent interrupts
  bkpt();           //use bt to see which trap was unhandled
  chSysHalt();      //restart applicaiton
}

#if !defined(CH_DEBUG_FAULTS) || defined(__DOXYGEN__)

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

#define unhandledSpecial(exception) unhandled(exception)

#else  //if unoptimized compile, create unique exception handlers to aid debug

/*!
* \file cortex_hardfault_handler.c --
* from http://mcufreaks.blogspot.com/2013/03/hard-fault-debugging-for-cortex-m-mcus.html
* \brief The code below implements a mechanism to discover HARD_FAULT sources in Cortex-M embedded applications.
* \version mcufreaks.blogspot.com
*/
/*!
* \note The following declaration is mandatory to avoid compiler errors.
* In the declaration below, we are assigning the assembler label __label_hardfaultGetContext__
* to the entry point of __hardfaultGetContext__ function
*/
void hardfaultGetContext(unsigned long* stackedContextPtr) asm("label_hardfaultGetContext");
/*!
* \fn void hardfaultGetContext(unsigned long* stackedContextPtr)
* \brief Copies system stacked context into function local variables.
* This function is called from asm-coded Interrupt Service Routine associated to HARD_FAULT exception
* \param stackedContextPtr : Address of stack containing stacked processor context.
*/
void __attribute__((naked,noreturn)) hardfaultGetContext(unsigned long* stackedContextPtr)
{
static volatile unsigned long stacked_r0;
static volatile unsigned long stacked_r1;
static volatile unsigned long stacked_r2;
static volatile unsigned long stacked_r3;
static volatile unsigned long stacked_r12;
static volatile unsigned long stacked_lr;
static volatile unsigned long stacked_pc;
static volatile unsigned long stacked_psr;
static volatile unsigned long _CFSR;
static volatile unsigned long _HFSR;
static volatile unsigned long _DFSR;
static volatile unsigned long _AFSR;
static volatile unsigned long _BFAR;
static volatile unsigned long _MMAR;
stacked_r0 = stackedContextPtr[0];
stacked_r1 = stackedContextPtr[1];
stacked_r2 = stackedContextPtr[2];
stacked_r3 = stackedContextPtr[3];
stacked_r12 = stackedContextPtr[4];
stacked_lr = stackedContextPtr[5];
stacked_pc = stackedContextPtr[6];
stacked_psr = stackedContextPtr[7];
// Configurable Fault Status Register
// Consists of MMSR, BFSR and UFSR
_CFSR = (*((volatile unsigned long *)(0xE000ED28))) ;
// Hard Fault Status Register
_HFSR = (*((volatile unsigned long *)(0xE000ED2C))) ;
// Debug Fault Status Register
_DFSR = (*((volatile unsigned long *)(0xE000ED30))) ;
// Auxiliary Fault Status Register
_AFSR = (*((volatile unsigned long *)(0xE000ED3C))) ;
// Read the Fault Address Registers. These may not contain valid values.
// Check BFARVALID/MMARVALID to see if they are valid values
// MemManage Fault Address Register
_MMAR = (*((volatile unsigned long *)(0xE000ED34))) ;
// Bus Fault Address Register
_BFAR = (*((volatile unsigned long *)(0xE000ED38))) ;

stack_pointer = stackedContextPtr;
bkpt(); //'info locals' shows regs, 'l *stacked_pc' shows fault site.
chSysHalt();

// The following code avoids compiler warning [-Wunused-but-set-variable]
stackedContextPtr[0] = stacked_r0;
stackedContextPtr[1] = stacked_r1;
stackedContextPtr[2] = stacked_r2;
stackedContextPtr[3] = stacked_r3;
stackedContextPtr[4] = stacked_r12;
stackedContextPtr[5] = stacked_lr;
stackedContextPtr[6] = stacked_pc;
stackedContextPtr[7] = stacked_psr;
(*((volatile unsigned long *)(0xE000ED28))) = _CFSR;
(*((volatile unsigned long *)(0xE000ED2C))) = _HFSR;
(*((volatile unsigned long *)(0xE000ED30))) = _DFSR;
(*((volatile unsigned long *)(0xE000ED3C))) = _AFSR;
(*((volatile unsigned long *)(0xE000ED34))) = _MMAR;
(*((volatile unsigned long *)(0xE000ED38))) = _BFAR;
}
/*!
* \fn void hardfaultHandler(void)
* \brief HARD_FAULT interrupt service routine. Selects among PSP or MSP stacks and \n
* calls \ref hardfaultGetContext passing the selected stack pointer address as parameter.
* \note __naked__ attribute avoids generating prologue and epilogue code sequences generated \n
* for C-functions, and only pure asm instructions should be included into the function body.
*/
void __attribute__((naked, interrupt)) _unhandled_HardFaultVector_exception(void)
{
  chSysDisable();
__asm__ volatile	(
" tst lr, #4    \n"
" ite eq        \n"
" mrseq r0, msp \n"
" mrsne r0, psp \n"
" B label_hardfaultGetContext"   /* pass MSP content as stackedContextPtr value. */
:: );
}

#define unhandledSpecial(exception)   \
  extern void exception(void); \
  __attribute__ ((naked))      \
  void exception(void)         \
    __attribute__((weak, alias("_unhandled_" #exception "_exception")))

#define unhandled(exception)   \
  void _unhandled_##exception##_exception(void) {crash();}\
  unhandledSpecial(exception);       \

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
unhandledSpecial(HardFaultVector);
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
