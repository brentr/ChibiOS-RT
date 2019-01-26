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

/**
 * @file    GCC/ARMCMx/chcore_v7m.c
 * @brief   ARMv7-M architecture port code.
 *
 * @addtogroup ARMCMx_V7M_CORE
 * @{
 */

#include "ch.h"

/*===========================================================================*/
/* Port interrupt handlers.                                                  */
/*===========================================================================*/

/**
 * @brief   System Timer vector.
 * @details This interrupt is used as system tick.
 * @note    The timer must be initialized in the startup code.
 */
CH_IRQ_HANDLER(SysTickVector) {

  CH_IRQ_PROLOGUE();

  chSysLockFromIsr();
  chSysTimerHandlerI();
  chSysUnlockFromIsr();

  CH_IRQ_EPILOGUE();
}

#if !CORTEX_SIMPLIFIED_PRIORITY || defined(__DOXYGEN__)
/**
 * @brief   SVCall vector.
 * @details The SVCall vector is used for exception mode re-entering after a
 *          context switch.
 * @note    The SVCallVector vector is only used in advanced kernel mode.
 */
void SVCallVector(void) {
  struct extctx *ctxp;

#if CORTEX_USE_FPU
  /* Enforcing unstacking of the FP part of the context.*/
  SCB_FPCCR &= ~FPCCR_LSPACT;
#endif

  /* Current PSP value.*/
  asm volatile ("mrs     %0, PSP" : "=r" (ctxp) : : "memory");

  /* Discarding the current exception context and positioning the stack to
     point to the real one.*/
  ctxp++;

  /* Restoring real position of the original stack frame.*/
  asm volatile ("msr     PSP, %0" : : "r" (ctxp) : "memory");
  port_unlock_from_isr();
}
#endif /* !CORTEX_SIMPLIFIED_PRIORITY */

#if CORTEX_SIMPLIFIED_PRIORITY || defined(__DOXYGEN__)
/**
 * @brief   PendSV vector.
 * @details The PendSV vector is used for exception mode re-entering after a
 *          context switch.
 * @note    The PendSV vector is only used in compact kernel mode.
 */
void PendSVVector(void) {
  struct extctx *ctxp;

#if CORTEX_USE_FPU
  /* Enforcing unstacking of the FP part of the context.*/
  SCB_FPCCR &= ~FPCCR_LSPACT;
#endif

  /* Current PSP value.*/
  asm volatile ("mrs     %0, PSP" : "=r" (ctxp) : : "memory");

  /* Discarding the current exception context and positioning the stack to
     point to the real one.*/
  ctxp++;

  /* Restoring real position of the original stack frame.*/
  asm volatile ("msr     PSP, %0" : : "r" (ctxp) : "memory");
}
#endif /* CORTEX_SIMPLIFIED_PRIORITY */

/*===========================================================================*/
/* Port exported functions.                                                  */
/*===========================================================================*/

/**
 * @brief   Port-related initialization code.
 */
void _port_init(void) {

  /* Initialization of the vector table and priority related settings.*/
#if 0  //do *not* set VTOR as if prevents using a bootloader to load app image
  SCB_VTOR = CORTEX_VTOR_INIT;
#endif
  SCB_AIRCR = AIRCR_VECTKEY | AIRCR_PRIGROUP(CORTEX_PRIGROUP_INIT);

  /* Initialization of the system vectors used by the port.*/
  nvicSetSystemHandlerPriority(HANDLER_SVCALL,
    CORTEX_PRIORITY_MASK(CORTEX_PRIORITY_SVCALL));
  nvicSetSystemHandlerPriority(HANDLER_PENDSV,
    CORTEX_PRIORITY_MASK(CORTEX_PRIORITY_PENDSV));
  nvicSetSystemHandlerPriority(HANDLER_SYSTICK,
    CORTEX_PRIORITY_MASK(CORTEX_PRIORITY_SYSTICK));
}

#if !CH_OPTIMIZE_SPEED
void _port_lock(void) {
  register uint32_t tmp asm ("r3") = CORTEX_BASEPRI_KERNEL;
  asm volatile ("msr     BASEPRI, %0" : : "r" (tmp) : "memory");
}

void _port_unlock(void) {
  register uint32_t tmp asm ("r3") = CORTEX_BASEPRI_DISABLED;
  asm volatile ("msr     BASEPRI, %0" : : "r" (tmp) : "memory");
}
#endif

/**
 * @brief   Exception exit redirection to _port_switch_from_isr().
 */
void _port_irq_epilogue(void) {

  port_lock_from_isr();
  if ((SCB_ICSR & ICSR_RETTOBASE) != 0) {
    struct extctx *ctxp;

#if CORTEX_USE_FPU
    /* Enforcing a lazy FPU state save. Note, it goes in the original
       context because the FPCAR register has not been modified.*/
    asm volatile ("vmrs    APSR_nzcv, FPSCR" : : : "memory");
#endif

    /* Current PSP value.*/
    asm volatile ("mrs     %0, PSP" : "=r" (ctxp) : : "memory");

    /* Adding an artificial exception return context, there is no need to
       populate it fully.*/
    ctxp--;
    ctxp->xpsr = (regarm_t)0x01000000;
#if CORTEX_USE_FPU
    ctxp->fpscr = (regarm_t)SCB_FPDSCR;
#endif
    asm volatile ("msr     PSP, %0" : : "r" (ctxp) : "memory");

    /* The exit sequence is different depending on if a preemption is
       required or not.*/
    if (chSchIsPreemptionRequired()) {
      /* Preemption is required we need to enforce a context switch.*/
      ctxp->pc = (regarm_t)_port_switch_from_isr;
    }
    else {
      /* Preemption not required, we just need to exit the exception
         atomically.*/
      ctxp->pc = (regarm_t)_port_exit_from_isr;
    }

    /* Note, returning without unlocking is intentional, this is done in
       order to keep the rest of the context switch atomic.*/
    return;
  }
  port_unlock_from_isr();
}

/** @} */
