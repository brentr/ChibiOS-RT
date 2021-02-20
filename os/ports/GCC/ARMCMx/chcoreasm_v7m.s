/*
    ChibiOS/RT - Copyright (C) 2006,2007,2008,2009,2010,
                 2011,2012,2013 Giovanni Di Sirio.
                 revised 2/19/2021 by brent@mbari.org
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
.text
.syntax unified
.thumb
/*
 * Imports the Cortex-Mx configuration headers.
 */
#define _FROM_ASM_
#include "chconf.h"
#include "chcore.h"
#define CONTEXT_OFFSET  12
#ifndef CORTEX_USE_FPU
#error  assembler needs to know CORTEX_USE_FPU
#endif
#ifndef CH_DBG_SYSTEM_STATE_CHECK
#error  assembler needs to know CH_DBG_SYSTEM_STATE_CHECK
#endif
#ifndef CORTEX_SIMPLIFIED_PRIORITY
#error  assembler needs to know CORTEX_SIMPLIFIED_PRIORITY
#endif
#ifndef CRT0_INIT_STACKS
#error assembler needs to know CRT0_INIT_STACKS
#endif
/*
 * Control special register initialization value.
 * The system is setup to run in privileged mode using the PSP
 * stack (dual stack mode).
 */
#if !defined(CRT0_CONTROL_INIT)
#define CRT0_CONTROL_INIT           0x00000002
#endif

                .set    SCB_ICSR, 0xE000ED04
                .set    ICSR_PENDSVSET, 0x10000000

                .syntax unified
                .cpu    cortex-m4
#if CORTEX_USE_FPU
                .fpu    fpv4-sp-d16
#else
                .fpu    softvfp
#endif

                .thumb
                .text

/*--------------------------------------------------------------------------*
 * Performs a context switch between two threads.
 *--------------------------------------------------------------------------*/
                .thumb_func
                .globl  _port_switch
_port_switch:
                push    {r4, r5, r6, r7, r8, r9, r10, r11, lr}
#if CORTEX_USE_FPU
                vpush   {s16-s31}
#endif

                str     sp, [r1, #CONTEXT_OFFSET]
#if (CORTEX_SIMPLIFIED_PRIORITY == FALSE) &&                                \
    ((CORTEX_MODEL == 3) || (CORTEX_MODEL == 4))
                /* Workaround for ARM errata 752419, only applied if
                   condition exists for it to be triggered.*/
                ldr     r3, [r0, #CONTEXT_OFFSET]
                mov     sp, r3
#else
                ldr     sp, [r0, #CONTEXT_OFFSET]
#endif

#if CORTEX_USE_FPU
                vpop    {s16-s31}
#endif
                pop     {r4, r5, r6, r7, r8, r9, r10, r11, pc}

/*--------------------------------------------------------------------------*
 * Start a thread by invoking its work function.
 *
 * Threads execution starts here, the code leaves the system critical zone
 * and then jumps into the thread function passed in register R4. The
 * register R5 contains the thread parameter. The function chThdExit() is
 * called on thread function return.
 *--------------------------------------------------------------------------*/
                .thumb_func
                .globl  _port_thread_start
_port_thread_start:
#if CH_DBG_SYSTEM_STATE_CHECK
                bl      dbg_check_unlock
#endif
#if CORTEX_SIMPLIFIED_PRIORITY
                cpsie   i
#else
                movs    r3, #CORTEX_BASEPRI_DISABLED
                msr     BASEPRI, r3
#endif
                mov     r0, r5
                blx     r4
                bl      chThdExit
_zombies:       b       _zombies

/*--------------------------------------------------------------------------*
 * Post-IRQ switch code.
 *
 * Exception handlers return here for context switching.
 *--------------------------------------------------------------------------*/
                .thumb_func
                .globl  _port_switch_from_isr
_port_switch_from_isr:
#if CH_DBG_SYSTEM_STATE_CHECK
                bl      dbg_check_lock
#endif
                bl      chSchDoReschedule
#if CH_DBG_SYSTEM_STATE_CHECK
                bl      dbg_check_unlock
#endif
#if CH_DBG_STATISTICS
                bl      _stats_stop_measure_crit_thd
#endif
                .globl  _port_exit_from_isr
_port_exit_from_isr:
#if CORTEX_SIMPLIFIED_PRIORITY
                movw    r3, #:lower16:SCB_ICSR
                movt    r3, #:upper16:SCB_ICSR
                mov     r2, ICSR_PENDSVSET
                str     r2, [r3, #0]
                cpsie   i
#else /* !CORTEX_SIMPLIFIED_PRIORITY */
                svc     #0
#endif /* !CORTEX_SIMPLIFIED_PRIORITY */
.L1:            b       .L1


/*
 * Reset handler
 */
.global ResetHandler
.type ResetHandler, %function
.thumb_func
ResetHandler:
         /* The processor enters Thread mode when it comes out of reset.
            Out of reset, all code uses the main stack.*/
         cpsid   i
         /* Process Stack initialization, it is allocated starting from the
            symbol __process_stack_end__ and its lower limit is the symbol
            __process_stack_base__.*/
         movw    r0, #:lower16:__process_stack_end__
         movt    r0, #:upper16:__process_stack_end__
         msr     PSP, r0
         mov     r1, #CRT0_CONTROL_INIT
         msr     CONTROL, r1
         isb
         /* Running on process stack.*/
#if CRT0_INIT_STACKS
         bl _init_process_stack
#endif
         b _start

