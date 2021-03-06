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

    Revised:  1/26/19 brent@mbari.org -- limited heap size
*/

/*
 * ST32L1152xB-A memory setup.
 */
__main_stack_size__     = 0x0400;
__process_stack_size__  = 0x0400;

/* Override __heap_size__ to reserve desired heap size */
PROVIDE(__heap_size__ = 0);

/* Bootloader links with appSpace==sizeof(largest app) and appOffset undefined
   Applications link with appOffset == 0x100 and appSpace undefined
   Stand alone images link with both appOffset and appSpace undefined */
PROVIDE(__appOffset__ = 0);
PROVIDE(__appSpace__ = 0);

/* Application area follows bootloader's Cortex M3 vectors
   Reserve 1st flash page for bootloader's vectors and last for its directory */

__flash_start__ = 0x08000000;
__flash_size__  = 128k;
__flash_end__   = __flash_start__ + __flash_size__;

__vectors_start__ = __flash_start__ + __appOffset__;
__vectors_size__  = 256;
__vectors_end__   = __vectors_start__ + __vectors_size__;

__app_start__   = __vectors_end__ + __appSpace__;
__app_end__     = __flash_end__ - 256;    /*reserve last flash page for loader*/
__app_size__    = __app_end__ - __app_start__;

MEMORY
{
    vectors (R): org = __flash_start__ + __appOffset__, len = __vectors_size__
    flash ( RX): org = __app_start__, len = __app_size__
    ram   (RWX): org = 0x20000000, len = 32k
}

__ram_start__           = ORIGIN(ram);
__ram_size__            = LENGTH(ram);
__ram_end__             = __ram_start__ + __ram_size__;


ENTRY(ResetHandler)

SECTIONS
{
    vectors : ALIGN(256)
    {
        KEEP(*(vectors))
        . = __vectors_size__ - 12;
        KEEP(*(textSegment))
    } > vectors

    constructors : ALIGN(4)
    {
        _text = .;
        PROVIDE(__init_array_start = .);
        KEEP(*(SORT(.init_array.*)))
        KEEP(*(.init_array))
        PROVIDE(__init_array_end = .);
    } > flash

    destructors : ALIGN(4)
    {
        PROVIDE(__fini_array_start = .);
        KEEP(*(.fini_array))
        KEEP(*(SORT(.fini_array.*)))
        PROVIDE(__fini_array_end = .);
    } > flash

    .text : ALIGN(16)
    {
        *(.text.startup.*)
        *(.text)
        *(.text.*)
        *(.rodata)
        *(.rodata.*)
        *(.glue_7t)
        *(.glue_7)
        *(.gcc*)
    } > flash

    .ARM.extab :
    {
        *(.ARM.extab* .gnu.linkonce.armextab.*)
    } > flash

    .ARM.exidx : {
        PROVIDE(__exidx_start = .);
        *(.ARM.exidx* .gnu.linkonce.armexidx.*)
        PROVIDE(__exidx_end = .);
     } > flash

    .eh_frame_hdr :
    {
        *(.eh_frame_hdr)
    } > flash

    .eh_frame : ONLY_IF_RO
    {
        *(.eh_frame)
    } > flash

/***  Enabling this can result in a 4 byte hole between .text and .data ***
    .textalign : ONLY_IF_RO
    {
        . = ALIGN(8);
    } > flash
*/

    . = ALIGN(4);
    _textdata = .;
    _etext = . + SIZEOF(.data);  /* past textdata */

    .stacks :
    {
        . = ALIGN(8);
        __main_stack_base__ = .;
        . += __main_stack_size__;
        . = ALIGN(8);
        __main_stack_end__ = .;
        __process_stack_base__ = .;
        __main_thread_stack_base__ = .;
        . += __process_stack_size__;
        . = ALIGN(8);
        __process_stack_end__ = .;
        __main_thread_stack_end__ = .;
    } > ram

    .data ALIGN(4) : ALIGN(4)
    {
        . = ALIGN(4);
        PROVIDE(_data = .);
        *(.data)
        *(.data.*)
        *(.ramtext)
        . = ALIGN(4);
        PROVIDE(_edata = .);
    } > ram AT > flash

    .bss ALIGN(4) : ALIGN(4)
    {
        . = ALIGN(4);
        PROVIDE(_bss_start = .);
        *(.bss)
        *(.bss.*)
        *(COMMON)
        . = ALIGN(4);
        PROVIDE(_bss_end = .);
        __heap_base__ = .;
        . += __heap_size__;
        __heap_end__ = .;
        PROVIDE(_eram = .);
    } > ram
}

PROVIDE(end = .);
_end            = .;
