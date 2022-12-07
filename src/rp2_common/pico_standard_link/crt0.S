/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "pico.h"
#include "hardware/regs/m0plus.h"
#include "hardware/regs/addressmap.h"
#include "hardware/regs/sio.h"
#include "pico/binary_info/defs.h"

#ifdef NDEBUG
#ifndef COLLAPSE_IRQS
#define COLLAPSE_IRQS
#endif
#endif

.syntax unified
.cpu cortex-m0plus
.thumb

.section .vectors, "ax"
.align 2

.global __vectors, __VECTOR_TABLE
__VECTOR_TABLE:
__vectors:
.word __StackTop
.word _reset_handler
.word isr_nmi
.word isr_hardfault
.word isr_invalid // Reserved, should never fire
.word isr_invalid // Reserved, should never fire
.word isr_invalid // Reserved, should never fire
.word isr_invalid // Reserved, should never fire
.word isr_invalid // Reserved, should never fire
.word isr_invalid // Reserved, should never fire
.word isr_invalid // Reserved, should never fire
.word isr_svcall
.word isr_invalid // Reserved, should never fire
.word isr_invalid // Reserved, should never fire
.word isr_pendsv
.word isr_systick
.word isr_irq0
.word isr_irq1
.word isr_irq2
.word isr_irq3
.word isr_irq4
.word isr_irq5
.word isr_irq6
.word isr_irq7
.word isr_irq8
.word isr_irq9
.word isr_irq10
.word isr_irq11
.word isr_irq12
.word isr_irq13
.word isr_irq14
.word isr_irq15
.word isr_irq16
.word isr_irq17
.word isr_irq18
.word isr_irq19
.word isr_irq20
.word isr_irq21
.word isr_irq22
.word isr_irq23
.word isr_irq24
.word isr_irq25
.word isr_irq26
.word isr_irq27
.word isr_irq28
.word isr_irq29
.word isr_irq30
.word isr_irq31

// all default exception handlers do nothing, and we can check for them being set to our
// default values by seeing if they point to somewhere between __defaults_isrs_start and __default_isrs_end
.global __default_isrs_start
__default_isrs_start:

// Declare a weak symbol for each ISR.
// By default, they will fall through to the undefined IRQ handler below (breakpoint),
// but can be overridden by C functions with correct name.

.macro decl_isr_bkpt name
.weak \name
.type \name,%function
.thumb_func
\name:
    bkpt #0
.endm

// these are separated out for clarity
decl_isr_bkpt isr_invalid
decl_isr_bkpt isr_nmi
decl_isr_bkpt isr_hardfault
decl_isr_bkpt isr_svcall
decl_isr_bkpt isr_pendsv
decl_isr_bkpt isr_systick

.global __default_isrs_end
__default_isrs_end:

.macro decl_isr name
.weak \name
.type \name,%function
.thumb_func
\name:
.endm

decl_isr isr_irq0
decl_isr isr_irq1
decl_isr isr_irq2
decl_isr isr_irq3
decl_isr isr_irq4
decl_isr isr_irq5
decl_isr isr_irq6
decl_isr isr_irq7
decl_isr isr_irq8
decl_isr isr_irq9
decl_isr isr_irq10
decl_isr isr_irq11
decl_isr isr_irq12
decl_isr isr_irq13
decl_isr isr_irq14
decl_isr isr_irq15
decl_isr isr_irq16
decl_isr isr_irq17
decl_isr isr_irq18
decl_isr isr_irq19
decl_isr isr_irq20
decl_isr isr_irq21
decl_isr isr_irq22
decl_isr isr_irq23
decl_isr isr_irq24
decl_isr isr_irq25
decl_isr isr_irq26
decl_isr isr_irq27
decl_isr isr_irq28
decl_isr isr_irq29
decl_isr isr_irq30
decl_isr isr_irq31

// All unhandled USER IRQs fall through to here
.global __unhandled_user_irq
.thumb_func
__unhandled_user_irq:
    bl __get_current_exception
    subs r0, #16
.global unhandled_user_irq_num_in_r0
unhandled_user_irq_num_in_r0:
    bkpt #0

// ----------------------------------------------------------------------------

.section .binary_info_header, "a"

// Header must be in first 256 bytes of main image (i.e. excluding flash boot2).
// For flash builds we put it immediately after vector table; for NO_FLASH the
// vectors are at a +0x100 offset because the bootrom enters RAM images directly
// at their lowest address, so we put the header in the VTOR alignment hole.

#if !PICO_NO_BINARY_INFO
binary_info_header:
.word BINARY_INFO_MARKER_START
.word __binary_info_start
.word __binary_info_end
.word data_cpy_table // we may need to decode pointers that are in RAM at runtime.
.word BINARY_INFO_MARKER_END
#endif

// ----------------------------------------------------------------------------

.section .reset, "ax"

// On flash builds, the vector table comes first in the image (conventional).
// On NO_FLASH builds, the reset handler section comes first, as the entry
// point is at offset 0 (fixed due to bootrom), and VTOR is highly-aligned.
// Image is entered in various ways:
//
// - NO_FLASH builds are entered from beginning by UF2 bootloader
//
// - Flash builds vector through the table into _reset_handler from boot2
//
// - Either type can be entered via _entry_point by the debugger, and flash builds
//   must then be sent back round the boot sequence to properly initialise flash

// ELF entry point:
.type _entry_point,%function
.thumb_func
.global _entry_point
_entry_point:

#if PICO_NO_FLASH
    // Vector through our own table (SP, VTOR will not have been set up at
    // this point). Same path for debugger entry and bootloader entry.
    ldr r0, =__vectors
#else
    // Debugger tried to run code after loading, so SSI is in 03h-only mode.
    // Go back through bootrom + boot2 to properly initialise flash.
    movs r0, #0
#endif
    ldr r1, =(PPB_BASE + M0PLUS_VTOR_OFFSET)
    str r0, [r1]
    ldmia r0!, {r1, r2}
    msr msp, r1
    bx r2

.macro addOffSet reg
#if __PIC__
    // if not, we might need to patch the flash (r1) address with our offset
    ldr r5, =__flash_binary_start
    ldr r6, =__flash_binary_end
    //ldr r7, =__firmwareOffset
    //ldr r7, [r7]
    cmp \reg, r5
    blo no_patch_data\@
    cmp \reg, r6
    bhs no_patch_data\@
    // patch the address
    adds \reg, \reg, r7
no_patch_data\@:
#endif
.endm

// Reset handler:
// - initialises .data
// - clears .bss
// - calls runtime_init
// - calls main
// - calls exit (which should eventually hang the processor via _exit)

.type _reset_handler,%function
.thumb_func
_reset_handler:
#ifdef __PIC__
    // our bootloader passes the offset of our firmware in flash in r10
    mov r7, r10
    ldr r2, =__firmwareOffset
    str r7, [r2]
#endif


    // Only core 0 should run the C runtime startup code; core 1 is normally
    // sleeping in the bootrom at this point but check to be sure
    ldr r0, =(SIO_BASE + SIO_CPUID_OFFSET)
    ldr r0, [r0]
    cmp r0, #0
    bne hold_non_core0_in_bootrom

    // In a NO_FLASH binary, don't perform .data copy, since it's loaded
    // in-place by the SRAM load. Still need to clear .bss
#if !PICO_NO_FLASH
    adr r4, data_cpy_table
    // patch in our offset, does nothing in NON-PIC
    // addOffSet r4
    // assume there is at least one entry
1:
    ldmia r4!, {r1-r3}
    // check if we are at the end of the table
    cmp r1, #0
    beq 2f
    // r1 is the source address, r2 is the destination address, r3 is the end of the area to copy
    // patch in our offset, does nothing in NON-PIC
    addOffSet r1
    bl data_cpy
    b 1b
2:
#endif

   //Patch any of the tables in ram that have pointers to flash functions
#if __PIC__
    PatchRamTables:
    // Load the addresses from the table
    adr r4, patch_flash_addr_table
    // patch in our offset, does nothing in NON-PIC
    // addOffSet r4
3:
    // load the start and end addresses into r1 and r2
    ldmia r4!, {r1-r2}
    // if r1 is 0, then we are finished
    cmp r1, #0
    // exit if r1 = 0
    beq 4f
    // otherwise, patch the flash addresses
    bl patch_flash_setup
    // and goto next entry for the flash table. 
    b 3b
4:
    LoadGlobalOffsetTableRegister:
    ldr r0, =__ram_global_offset_table_begin
    mov r9, r0 // Stupid trick to put global offset table location to r9, for Cortex-M0

#endif

    // Zero out the BSS
    // XXX PIC: Patch with offsets
    ldr r1, =__bss_start__
    ldr r2, =__bss_end__
    movs r0, #0
    b bss_fill_test
bss_fill_loop:
    stm r1!, {r0}
bss_fill_test:
    cmp r1, r2
    bne bss_fill_loop

platform_entry: // symbol for stack traces
    // Use 32-bit jumps, in case these symbols are moved out of branch range
    // (e.g. if main is in SRAM and crt0 in flash)
    // XXX PIC: Patch with Offsets?
    ldr r1, =runtime_init
    // patch in our offset, does nothing in NON-PIC
    addOffSet r1
    blx r1

    ldr r1, =main
    // patch in our offset, does nothing in NON-PIC
    addOffSet r1
    blx r1

    ldr r1, =exit
    // patch in our offset, does nothing in NON-PIC
    addOffSet r1
    blx r1
    // exit should not return.  If it does, hang the core.
    // (fall thru into our hang _exit impl

// ----------------------------------------------------------------------------
// If core 1 somehow gets into crt0 due to a spectacular VTOR mishap, we need to
// catch it and send back to the sleep-and-launch code in the bootrom. Shouldn't
// happen (it should sleep in the ROM until given an entry point via the
// cross-core FIFOs) but it's good to be defensive.

hold_non_core0_in_bootrom:
    ldr r0, = 'W' | ('V' << 8)
    bl rom_func_lookup
    bx r0

.weak _exit
.type _exit,%function
.thumb_func
_exit:
1: // separate label because _exit can be moved out of branch range
    bkpt #0
    b 1b

#if !PICO_NO_FLASH
data_cpy_loop:
    ldm r1!, {r0}
    stm r2!, {r0}
data_cpy:
    cmp r2, r3
    blo data_cpy_loop
    bx lr
#endif

// Note the data copy table is still included for NO_FLASH builds, even though
// we skip the copy, because it is listed in binary info

.align 2
data_cpy_table:
#if PICO_COPY_TO_RAM
.word __ram_text_source__
.word __ram_text_start__
.word __ram_text_end__
#endif
.word __etext
.word __data_start__
.word __data_end__

.word __scratch_x_source__
.word __scratch_x_start__
.word __scratch_x_end__

.word __scratch_y_source__
.word __scratch_y_start__
.word __scratch_y_end__

#if __PIC__
/* copy our vector table to RAM here, as we need to patch it */
.word __vectors
.word ram_vector_table
.word ram_vector_table + (48 * 4)

.word __flash_global_offset_table_begin
.word __ram_global_offset_table_begin
.word __ram_global_offset_table_end
#endif

.word 0 // null terminator

#if __PIC__
patch_flash_loop:
    //load r1 pointer into r0
    ldr r0, [r1]
    // if r0 is within flash range, then patch it
    cmp r0, r5
    blo patch_flash_store
    cmp r0, r6
    bhs patch_flash_store
    // patch the address
    adds r0, r0, r3
patch_flash_store:
    // store r0 back to r1 pointer
    stm r1!, {r0}
    b patch_check_loop
patch_flash_setup:
    // are we at the end of this entry (start = end as r1 is incremented on each store)
    ldr r5, =__flash_binary_start
    ldr r6, =__flash_binary_end
    ldr r3, =__firmwareOffset
    ldr r3, [r3]
patch_check_loop:
    cmp r1, r2
    blo patch_flash_loop
    bx lr
#endif


#if __PIC__
.align 2
patch_flash_addr_table:
/* patch our vector tables */
.word ram_vector_table
.word ram_vector_table + (48 * 4)

.word __mutex_array_start
.word __mutex_array_end

.word __preinit_array_start
.word __preinit_array_end

.word __init_array_start
.word __init_array_end

.word __fini_array_start
.word __fini_array_end

.word __ram_global_offset_table_begin
.word __ram_global_offset_table_end

.word 0 // null terminator
#endif

// ----------------------------------------------------------------------------
// Provide safe defaults for _exit and runtime_init
// Full implementations usually provided by platform.c

.weak runtime_init
.type runtime_init,%function
.thumb_func
runtime_init:
    bx lr


.global __get_current_exception
.thumb_func
__get_current_exception:
    mrs  r0, ipsr
    uxtb r0, r0
    bx   lr

// ----------------------------------------------------------------------------
// Stack/heap dummies to set size

.section .stack
// align to allow for memory protection (although this alignment is pretty much ignored by linker script)
.align 5
    .equ StackSize, PICO_STACK_SIZE
.space StackSize

.section .heap
.align 2
    .equ HeapSize, PICO_HEAP_SIZE
.space HeapSize
