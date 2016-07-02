
#include "rpi-base.h"
#include "tube-defs.h"
#include "cache.h"
   
.global arm_irq_handler      
.global lock_isr

// =================================================
// LOCK the ISR CODE into cache
// =================================================
CACHELINE_ALIGN = 5
        
lock_isr:
    push    {lr}

    // Copy locking code into un-cached memory
    ldr     r0, =lock_isr_into_cache_start
    ldr     r1, =UNCACHED_MEM_BASE
    mov     r2, #(lock_isr_into_cache_end - lock_isr_into_cache_start)
copy_loop:
    ldr     r3, [r0], #4
    str     r3, [r1], #4
    subs    r2, r2, #4
    bne     copy_loop

    // disable interrupts (so handler doesn't get accidentally executed
    bl      _disable_interrupts

    // execute locking code
    ldr     r0, =UNCACHED_MEM_BASE
    blx     r0

    // enable interrupts
    bl      _enable_interrupts

    pop     {lr}
    mov     pc, lr

// This code must only be executed from uncachable memory
// and must be position independant

// Cache ways are 4KB, and contain 128 x 32 byte lines
        
// I Cache Locked Way
        
// 0x00000000-0x0000001f - cache line  0     - (vectors)
// 0x01f01180-0x01f012ff - cache lines 12-23 - (ISR code)
        
// D Cache Locked Way

// 0x?????000-0x?????01f - cache line  0       - (vector TLBs)
// 0x00000020-0x0000003f - cache line  1       - (vectors)
// 0x?????060-0x?????07f - cache line  3       - (code/data/stack TLBs)
// 0x01f01300-0x01f01380 - cache lines 24-27   - (ISR data)
// 0x?????800-0x?????81f - cache line  64      - (IO TLB)
// 0x01edff00-0x01edffff - cache lines 120-127 - (FIQ stack, not currently used)

.align CACHELINE_ALIGN
lock_isr_into_cache_start:

    // invalidate entire instruction cache (page 3-74)
    mov     r0, #0
    mcr     p15, 0, r0, c7, c5, 0

    // clean and invalidate the entire data cache (page 3-74)
    mov      r0, #0
    mcr      p15, 0, r0, c7, c14, 0

    // data synchronization barrier to ensure all outstanding memory transactions are complete
    mov     r0, #0
    mcr     p15, 0, r0, c7, c10, 4
        
    // Set the fast interrupt mode bit
    // This is commented out because
    // 1. It costs in overall performance
    // 2. It did not make an observable different to tube read latency

    // mrc     p15, 0, r0, c1, c0, 0
    // orr     r0, r0, #(1<<21)
    // mcr     p15, 0, r0, c1, c0, 0

    // data synchronization barrier to ensure all outstanding memory transactions are complete
    mov     r0, #0
    mcr     p15, 0, r0, c7, c10, 4
        
    // enable d-cache way 0, lock ways 1, 2, 3 (page 3-89)
    mov     r0, #0xfffffffe
    mcr     p15, 0, r0, c9, c0, 0
        
    // enable i-cache way 0, lock ways 1, 2, 3 (page 3-89)
    mov     r0, #0xfffffffe
    mcr     p15, 0, r0, c9, c0, 1

    // bring the isr code into i-cache
    // these loads are relative references, so should relocate
    ldr     r0, isr_code_start_ptr
    ldr     r1, isr_code_end_ptr
lock_code_loop:
    // prefetch cache line into instruction cache (page 3-76)
    // r0 is the virtual address, with bits 4..0 set to zero
    mcr     p15, 0, r0, c7, c13, 1
    // move to the next cache line
    add     r0, r0, #32
    cmp     r0, r1
    bne     lock_code_loop

    // prefetch the vector instructions (8 words starting at 0x00000000) into the i-cache
    mov     r0, #0x00
    mcr     p15, 0, r0, c7, c13, 1

    // any data prefetched is actually used by accumulating into r3
    // don't think this is actually necessary, but lets be really
    // sure the ARM doesn't abort the transactions
    mov     r3, #0x00
        
    // bring the isr data into d-cache
    // these loads are relative references, so should relocate
    ldr     r0, isr_data_start_ptr
    ldr     r1, isr_data_end_ptr
lock_data_loop:
    // load from the cache line to prefetch it
    ldr     r2, [r0]
    add     r3, r3, r2
    // move to the next cache line
    add     r0, r0, #32
    cmp     r0, r1
    bne     lock_data_loop

    // prefetch the vector data (8 words starting at 0x00000020) into the d-cache
    mov     r0, #0x20
    ldr     r2, [r0]
    add     r3, r3, r2

    // prefetch the used page table (TLB) entries
    // The complete table is 4K words (16K bytes) with one work per 1MB of virtual memory
    // It is aligned on a 4KB boundary
    // Each cache line contains 8 words representing 8MB of memory
    ldr     r0, page_table_ptr
    ldr     r2, [r0, #(0x000 * 4)]  // 0x00000000-0x000fffff (vectors)
    add     r3, r3, r2
    ldr     r2, [r0, #(0x018 * 4)]  // 0x01800000-0x01ffffff (stacks, code and data)
    add     r3, r3, r2
    ldr     r2, [r0, #(0x200 * 4)]  // 0x20000000-0x027fffff (IO - todo should use PERIPHERAL_BASE here)
    add     r3, r3, r2

    // bring the isr stack into d-cache
    // these loads are relative references, so should relocate
    ldr     r0, isr_stack_start_ptr
    ldr     r1, isr_stack_end_ptr
lock_stack_loop:
    // load from the cache line to prefetch it
    ldr     r2, [r0]
    add     r3, r3, r2
    // move to the next cache line
    add     r0, r0, #32
    cmp     r0, r1
    bne     lock_stack_loop

    // data synchronization barrier to ensure all outstanding memory transactions are complete
    mov     r0, #0
    mcr     p15, 0, r0, c7, c10, 4
        
    // lock d-cache way 0, enable ways 1, 2, 3 (page 3-89)
    mov     r0, #0xfffffff1
    mcr     p15, 0, r0, c9, c0, 0

    // lock i-cache way 0, enable ways 1, 2, 3 (page 3-89)
    mov     r0, #0xfffffff1
    mcr     p15, 0, r0, c9, c0, 1

    mov     pc, lr

isr_code_start_ptr:
.word isr_code_start

isr_code_end_ptr:
.word isr_code_end

isr_data_start_ptr:
.word isr_data_start

isr_data_end_ptr:
.word isr_data_end

page_table_ptr:
.word PageTable
        
// TODO these should be somehow imported from arm-startup.S
        
isr_stack_start_ptr:
.word 0x01EE0000 - 0x100

isr_stack_end_ptr:
.word 0x01EE0000


.align CACHELINE_ALIGN
lock_isr_into_cache_end:

// =================================================
// ISR CODE
// =================================================


// Default handlers for FIQ/IRQ do nothing


arm_irq_handler:
        subs    pc, lr, #4


