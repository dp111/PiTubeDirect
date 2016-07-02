// This code sits at the FIQ vector
// and is included at the vector


#include "rpi-base.h"
#include "tube-defs.h"

#define DETECT_DUMMY_READ
// #define DEBUG_OVERRUN
// #define DEBUG_LATE
// #define ON_READS_SPIN_FOR_PHI2_HIGH

#define GPFBASE (GPFSEL0)
#define GPFSEL0_offset (GPFSEL0 - GPFBASE )  // controls GPIOs 0..9
#define GPFSEL1_offset (GPFSEL1 - GPFBASE )  // controls GPIOs 10..19
#define GPFSEL2_offset (GPFSEL2 - GPFBASE )  // controls GPIOs 20..29
#define GPSET0_offset  (GPSET0 - GPFBASE)
#define GPCLR0_offset  (GPCLR0 - GPFBASE)
#define GPLEV0_offset  (GPLEV0 - GPFBASE)
#define GPEDS0_offset  (GPEDS0 - GPFBASE)     

.global tube_regs
.global tube_mailbox
.global tube_signal_type

.global gpfsel_data_idle
.global gpfsel_data_driving
.global isr_code_start
.global isr_code_end
.global isr_data_start
.global isr_data_end


.macro SET_TEST_PIN_HIGH
#ifdef HAS_40PINS
        mov     r12, #TEST_MASK
        str     r12, [r13,#GPSET0_offset]
#endif
.endm

.macro SET_TEST_PIN_LOW
 #ifdef HAS_40PINS
        mov     r12, #TEST_MASK
        str     r12, [r13,#GPCLR0_offset]
#endif
.endm

arm_fiq_handler:
isr_code_start:

        //r8-r11 are shadowed in FIQ mode, so no need to push
	// 	R13 ( sp ) is defiend to point to GPFBASE
	//LDR	r13,=GPFBASE
	SET_TEST_PIN_HIGH
	
        mov     r9, #-1
        str     r9, [r13,#GPEDS0_offset]        // clear all Pin events

        ldr     r9, [r13,#GPLEV0_offset]        // *** very expensive but necessary ***

	adr     r8, gpfsel_data_driving		// free cycle get ready incase of a read cycle
	adr     r11, tube_regs			// free cycle	
	ldmia	r8,{r8,r10,r12}			// free cycle
	
        tst     r9, #NRST_MASK          	// test for reset
        beq     post_mail

        tst     r9, #NTUBE_MASK         	// test for glitches
        bne     exit

        tst     r9, #RNW_MASK
        beq     wr_cycle

// READ_CYCLE
        //stmia	r13,{r8,r10,r12}		// Doesn't work
	str     R8, [r13,#GPFSEL0_offset]
	str     r10, [r13, #GPFSEL1_offset]            // *** expensive but necessary ***
	str     r12, [r13, #GPFSEL2_offset]            // *** expensive but necessary ***

	tst     r9, #A0_MASK
	and	r9,r9,#A1_MASK+A2_MASK
        orrne   r11, #1

        ldrb    r9, [r11,R9,A1_SHIFT]

        and     r11, r9, #0x0F 				// get lower nibble
	and 	r9,  r9, #0xF0 				// get upper nible
	mov	r9,  r9,LSL #D4_BASE-4 			// move upper nibble to the correct place
        orr     r9, r9, r11,LSL #D0_BASE 		// move lower nibble to correct place and orr it it
	
        str     r9, [r13,#GPSET0_offset]                // *** expensive but necessary ***
  
 	SET_TEST_PIN_LOW

 	
#ifdef ON_READS_SPIN_FOR_PHI2_HIGH
rd_wait_for_phi2_high1:
        ldr     r10, [r13,#GPLEV0_offset]                // ** very expensive but necessary ***
        tst     r10, #PHI2_MASK
        beq     rd_wait_for_phi2_high1
#endif
	adr	r8,gpfsel_data_idle
	LDMIA	R8,{r8,R11,R12}	

rd_wait_for_phi2_low:
        ldr     r10, [r13,#GPLEV0_offset]                // ** very expensive but necessary ***
        tst     r10, #PHI2_MASK
        movne   r9, r10
        bne     rd_wait_for_phi2_low

	//STMIA	r13,{r8,R11,R12}		// Doesn't work
        str     r8, [r13,#GPFSEL0_offset]              	// *** expensive but necessary ***
        str     r11, [r13, #GPFSEL1_offset]            	// *** expensive but necessary ***
        str     r12, [r13, #GPFSEL2_offset]            	// *** expensive but necessary ***

        mov     r10, #D30_MASK				// Clear databus
        orr     r10, r10, #D74_MASK
        str     r10, [r13,#GPCLR0_offset]               // *** expensive but necessary ***

// In some rare cases, a read may be immediately followed by a write
// A concrete case of this is *FX 151,230,N which uses STA &FEE0,X
// To detect this case, on reads we wait for one more edge of Phi2
// and the re-check the nTUBE and RNW lines for a possible write

#ifdef DETECT_DUMMY_READ        
rd_wait_for_phi2_high2:
        ldr     r10, [r13,#GPLEV0_offset]                // ** very expensive but necessary ***
        tst     r10, #PHI2_MASK
        beq     rd_wait_for_phi2_high2
        tst     r10, #(NTUBE_MASK | RNW_MASK)
        beq     wr_wait_for_phi2_low
#endif

        tst     r9, #A0_MASK             // don't bother emulator with status reads
        bne     post_mail

	SET_TEST_PIN_LOW
        subs    pc, lr, #4

// WRITE_CYCLE
wr_cycle:

wr_wait_for_phi2_high:
        ldr     r9, [r13,#GPLEV0_offset]
        tst     r9, #PHI2_MASK
        beq     wr_wait_for_phi2_high

wr_wait_for_phi2_low:
        ldr     r10, [r13,#GPLEV0_offset]
        tst     r10, #PHI2_MASK
        movne   r9, r10
        bne     wr_wait_for_phi2_low

// At this point, cache misses will no longer disrupt the 6502 bus timings
post_mail:
        ldr     r10, =PINS_MASK
        and     r9, r9, r10

post_mail2:
        orr     r9, r9, #ATTN_MASK

        ldr     r10, tube_mailbox
        tst     r10, #ATTN_MASK         // if previous message not acknowledged, then flag overrun
        orrne   r9, r9, #OVERRUN_MASK
        str     r9, tube_mailbox

// Update TEST2 Pin to reflect overun state      
#if defined(DEBUG_OVERRUN) && defined(HAS_40PINS)
        mov     r10, #TEST2_MASK
        strne     r10, [r13,#GPSET0_offset]
        streq     r10, [r13,#GPCLR0_offset]

#endif

// Update TEST3 Pin to reflect late state      
#if defined(DEBUG_LATE) && defined(HAS_40PINS)
        mov     r10, #TEST3_MASK
        tst     r9, #NTUBE_MASK
        strne     r10, [r13,#GPSET0_offset]
        streq     r10, [r13,#GPCLR0_offset]
#endif
        ldr	r9,tube_signal_type
	MOVS	r9,r9
	BEQ	exit

        // Switch back to irq mode so ip is not shadowed
        mrs     r9, cpsr
        bic     r9, r9, #0x1F
        orr     r9, r9, #0x12
        msr     cpsr_c, r9
        
        orr     ip, ip, #1024   // signal event to 6502 instruction flow

        // Switch back to fiq mode, so we return correctly
        // lr is used here because its shadowed in IRQ mode, so doesn't corrupt the normal lr
        mrs     lr, cpsr
        bic     lr, lr, #0x1F
        orr     lr, lr, #0x11
        msr     cpsr_c, lr

        // Note, the above mode switching could be avoided if we reworked the register assignment
        // in the 6502 emulator to use a register r0..r7 for signalling.
        // I did do this as an experiment, and it's parked in a branch.
        // I'd like to avoid making big changes to that code for now.

exit:     

        SET_TEST_PIN_LOW
        //r8-r11 are shadowed in FIQ mode, so no need to pop
        //pop   {r8-r11}
        subs    pc, lr, #4

glitch:
        ldr     r10, =PINS_MASK
        and     r9, r9, r10
        orr     r9, r9, #GLITCH_MASK
        b       post_mail2

CACHELINE_ALIGN = 5
.align CACHELINE_ALIGN
isr_code_end:

// =================================================
// ISR DATA
// =================================================

.align CACHELINE_ALIGN
isr_data_start:

// 8 BYTES OF STATE  Muts be align to an 8 byte boundary 
tube_regs:
        .byte 0xfe
        .byte 0xed
        .byte 0xdc
        .byte 0xcb
        .byte 0xba
        .byte 0xa9
        .byte 0x98
        .byte 0x87
// Insert the current literal pool, so these get locked in the cache
.ltorg

// Some constants for controlling the data bus        
gpfsel_data_idle:
        .word 0
        .word 0
        .word 0

gpfsel_data_driving:
        .word 0
        .word 0
        .word 0

// Mailbox between tube isr and events handler (in C)
tube_mailbox:
        .word 0

// Selects which type of signalling used
tube_signal_type:
	.word 0	


.align CACHELINE_ALIGN
isr_data_end:
