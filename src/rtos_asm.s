; ///////////////////////////////////////////////////////////////////////////////////////
; This file contains the interrupt service routine for the INT0 pin change interrupt,
; which is used to do a task switch.
;

;  MIT License
;  
;  Copyright (c) 2020  Clinton J.S. Reddekop
;  
;  Permission is hereby granted, free of charge, to any person obtaining a copy
;  of this software and associated documentation files (the "Software"), to deal
;  in the Software without restriction, including without limitation the rights
;  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
;  copies of the Software, and to permit persons to whom the Software is
;  furnished to do so, subject to the following conditions:
;  
;  The above copyright notice and this permission notice shall be included in all
;  copies or substantial portions of the Software.
;  
;  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
;  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
;  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
;  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
;  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
;  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
;  SOFTWARE.
;

; Contains vector define for INT0_vect, as well as I/O register defines:
#include <avr/io.h>

; Error checks
#if !defined(__AVR_ATmega328P__)
    #warning rtos was written for an ATmega328P, and might not work for your part!
#endif


; While a task is not the running task, the processor state for that task is kept on
; its stack in the following order:
;
;  (higher addresses)
;   ...
;   return address (program counter) LSByte
;   return address (program counter) MSByte
;   saved r31
;   saved SREG
;   saved r30
;   saved r29
;   ...
;   saved r1
;   saved r0
;   (next unused stack location - SP points here)
;   ...
;  (lower addresses)
;
; The stack pointer is saved in pStacks[prio], where prio is the priority of the task
;


; Comments indicating cycle counts are for the atmega328p


; This ISR is used to do a task switch

.global INT0_vect
    .type INT0_vect, @function
INT0_vect:
    
    ; return address (program counter) was automatically pushed on the stack when this
    ; ISR started
    ; global interrupt flag in SREG was automatically cleared when this ISR started
    
    ; save r31 on the old stack
    push r31
    
    ; save SREG on the old stack - we do this before enabling interrupts so that the
    ; global interrupt flag is 0 in the saved SREG
    in r31, _SFR_IO_ADDR(SREG)
    push r31
    
    ; disable INT0 (this interrupt), then set global interrupt flag so other interrupts
    ; can occur
    cbi _SFR_IO_ADDR(EIMSK), INT0
    sei
    
    ; save the remaining cpu registers on the old stack
    push r30
    push r29
    push r28
    push r27
    push r26
    push r25
    push r24
    push r23
    push r22
    push r21
    push r20
    push r19
    push r18
    push r17
    push r16
    push r15
    push r14
    push r13
    push r12
    push r11
    push r10
    push r9
    push r8
    push r7
    push r6
    push r5
    push r4
    push r3
    push r2
    push r1
    push r0
    
    ; 70 cycles to here
    
    ; use r20 as zero reg
    eor r20, r20
    
    ; save the old stack pointer for the old task in pStacks[curPrio]
    ; (curPrio is the priority of the old task)
    lds r24, curPrio
    add r24, r24            ; double because each pStacks entry is 2 bytes
    ldi r30, lo8(pStacks)
    ldi r31, hi8(pStacks)
    add r30, r24
    adc r31, r20
    in r24, _SFR_IO_ADDR(SPL)
    in r25, _SFR_IO_ADDR(SPH)
    std Z+0, r24
    std Z+1, r25
    
    ; 84 cycles to here
    
waitForTaskReady:
; We jump here from rtosTaskSwitchFirstTime, or fall through from above
; When we get here INT0 MUST be disabled in EIMSK register
; When we get here r20 MUST contain 0
    
    ; wait until some task is ready to run
    
    ; r22 is used in loop below to enable sleep in idle mode
    ; We use idle mode to keep clki/o running, since it feeds the timer that
    ; generates tick interrupts
    ldi r22, (1 << SE)

loopWaitForTaskReady:
    
    ; disable interrupts before reading 'ready' - why is explained after this loop
    cli
    lds r24, ready
    tst r24
    ; brne takes 2 cycles if branch is taken, else 1 cycle
    brne someTaskIsReady
    
    ; No task is ready; enable interrupts and go to sleep.  This allows interrupts to
    ; make tasks ready.
    ; NOTE sleep mode control register SMCR is changed here - so if other (non-rtos)
    ; code in the system wants to use sleep, that code should always disable interrupts,
    ; set SMCR and do the sleep instruction immediately after reenabling interrupts
    out _SFR_IO_ADDR(SMCR), r22
    sei
    sleep
    ; disable sleep
    out _SFR_IO_ADDR(SMCR), r20
    
    rjmp loopWaitForTaskReady
    
someTaskIsReady:
    
    ; BEST CASE 91 cycles to here
    
    ; (INT0 is still disabled in EIMSK register, preventing re-entry into this ISR.)
    ; If another ISR has run since this ISR began, it may have triggered INT0 to cause
    ; a task switch.  Since the global interrupt flag has been clear since just before
    ; we read 'ready' in the loop above, the value of 'ready' in r24 already reflects
    ; the correct task to switch to.  Clear any new INT0 trigger since we are already
    ; doing the correct task switch, and we do not want INT0_vect (this ISR) to run
    ; again as soon as interrupts are enabled at the end of this ISR.
    ; Write 1 to clear the flag
    sbi _SFR_IO_ADDR(EIFR), INTF0
    
    ; determine priority of the highest-priority task that is ready to run
    ; put it in r22
    ; 'ready' is still in r24
    ldi r30, lo8(prioFromBitmap)
    ldi r31, hi8(prioFromBitmap)
    add r30, r24
    adc r31, r20
    lpm r22, Z
    
    ; 100 cycles to here

    ; make that the new running task
    ldi r30, lo8(bitId)
    ldi r31, hi8(bitId)
    add r30, r22
    adc r31, r20
    ld r24, Z
    sts running, r24
    sts curPrio, r22
    
    ; Now that 'running' and 'curPrio' have been updated, set the global interrupt flag
    ; again so other interrupts can occur.  Any such other interrupt might determine that
    ; a new task switch needs to occur, and it needs to know the current running priority
    ; to correctly determine that, which is why we waited until now to re-enable
    ; interrupts.  If such new task switch needs to occur, INT0 will be triggered and the
    ; INT0 flag will be set.  That is exactly what we want since the INT0 flag will
    ; cause the new task switch (i.e. re-trigger this interrupt) as soon as this ISR ends
    sei
    
    ; 111 cycles to here
    
    ; switch to the stack of the new running task
    add r22, r22            ; double because each pStacks entry is 2 bytes
    ldi r30, lo8(pStacks)
    ldi r31, hi8(pStacks)
    add r30, r22
    adc r31, r20
    ldd r24, Z+0
    ldd r25, Z+1
    ; Disable interrupts while changing SP - since interrupts use the stack, we need
    ; SP to be a valid stack location whenever an interrupt happens
    cli
    out _SFR_IO_ADDR(SPL), r24
    out _SFR_IO_ADDR(SPH), r25
    sei
    
    ; 124 cycles to here
    
    ; restore the cpu registers from the new stack - except r31
    pop r0
    pop r1
    pop r2
    pop r3
    pop r4
    pop r5
    pop r6
    pop r7
    pop r8
    pop r9
    pop r10
    pop r11
    pop r12
    pop r13
    pop r14
    pop r15
    pop r16
    pop r17
    pop r18
    pop r19
    pop r20
    pop r21
    pop r22
    pop r23
    pop r24
    pop r25
    pop r26
    pop r27
    pop r28
    pop r29
    pop r30

    ; restore SREG from the new stack
    ; this will not enable interrupts because the global interrupt flag was 0 when SREG
    ; was saved on the stack
    pop r31
    out _SFR_IO_ADDR(SREG), r31
    
    ; 189 cycles to here
    
    ; restore r31 from the new stack
    pop r31
    
    ; Clear the global interrupt flag, then reenable INT0 (this interrupt.)  If another
    ; interrupt has re-triggered INT0, then the INT0 interrupt will occur again right
    ; after the reti instruction below which reenables interrupts.
    cli
    sbi _SFR_IO_ADDR(EIMSK), INT0
  
    ; 194 cycles to here
    
    ; reti will set the global interrupt enable
    ; returns to whatever address is on the top of the new stack
    reti
    ; BEST CASE 198 cycles total

    
; We jump here only when the scheduler first starts, to start a task for the first time
; When we get here INT0 MUST be disabled in EIMSK register
.global rtosTaskSwitchFirstTime
    .type rtosTaskSwitchFirstTime, @function
rtosTaskSwitchFirstTime:

    ; use r20 as zero reg
    eor r20, r20
    rjmp waitForTaskReady
