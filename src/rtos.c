/////////////////////////////////////////////////////////////////////////////////////////
// A simple RTOS for the AVR ATmega328P
// It could be made to work with many other 8-bit AVRs
//

//  MIT License
//  
//  Copyright (c) 2020  Clinton J.S. Reddekop
//  
//  Permission is hereby granted, free of charge, to any person obtaining a copy
//  of this software and associated documentation files (the "Software"), to deal
//  in the Software without restriction, including without limitation the rights
//  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
//  copies of the Software, and to permit persons to whom the Software is
//  furnished to do so, subject to the following conditions:
//  
//  The above copyright notice and this permission notice shall be included in all
//  copies or substantial portions of the Software.
//  
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
//  SOFTWARE.

// References:
//  avr-gcc ABI details at https://gcc.gnu.org/wiki/avr-gcc
//  ATmega48A/PA/88A/PA/168A/PA/328/P DATASHEET
//      Rev:Atmel-8271J-AVR-ATmega48A/48PA/88A/88PA/168A/168PA/328/328P-Datasheet_11/2015
//  Microchip AVR Instruction Set Manual
//      Rev.0856F - 05/2008
//  https://gcc.gnu.org/onlinedocs/gcc-5.4.0/gcc/Extended-Asm.html

// Written for avr-gcc version 5.4.0 and avr-libc version 2.0.0

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>

#include <inttypes.h>

#include "rtos.h"
#include "timer0.h"


// Error checks
#if !defined(__AVR_ATmega328P__)
    #warning rtos was written for an ATmega328P, and might not work for your part!
#endif

// Ticks since rtos start
static uint32_t tickCounter = 0;

// Saved stack pointers for task functions, indexed by task priority
uint8_t *pStacks[8];

// Bitmaps are used to represent (sets of) tasks.  The bit representing the task at
// priority p is set in bitId[p] as: bitId[p] = 1 << p
// If there is no task registered at priority p, then bitId[p] == 0
uint8_t bitId[8];
uint8_t ready;      // Bitmap indicating which tasks are ready to run
uint8_t running;    // Bitmap with one of the bitId[]s, indicating the running task
uint8_t curPrio;    // Priority of running task 0..7
static uint8_t timeoutIds; // Bitmap; which blocked tasks have registered a timeout

// LUT to quickly get the priority corresponding to the most-significant 1 bit in a
// bitmap -- i.e. the bit position of the most-significant 1 bit
// NOTE the PROGMEM tells the compiler to put it in program memory.  To read it use
//  pgm_read_byte() -- in C code
//  lpm             -- in assembly
// See avr-libc documentation
const uint8_t prioFromBitmap[256] PROGMEM =
{
    0, // unused
     0,
      1, 1,
       2, 2, 2, 2,
        3, 3, 3, 3, 3, 3, 3, 3,
         4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4,
          5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 
          5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 
           6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6,
           6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 
           6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6,
           6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 
            7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 
            7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 
            7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 
            7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 
            7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 
            7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 
            7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 
            7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 
};

// A "blocking slot" (one per task) is used when a task is blocked, to indicate what the
// task is waiting on, and to keep any associated timeout
typedef struct
{
    uint32_t wakeTime;  // Tick count @ which timeout occurs (if there is a timeout)
    uint8_t timedOut;   // When the task gets unblocked, timedOut!=0 tells the task
                        // that the action failed due to timeout. (Else the action was
                        // successful)
    union
    {
        struct
        {
            // For queues
            // If blocked waiting to send to queue, this is msg we are waiting to send
            // If blocked waiting to receive from queue, this is where the received
            // message will be put for the task to retrieve it
            void *pMessage;
        } q;
        
        struct
        {
            // For event flags
            uint8_t flags;  // which flags we're waiting on
            uint8_t all;    // !0: waiting on all flags  /  0: waiting on any flag
        } f;
        
    } item;
    
    // When this task is blocked, at least one of pWaitingOn (below) or the task's bit in
    // timeoutIds will be nonzero; otherwise both will be zero.
    //  If we're in a delay and not waiting on any action, only the bit in timeoutIds
    //   will be nonzero
    //  If we're waiting for an action with no timeout, only pWaitingOn will be nonzero
    //  If waiting for an action with a timeout, both will be nonzero
    // When this task gets unblocked, either by timeout expiry or because the action is
    // completed, the task or ISR unblocking this task must clear (set to zero) all of:
    //  - This task's bit in *pWaitingOn, if pWaitingOn is nonzero
    //  - pWaitingOn
    //  - This task's bit in timeoutIds
    // BEFORE making this task ready again
    uint8_t *pWaitingOn;// &of bitmap for action/item we're blocked on; it has our bitId
    
} BLOCKING_SLOT;

static BLOCKING_SLOT blockingSlots[8];

extern void rtosTaskSwitchFirstTime(void); // in rtos_asm.s


// ---- Macros ----

// Macros for setting, clearing global interrupt enable
// They are also memory barriers
// (I'm using these rather than the cli() and sei() macros in avr-libc because those in
//  avr-libc used to not have the "memory" clobber, and I'm worried they might change
//  them back)
#define rtos_CLI_MB()   __asm__ __volatile__    ( "cli" : : : "memory", "cc" )
#define rtos_SEI_MB()   __asm__ __volatile__    ( "sei" : : : "memory", "cc" )

// A compiler memory barrier
#define rtos_MB()       __asm__ __volatile__    ( "" : : : "memory", "cc" )

// Macro to declare a uint8_t variable and save the status register in it, then clear
// the global interrupt enable bit.  The parameter is the name to use for the variable.
// Protected before and after by memory barriers
#define rtos_SAVE_SREG_AND_CLI_MB(_SAVE_SREG_VAR_NAME_) \
    rtos_MB();                                          \
    uint8_t _SAVE_SREG_VAR_NAME_ = SREG;                \
    rtos_CLI_MB();

// Macro to restore the status register from the given variable, which should be a
// uint8_t that is in scope
// Protected before and after by memory barriers
#define rtos_RESTORE_SREG_MB(_SAVED_SREG_VAR_NAME_) \
    rtos_MB();                                      \
    SREG = _SAVED_SREG_VAR_NAME_;                   \
    rtos_MB();

// Macro that triggers the task switch interrupt
// It is also a memory barrier
// Toggles PD2 to trigger INT0 interrupt.  The ISR for that interrupt is what does
// a task switch.
//
// The global interrupt enable flag in the status register must be zero when this macro
// is reached.
//
// There must be a uint8_t variable in scope which has been loaded with the value to be
// restored to the SREG by this macro before the interrupt is triggered.  This variable's
// name should be given for the _SAVE_SREG_VAR_NAME_ parameter.
//
// The number of nops was verified by experiment, and ensures that the INT0 interrupt
// flag gets set before the end of the macro.  If the write to the status register from
// _SAVE_SREG_VAR_NAME_ sets the global interrupt flag, then this also means the INT0
// interrupt will be triggered before the end of the macro; otherwise it will be
// triggered whenever the global interrupt flag is next set.
//
//  PIND = (1<<PIND2) in assembly: 0x09 is PIND address, 0x04 is pin 2
#define rtos_TASK_SWITCH_TRIGGER_MB(_SAVED_SREG_VAR_NAME_) \
    __asm__ __volatile__(               \
        "ldi r24, 0x04\n\t"             \
        "out 0x09, r24\n\t"             \
        "out __SREG__, %0\n\t"          \
        "nop\n\t"                       \
        "nop\n\t"                       \
        "nop\n\t"                       \
        :                               \
        : "r" (_SAVED_SREG_VAR_NAME_)   \
        : "r24", "memory", "cc");


// ---- Functions ----

// Registers a task function with the rtos
// This function MUST be called with INT0 and tick interrupt DISABLED
// This function MUST be called before rtosStart()
// There can be at most one task running at each priority
// The given function 'task' is the task to be run at 'priority'
// 'priority' must be 0..7, where higher number = higher priority
// 'pStack' references the stack space to be used by the task; it points to the
// highest-address byte in the space (remember the stack grows toward lower addresses)
void rtosTaskRegister(void (*task)(void), int priority, uint8_t *pStack)
{
    // Error checks
    if ((priority < 0) || (priority > 7) || (task == 0) || (pStack == 0))
    {
        return;
    }
    
    // Nonzero bitId indicates there is a task at this priority
    bitId[priority] = 1 << priority;
    
    // Put values on the stack that will allow the processor state needed by this
    // task to be "restored" when this task is switched to
    
    // 2-byte program counter
    pStack -= 35;
    pStack[35] = (uint16_t)task >> 0 & 0x0ff;
    pStack[34] = (uint16_t)task >> 8 & 0x0ff;
    
    // pStack[33] is saved r31
    //
    // pStack[32] is saved SREG
    // NOTE that we don't want the global interrupt enable bit set here, so the task
    // switcher can "restore" SREG to this value without interrupts getting enabled
    // before the task switcher is ready for that to happen.
    // The other bits of SREG don't matter here.
    //
    // pStack[31] to pStack[1] are saved r30 ... saved r0
    // The only register whose value matters is __zero_reg__, which the compiler
    // expects to hold 0.  __zero_reg__ is r1, but we'll set them all to zero here
    // in case that changes someday.
    for (int i = 1; i <= 33; i += 1)
    {
        pStack[i] = 0;
    }
    // The stack pointer points to the first unused entry on the stack, so the value
    // of pStack[0] doesn't matter
    
    pStacks[priority] = pStack;
    
    // Mark it ready to run
    ready |= bitId[priority];
}

// Initialises a queue
// This function MUST be called on a given queue before any other rtosQueue*() function
// is called on the queue.  The easiest and recommended way to ensure this is by calling
// this function on the queue before rtosStart() is called.
// 'pQ' is the address of the queue to initialise
// 'pBuffer' must point to enough memory to hold 'length' pointers to void
// 'length' may be 0
// The queue starts out empty
void rtosQueueInit(RTOS_QUEUE *pQ, void **pBuffer, uint8_t length)
{
    pQ->pFirstFull = 0;     // pFirstFull == NULL indicates queue is empty
    if (length != 0)
    {
        pQ->pNextEmpty = pBuffer;
    }
    else
    {
        pQ->pNextEmpty = 0; // pNextEmpty == NULL indicates 0 length queue
    }
    pQ->pBuffer = pBuffer;
    pQ->pBufferEnd = &pBuffer[length];
    pQ->sendersWaiting = 0;
    pQ->receiversWaiting = 0;
}

// Initialises a semaphore
// This function MUST be called on a given semaphore before any other rtosSem*() function
// is called on the semaphore.  The easiest and recommended way to ensure this is by
// calling this function on the semaphore before rtosStart() is called.
// 'pS' is the address of the semaphore to initialise
// 'initialCount' is the initial count in the semaphore
void rtosSemInit(RTOS_SEM *pS, uint8_t initialCount)
{
    pS->count = initialCount;
    pS->takersWaiting = 0;
}

// Initialises event flags
// This function MUST be called on a given flags value before any other rtosFlags*()
// function is called on the flags value.  The easiest and recommended way to ensure this
// is by calling this function on the flags value before rtosStart() is called.
// 'pF' is the address of the flags value to initialise
// 'initialValue' is the initial value of the flags
void rtosFlagsInit(RTOS_FLAGS *pF, uint8_t initialValue)
{
    pF->flags = initialValue;
    pF->takersWaiting = 0;
}

// Starts the rtos, by setting up the task switcher interrupt, starting the ticker and
// kicking off the scheduler.
// This function MUST be called with interrupts DISABLED
// This function MUST be called with INT0 disabled in EIMSK register
// DO NOT CALL FROM AN ISR
// The 'startTicker' function must set up the tick generator to start calling
// RTOS_HANDLE_TICK() at the tick frequency.
// 'startTicker' MUST NOT set the global interrupt enable flag in SREG
// This function does not return
void rtosStart(void (*startTicker)(void))
{
    rtos_MB();
    
    // We'll use PD2 as an output and use the pin-change interrupt on that pin
    // (INT0 pin) to trigger task switches
    // See also INT0_vect in rtos_asm.s
    EIMSK &= ~(1<<INT0);    // Disable INT0 interrupt
    DDRD  |=  (1<<DDD2);    // PD2 as output
    EICRA &= ~(1<<ISC01);
    EICRA |=  (1<<ISC00);   // INT0 sensed on any logical change on pin
    EIFR   =  (1<<INTF0);   // Clear INT0 interrupt flag
    // Leave INT0 disabled in EIMSK for now; it will be enabled by the task switcher
    // on the first task switch
    
    // Start calling the tick handler
    startTicker();
    
    // Go to rtosTaskSwitchFirstTime, which will switch to the highest-priority task
    // and then set the global interrupt enable flag in SREG when it returns.  It doesn't
    // return here, it "returns" to the start of the highest priority task, whose start
    // address was already put on its stack by rtosTaskRegister().
    // All tasks were marked ready when they were registered.
    rtosTaskSwitchFirstTime();
    
    // We will never get here
}

// The tick handler
// Increments the tick count and unblocks any blocked task whose timeout expires
// RTOS_HANDLE_TICK() is a macro to be defined elsewhere
// I've assumed that RTOS_HANDLE_TICK() will be defined to make this function
// an interrupt handler, in which case interrupts will automatically be disabled when the
// interrupt happens, and be reenabled when this function returns
// - If that's not the case, make sure interrupts are disabled before we reach this
//   function, and reenabled after we leave this function
RTOS_HANDLE_TICK()
{
    rtos_MB();
    
    tickCounter += 1;
    
    // Check if any blocked task needs to be made ready because its timeout expired
    uint8_t taskBitId = 0x01;
    int taskPrio = 0;
    while (taskPrio <= 7)
    {
        // bit in timeoutIds is nonzero iff the timeout is being used for the
        // corresponding task
        if (((timeoutIds & taskBitId) != 0)
        &&  (blockingSlots[taskPrio].wakeTime == tickCounter))
        {
            ready |= taskBitId;                     // task now ready to run
            blockingSlots[taskPrio].timedOut = 1;   // was made ready by timeout
            timeoutIds ^= taskBitId;                // no longer waiting for timeout
            
            // In case the task was blocked on some item, remove the task's id bit from
            // the bitmap in that item
            if (blockingSlots[taskPrio].pWaitingOn != 0)
            {
                // The task's bit must be set in *pWaitingOn else pWaitingOn would be
                // null, so ^= here clears the bit
                *(blockingSlots[taskPrio].pWaitingOn) ^= taskBitId;
                blockingSlots[taskPrio].pWaitingOn = 0;
            }
        }
        
        taskBitId <<= 1;
        taskPrio += 1;
    }
    
    // If any task we made ready above has higher priority than the current running task
    // then trigger a task switch
    // (Note: the set bit in running is always set in ready, so the ^ is clearing it)
    if ((ready ^ running) > running)
    {
        // Trigger task switch without re-enabling interrupts - interrupts will be
        // reenabled at the end of this ISR
        uint8_t sregVal = 0;
        rtos_TASK_SWITCH_TRIGGER_MB(sregVal);
    }
    
    rtos_MB();
}

// Reads the tick count
// MAY BE CALLED FROM AN ISR
uint32_t rtosTickCountRead(void)
{
    rtos_SAVE_SREG_AND_CLI_MB(savedSreg);
    uint32_t tickCount;
    
    tickCount = tickCounter;
    
    rtos_RESTORE_SREG_MB(savedSreg);
    return tickCount;
}

// Blocks a task for a given delay time:
// 'mode' == RTOS_MODE_BLOCK_DURATION:  block until 'timeout' ticks occur
// 'mode' == RTOS_MODE_BLOCK_UNTIL:     block until tick count reaches 'timeout'
// 'mode' == RTOS_MODE_BLOCK:           block forever; 'timeout' unused
// other:                               does not block
// NOTE for RTOS_MODE_BLOCK_UNTIL, if the timeout value given is more than 0x7fffffff
// ticks in the future, we assume it was meant to happen in the past but we didn't get
// here in time, so in that case we time out immediately
// Upon return, the global interrupt enable flag is set
// DO NOT CALL DIRECTLY FROM AN ISR
// DO NOT CALL BEFORE rtosStart()
//
// This function may cause another task to be made ready, possibly causing a task switch
//
// This function is also used by other rtos functions that need to set a timeout and
// block the current task.  Those other functions:
// - may be called from an ISR
//   - when called from an ISR, they call this function with mode == RTOS_MODE_NO_BLOCK
// - are responsible for setting pWaitingOn and writing the current task's bitId into
//   *pWaitingOn before this function is called
// - DO NOT clear the task's bitId from 'ready' nor set the task's bit in timeoutIds,
//   nor set .timedOut, .wakeTime - those are all the responsibility of this function
void rtosDelay(uint32_t timeout, RTOS_MODE mode)
{
    rtos_CLI_MB();
    uint8_t taskBitId = running;
    
    if ((mode == RTOS_MODE_BLOCK_DURATION) && (timeout != 0))
    {
        timeout += tickCounter;
    }
    // Note ((tickCounter - timeout) >= 0x80000000) ensures we time out immediately if
    // timeout == tickCounter, which is what we want:
    else if ((mode == RTOS_MODE_BLOCK_UNTIL) && ((tickCounter - timeout) >= 0x80000000))
    {
        // keep timeout as-is
    }
    else if (mode == RTOS_MODE_BLOCK)
    {
        // This makes the tick handler ignore the blocking slot:
        taskBitId = 0;
    }
    else
    {
        // Time out immediately
        blockingSlots[curPrio].timedOut = 1;
        if (blockingSlots[curPrio].pWaitingOn != 0)
        {
            // The task's bit must be set in *pWaitingOn else pWaitingOn would be null,
            // so ^= here clears the bit
            *(blockingSlots[curPrio].pWaitingOn) ^= taskBitId;
            blockingSlots[curPrio].pWaitingOn = 0;
        }
        rtos_SEI_MB();
        return;
    }
    
    timeoutIds |= taskBitId;                    // indicates timeout set for this task
    blockingSlots[curPrio].wakeTime = timeout;
    blockingSlots[curPrio].timedOut = 0;        // not yet timed out
    
    // Block the current task
    ready ^= running;   // clears the bit
    // Trigger task switch, set global interrupt flag for immediate task switch
    uint8_t sregVal = 1<<SREG_I;
    rtos_TASK_SWITCH_TRIGGER_MB(sregVal);
}

// Sends a message to a queue, optionally blocking until the message can be sent or a
// timeout occurs
// The timeout behaviour is the same as for rtosDelay(), but the timeout is canceled if
// the message is sent before the timeout occurs
// Only the pointer 'pMessage' is sent, and any value for pMessage is allowed,
// including null
// Returns nonzero on successful send, else zero
//
// This function may block or may cause another task to be made ready, possibly causing
// a task switch
//
// DO NOT CALL BEFORE rtosStart()
//
// (For below, 'I' is the global interrupt enable flag)
// If this function is called from an ISR:
//  - 'mode' MUST be RTOS_MODE_NO_BLOCK
//  - 'I' MUST be 0 when this function is called, and it will not be changed in this
//    function so no interrupts will be serviced during this function
//  - any task switch that is triggered by this function will not occur within this
//    function, but instead the interrupt for the task switch ISR will be pending
//    when this function returns
// If called from non-ISR code:
//  - if 'I' is 1 when this function is called, when this function returns 'I' will be 1
//  - if 'I' is 0 when this function is called:
//      1. if this function doesn't block, 'I' remains 0 throughout this function and is
//          0 on function return
//      2. (else) this function blocks, then 'I' will be 1 when this function returns
//  - interrupts and/or task switches might occur during this function if:
//     1. 'I' is 1 when the function is called; or
//     2. 'mode' is NOT RTOS_MODE_NO_BLOCK
int rtosQueueSend(RTOS_QUEUE *pQ, void *pMessage, uint32_t timeout, RTOS_MODE mode)
{
    rtos_SAVE_SREG_AND_CLI_MB(savedSreg);
    
    // If there is any task waiting to receive from this queue, give the message to
    // the highest-priority such task and make that task ready to run
    // (Note in this case the queue must be empty else nothing would be waiting)
    if (pQ->receiversWaiting != 0)
    {
        uint8_t rxerPrio = pgm_read_byte(&prioFromBitmap[pQ->receiversWaiting]);
        uint8_t rxerBitId = bitId[rxerPrio];
        pQ->receiversWaiting ^= rxerBitId;      // clears bit
        blockingSlots[rxerPrio].pWaitingOn = 0;
        timeoutIds &= ~rxerBitId;               // cancel any timeout
        blockingSlots[rxerPrio].item.q.pMessage = pMessage;
        ready |= bitId[rxerPrio];
        
        // If the task we just made ready has higher priority than the current task,
        // trigger a task switch
        if (rxerPrio > curPrio)
        {
            rtos_TASK_SWITCH_TRIGGER_MB(savedSreg);
            return 1;
        }
        rtos_RESTORE_SREG_MB(savedSreg);
        return 1;
    }
    
    // If the queue has length == 0, or it is full, then we must either fail immediately
    // or block the calling task
    // pFirstFull == pNextEmpty means either queue is full or queue has length 0
    if (pQ->pFirstFull == pQ->pNextEmpty)
    {
        if (mode == RTOS_MODE_NO_BLOCK)
        {
            // Fail immediately
            rtos_RESTORE_SREG_MB(savedSreg);
            return 0;
        }
        else
        {
            // Block caller while waiting to send
            pQ->sendersWaiting |= running;
            blockingSlots[curPrio].pWaitingOn = &(pQ->sendersWaiting);
            blockingSlots[curPrio].item.q.pMessage = pMessage;
            
            // rtosDelay sets up any timeout, then triggers task switch
            rtosDelay(timeout, mode); // note reenables interrupts
            
            // When we get here, it's safe to read timedOut because when this task was
            // reenabled it was no longer waiting on anything, so nothing outside this
            // task will touch timedOut
            if (blockingSlots[curPrio].timedOut != 0)
            {
                // Switched back to this task due to timeout
                rtos_MB();
                return 0;
            }
            // Switched back to this task because message was sent or enqueued
            rtos_MB();
            return 1;
        }
    }
    
    // Put the message in the queue
    *(pQ->pNextEmpty) = pMessage;
    if (pQ->pFirstFull == 0) // Queue was empty
    {
        pQ->pFirstFull = pQ->pNextEmpty;
    }
    pQ->pNextEmpty++;
    if (pQ->pNextEmpty == pQ->pBufferEnd)
    {
        pQ->pNextEmpty = pQ->pBuffer;
    }
    
    rtos_RESTORE_SREG_MB(savedSreg);
    return 1;
}

// Receives a message from a queue, optionally blocking until a message can be received
// or a timeout occurs
// The timeout behaviour is the same as for rtosDelay(), but the timeout is canceled if
// a message is received before the timeout occurs
// Returns nonzero on successful receive, else zero
// If the receive is successful, the received message (of type void*) is put in
// *ppMessage
//
// This function may block or may cause another task to be made ready, possibly causing
// a task switch
//
// DO NOT CALL BEFORE rtosStart()
//
// (For below, 'I' is the global interrupt enable flag)
// If this function is called from an ISR:
//  - 'mode' MUST be RTOS_MODE_NO_BLOCK
//  - 'I' MUST be 0 when this function is called, and it will not be changed in this
//    function so no interrupts will be serviced during this function
//  - any task switch that is triggered by this function will not occur within this
//    function, but instead the interrupt for the task switch ISR will be pending
//    when this function returns
// If called from non-ISR code:
//  - if 'I' is 1 when this function is called, when this function returns 'I' will be 1
//  - if 'I' is 0 when this function is called:
//      1. if this function doesn't block, 'I' remains 0 throughout this function and is
//          0 on function return
//      2. (else) this function blocks, then 'I' will be 1 when this function returns
//  - interrupts and/or task switches might occur during this function if:
//     1. 'I' is 1 when the function is called; or
//     2. 'mode' is NOT RTOS_MODE_NO_BLOCK
int rtosQueueReceive(RTOS_QUEUE *pQ, void **ppMessage, uint32_t timeout, RTOS_MODE mode)
{
    rtos_SAVE_SREG_AND_CLI_MB(savedSreg);
    
    // If the queue is empty and there is no other task waiting to send, we must either
    // fail immediately or block the caller
    // pFirstFull == 0 means queue is empty
    if ((pQ->pFirstFull == 0) && (pQ->sendersWaiting == 0))
    {
        if (mode == RTOS_MODE_NO_BLOCK)
        {
            // Fail immediately
            rtos_RESTORE_SREG_MB(savedSreg);
            return 0;
        }
        else
        {
            // Block waiting to receive
            pQ->receiversWaiting |= running;
            blockingSlots[curPrio].pWaitingOn = &(pQ->receiversWaiting);
            
            // rtosDelay sets up any timeout, then triggers task switch
            rtosDelay(timeout, mode); // note reenables interrupts
            
            // When we get here, it's safe to read blockingSlots[curPrio] because before
            // this task was reenabled it was no longer waiting on anything, so nothing
            // outside this task will touch blockingSlots[curPrio]
            if (blockingSlots[curPrio].timedOut != 0)
            {
                // Switched back to this task due to timeout
                rtos_MB();
                return 0;
            }
            // Switched back to this task because a message was received - it is in
            // .item.q.pMessage
            *ppMessage = blockingSlots[curPrio].item.q.pMessage;
            rtos_MB();
            return 1;
        }
    }
    
    // If there is any task waiting to send, get the message from the highest-priority
    // such task and make that task ready to run
    void *pWaitingMessage = 0;
    uint8_t isMessageWaiting = 0;
    uint8_t txerPrio = 0;
    if (pQ->sendersWaiting != 0)
    {
        txerPrio = pgm_read_byte(&prioFromBitmap[pQ->sendersWaiting]);
        uint8_t txerBitId = bitId[txerPrio];
        pQ->sendersWaiting ^= txerBitId;        // clears bit
        blockingSlots[txerPrio].pWaitingOn = 0;
        timeoutIds &= ~txerBitId;               // cancel any timeout
        pWaitingMessage = blockingSlots[txerPrio].item.q.pMessage;
        isMessageWaiting = 1;
        ready |= txerBitId;
    }
    
    // If the queue is empty, the caller gets the waiting message, else the caller gets
    // the oldest message from the queue
    // pFirstFull == 0 means the queue is empty
    if (pQ->pFirstFull == 0)
    {
        *ppMessage = pWaitingMessage;
        isMessageWaiting = 0;
    }
    else
    {
        *ppMessage = *(pQ->pFirstFull);
        pQ->pFirstFull++;
        if (pQ->pFirstFull == pQ->pBufferEnd)
        {
            pQ->pFirstFull = pQ->pBuffer;
        }
        
        // If the queue is now empty, mark it empty by setting pFirstFull == 0
        if (pQ->pFirstFull == pQ->pNextEmpty)
        {
            pQ->pFirstFull = 0;
        }
    }
    
    // If *ppMessage didn't get the waiting message, then *ppMessage got a message from
    // the queue, so there is now at least one free spot in the queue.  Put the waiting
    // message into the queue.
    if (isMessageWaiting != 0)
    {
        *(pQ->pNextEmpty) = pWaitingMessage;
        if (pQ->pFirstFull == 0) // Queue was empty
        {
            pQ->pFirstFull = pQ->pNextEmpty;
        }
        pQ->pNextEmpty++;
        if (pQ->pNextEmpty == pQ->pBufferEnd)
        {
            pQ->pNextEmpty = pQ->pBuffer;
        }
    }
    
    // If we took a waiting message from a blocked task, and that task has a higher
    // priority than the current task, do a task switch
    if (txerPrio > curPrio)
    {
        rtos_TASK_SWITCH_TRIGGER_MB(savedSreg);
        return 1;
    }
    
    rtos_RESTORE_SREG_MB(savedSreg);
    return 1;
}

// Takes from (i.e. decrements) a semaphore, optionally blocking until it is available or
// a timeout occurs
// The timeout behaviour is the same as for rtosDelay(), but the timeout is canceled if
// the semaphore becomes available before the timeout occurs
// Returns nonzero on successful take, else zero
//
// This function may block, causing a task switch
//
// DO NOT CALL BEFORE rtosStart()
//
// (For below, 'I' is the global interrupt enable flag)
// If this function is called from an ISR:
//  - 'mode' MUST be RTOS_MODE_NO_BLOCK
//  - 'I' MUST be 0 when this function is called, and it will not be changed in this
//    function so no interrupts will be serviced during this function
//  - any task switch that is triggered by this function will not occur within this
//    function, but instead the interrupt for the task switch ISR will be pending
//    when this function returns
// If called from non-ISR code:
//  - if 'I' is 1 when this function is called, when this function returns 'I' will be 1
//  - if 'I' is 0 when this function is called:
//      1. if this function doesn't block, 'I' remains 0 throughout this function and is
//          0 on function return
//      2. (else) this function blocks, then 'I' will be 1 when this function returns
//  - interrupts and/or task switches might occur during this function if:
//     1. 'I' is 1 when the function is called; or
//     2. 'mode' is NOT RTOS_MODE_NO_BLOCK
int rtosSemTake(RTOS_SEM *pS, uint32_t timeout, RTOS_MODE mode)
{
    rtos_SAVE_SREG_AND_CLI_MB(savedSreg);
    
    // If the semaphore has a nonzero count, we can take from it immediately
    if (pS->count > 0)
    {
        pS->count -= 1;
        rtos_RESTORE_SREG_MB(savedSreg);
        return 1;
    }
    
    // Semaphore count is 0, so we have to fail immediately or block the caller
    
    if (mode == RTOS_MODE_NO_BLOCK)
    {
        // Fail immediately
        rtos_RESTORE_SREG_MB(savedSreg);
        return 0;
    }
    
    // Block the caller
    pS->takersWaiting |= running;
    blockingSlots[curPrio].pWaitingOn = &(pS->takersWaiting);
    
    // rtosDelay sets up any timeout, then triggers task switch
    rtosDelay(timeout, mode); // note reenables interrupts
    
    // When we get here, it's safe to read timedOut because when this task was
    // reenabled it was no longer waiting on anything, so nothing outside this
    // task will touch timedOut
    if (blockingSlots[curPrio].timedOut != 0)
    {
        rtos_MB();
        return 0;
    }
    rtos_MB();
    return 1;
}

// Gives to (i.e. increments) a semaphore
// NOTE that this function does not prevent the value from overflowing 255->0
//
// This function may cause another task to be made ready, possibly causing a task switch
//
// DO NOT CALL BEFORE rtosStart()
//
// (For below, 'I' is the global interrupt enable flag)
// If this function is called from an ISR:
//  - 'I' MUST be 0 when this function is called, and it will not be changed in this
//    function so no interrupts will be serviced during this function
// Whether called from an ISR or not, if 'I' is 0 when this function is called then 'I'
// will remain 0 throughout this function; any task switch that is triggered by this
// function will not occur within this function, but instead the interrupt for the task
// switch ISR will be pending when this function returns.  If 'I' is 1 when this
// function is called then any task switch that is triggered will occur within this
// function.
void rtosSemGive(RTOS_SEM *pS)
{
    rtos_SAVE_SREG_AND_CLI_MB(savedSreg);
    
    // If there is any task waiting on the semaphore, unblock the highest-priority such
    // task (Note in this case the count must be 0 else nothing would be waiting)
    if (pS->takersWaiting != 0)
    {
        uint8_t takerPrio = pgm_read_byte(&prioFromBitmap[pS->takersWaiting]);
        uint8_t takerBitId = bitId[takerPrio];
        pS->takersWaiting ^= takerBitId;        // clears bit
        blockingSlots[takerPrio].pWaitingOn = 0;
        timeoutIds &= ~takerBitId;              // cancel any timeout
        ready |= takerBitId;
        
        // If the task we just made ready has higher priority than the current task,
        // trigger a task switch
        if (takerPrio > curPrio)
        {
            rtos_TASK_SWITCH_TRIGGER_MB(savedSreg);
            return;
        }
    }
    else
    {
        // No taker waiting, so just increment the semaphore count
        pS->count += 1;
    }
    
    rtos_RESTORE_SREG_MB(savedSreg);
}

// Takes event flag(s), optionally blocking until they are available or a timeout occurs
// 'flags' is the set of flags requested, and must be nonzero
// 'all' == 0: takes the least-significant set flag bit among those requested, or none.
// 'all' != 0: takes all flag bits among those requested, or none.
// The timeout behaviour is the same as for rtosDelay(), except that the timeout will be
// canceled if the requested flag(s) are obtained before the timeout occurs
// The flag bits taken are cleared from *pF
// Returns the flag bit(s) taken; 0 if it couldn't get flag bit(s) before timeout
//
// This function may block, causing a task switch
//
// DO NOT CALL BEFORE rtosStart()
//
// (For below, 'I' is the global interrupt enable flag)
// If this function is called from an ISR:
//  - 'mode' MUST be RTOS_MODE_NO_BLOCK
//  - 'I' MUST be 0 when this function is called, and it will not be changed in this
//    function so no interrupts will be serviced during this function
//  - any task switch that is triggered by this function will not occur within this
//    function, but instead the interrupt for the task switch ISR will be pending
//    when this function returns
// If called from non-ISR code:
//  - if 'I' is 1 when this function is called, when this function returns 'I' will be 1
//  - if 'I' is 0 when this function is called:
//      1. if this function doesn't block, 'I' remains 0 throughout this function and is
//          0 on function return
//      2. (else) this function blocks, then 'I' will be 1 when this function returns
//  - interrupts and/or task switches might occur during this function if:
//     1. 'I' is 1 when the function is called; or
//     2. 'mode' is NOT RTOS_MODE_NO_BLOCK
uint8_t rtosFlagsTake(
    RTOS_FLAGS *pF, uint8_t flags, uint8_t all, uint32_t timeout, RTOS_MODE mode)
{
    rtos_SAVE_SREG_AND_CLI_MB(savedSreg);
    
    // Check if flag(s) requested are available
    uint8_t available = pF->flags & flags;
    uint8_t taken = 0;
    if ((all != 0) && (available == flags))
    {
        // Request is for all flags, and they're all available
        taken = available;
    }
    else if ((all == 0) && (available != 0))
    {
        // Request is for any of the flags, and at least one is available
        // Take the least-significant one
        taken = ((available - 1) & available) ^ available;
    }
    else
    {
        // Not available; fail immediately or block caller
        if (mode == RTOS_MODE_NO_BLOCK)
        {
            // Fail immediately
            rtos_RESTORE_SREG_MB(savedSreg);
            return 0;
        }
        
        // Block the caller
        pF->takersWaiting |= running;
        blockingSlots[curPrio].pWaitingOn = &(pF->takersWaiting);
        blockingSlots[curPrio].item.f.flags = flags;
        blockingSlots[curPrio].item.f.all = all;

        // rtosDelay sets up any timeout, then triggers task switch
        rtosDelay(timeout, mode); // note reenables interrupts
        
        // When we get here, it's safe to read blockingSlots[curPrio] because when this
        // task was reenabled it was no longer waiting on anything, so nothing outside
        // this task will touch blockingSlots[curPrio]
        if (blockingSlots[curPrio].timedOut != 0)
        {
            rtos_MB();
            return 0;
        }
        
        // Whoever reenabled this task will have updated .flags to the ones actually
        // taken, and will have cleared those flags in *pF for us
        flags = blockingSlots[curPrio].item.f.flags;
        rtos_MB();
        return flags;
    }
    
    pF->flags ^= taken; // clears those taken
    rtos_RESTORE_SREG_MB(savedSreg);
    return taken;
}

// Sets the given event flags
//
// This function may cause one or more tasks to be made ready, possibly causing a task
// switch
//
// DO NOT CALL BEFORE rtosStart()
//
// (For below, 'I' is the global interrupt enable flag)
// If this function is called from an ISR:
//  - 'I' MUST be 0 when this function is called, and it will not be changed in this
//    function so no interrupts will be serviced during this function
// Whether called from an ISR or not, if 'I' is 0 when this function is called then 'I'
// will remain 0 throughout this function; any task switch that is triggered by this
// function will not occur within this function, but instead the interrupt for the task
// switch ISR will be pending when this function returns.  If 'I' is 1 when this
// function is called then any task switch that is triggered will occur within this
// function.
void rtosFlagsSet(RTOS_FLAGS *pF, uint8_t flags)
{
    rtos_SAVE_SREG_AND_CLI_MB(savedSreg);
    
    pF->flags |= flags;
    
    // Check if any task(s) blocked on *pF can now take its requested flag(s) and run
    // If so give the task(s) the flags and unblock the task(s), in highest-priority
    // first order
    uint8_t takerPrio = 7;
    uint8_t takerBitId = 0x80;
    while (takerBitId != 0)
    {
        uint8_t taken = 0;
        if ((pF->takersWaiting & takerBitId) != 0)
        {
            flags = blockingSlots[takerPrio].item.f.flags;
            uint8_t available = pF->flags & flags;
            if ((blockingSlots[takerPrio].item.f.all != 0) && (available == flags))
            {
                // Request is for all flags, and they're all available
                taken = available;
            }
            else if ((blockingSlots[takerPrio].item.f.all == 0) && (available != 0))
            {
                // Request is for any of the flags, and at least one is available
                // Take the least-significant one
                taken = ((available - 1) & available) ^ available;
            }
            
            if (taken != 0)
            {
                // Give this task the taken flags and unblock this task
                pF->flags ^= taken; // clears those taken
                pF->takersWaiting ^= takerBitId; // clears bit
                blockingSlots[takerPrio].item.f.flags = taken;
                blockingSlots[takerPrio].pWaitingOn = 0;
                timeoutIds &= ~takerBitId;  // cancel any timeout
                ready |= takerBitId;
            }
        }
        
        takerPrio -= 1;
        takerBitId >>= 1;
    }
    
    // If any task that we just made ready has a higher priority than the current
    // task, do a task switch
    // (Note: the set bit in running is always set in ready, so the ^ is clearing it)
    if ((ready ^ running) > running)
    {
        rtos_TASK_SWITCH_TRIGGER_MB(savedSreg);
        return;
    }
    
    rtos_RESTORE_SREG_MB(savedSreg);
}

// Clears the given event flags
// MAY BE CALLED FROM AN ISR
// (For below, 'I' is the global interrupt enable flag)
// If 'I' is 0 when this function is called then 'I' will remain 0 throughout this
// function and be 0 upon return.
void rtosFlagsClear(RTOS_FLAGS *pF, uint8_t flags)
{
    rtos_SAVE_SREG_AND_CLI_MB(savedSreg);
    
    pF->flags &= ~flags;
    
    rtos_RESTORE_SREG_MB(savedSreg);
}
