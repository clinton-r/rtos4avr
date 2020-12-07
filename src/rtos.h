/////////////////////////////////////////////////////////////////////////////////////////

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


#ifndef RTOS_H
#define RTOS_H

#include <inttypes.h>

// Note to users: everything in this struct is managed by rtos functions; you don't need
// to access any of its members
typedef struct
{
    // Queue is managed as a circular FIFO buffer
    // Queue entries are pointers to void
    void **pNextEmpty;          // NULL iff queue has zero length
    void **pFirstFull;          // NULL when queue is empty or has zero length
    // Note if pNextEmpty == pFirstFull (and non-NULL) then the queue is full
    
    void **pBuffer;             // Points to message buffer
    void **pBufferEnd;          // Points to first byte after last space in buffer
    
    uint8_t sendersWaiting;     // Bitmap; tasks blocked waiting to send to queue
    uint8_t receiversWaiting;   // Bitmap; tasks blocked waiting to receive from queue
} RTOS_QUEUE;

// Note to users: everything in this struct is managed by rtos functions; you don't need
// to access any of its members
typedef struct
{
    uint8_t count;
    uint8_t takersWaiting;      // Bitmap; tasks blocked waiting to take semaphore
} RTOS_SEM;

// Note to users: everything in this struct is managed by rtos functions; you don't need
// to access any of its members
typedef struct
{
    uint8_t flags;
    uint8_t takersWaiting;      // Bitmap; tasks blocked waiting to take flags
} RTOS_FLAGS;


// Modes to be passed to delay, queue, semaphore, etc. functions
typedef uint8_t RTOS_MODE;
                                        // While action not immediately possible...
#define RTOS_MODE_NO_BLOCK          1   // ... return immediately;
#define RTOS_MODE_BLOCK_DURATION    2   // ... block for at most 'timeout' ticks
#define RTOS_MODE_BLOCK_UNTIL       3   // ... block at most until tick count = 'timeout'
#define RTOS_MODE_BLOCK             4   // ... block until action completes


extern void rtosTaskRegister(void (*task)(void), int priority, uint8_t *pStack);
extern void rtosQueueInit(RTOS_QUEUE *pQ, void **pBuffer, uint8_t length);
extern void rtosSemInit(RTOS_SEM *pS, uint8_t initialCount);
extern void rtosFlagsInit(RTOS_FLAGS *pF, uint8_t initialValue);
extern void rtosStart(void (*startTicker)(void));

extern uint32_t rtosTickCountRead(void);
extern void rtosDelay(uint32_t timeout, RTOS_MODE mode);

extern int rtosQueueSend(RTOS_QUEUE *pQ, void *pMessage, uint32_t timeout, RTOS_MODE mode);
extern
 int rtosQueueReceive(RTOS_QUEUE *pQ, void **ppMessage, uint32_t timeout, RTOS_MODE mode);

extern int rtosSemTake(RTOS_SEM *pS, uint32_t timeout, RTOS_MODE mode);
extern void rtosSemGive(RTOS_SEM *pS);

extern uint8_t rtosFlagsTake(
    RTOS_FLAGS *pF, uint8_t flags, uint8_t all, uint32_t timeout, RTOS_MODE mode);
extern void rtosFlagsSet(RTOS_FLAGS *pF, uint8_t flags);
extern void rtosFlagsClear(RTOS_FLAGS *pF, uint8_t flags);

#endif // RTOS_H
