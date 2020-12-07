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


#include <avr/io.h>
#include "timer0.h"

// Timer0 is used for RTOS tick generation at 1kHz

#if (F_CPU == 8000000)
    
    // Use clkI/O / 64 as timer clock source, so timer is clocked at 125000 Hz
    // Repeatedly count from 0 to 124 to get an interrupt frequency of 1 kHz
    #define TCCR0A_SETTING      (1<<WGM01)              // CTC mode
    #define TCCR0B_SETTING      ((1<<CS01) | (1<<CS00)) // clkI/O / 64
    #define OCR0A_SETTING       124
    
#else
    
    #error Calculate timer/counter 0 settings!
    
#endif


// ---- Functions ----

void Timer0Init(void)
{
    // Disable timer
    TIMSK0 = 0;                 // disable timer interrupts
    TCCR0B = 0;                 // stop timer
    TCNT0 = 0;                  // clear timer
    TIFR0 = (1<<OCF0B) | (1<<OCF0A) | (1<<TOV0); // clear interrupt flags
    
    // Set control registers and timer period
    PRR &= ~(1<<PRTIM0);        // Make sure timer 0 not disabled for power reduction
    TCCR0A = TCCR0A_SETTING;
    TIMSK0 = (1<<OCIE0A);       // Enable timer 0 compare match A interrupt
    OCR0A = OCR0A_SETTING;
    TCCR0B = TCCR0B_SETTING;    // Starts timer
}
