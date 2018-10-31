
#ifndef H_TIMERCODE_INCLUDED
#define H_TIMERCODE_INCLUDED 1

#include "Arduino.h"

void setupTimer()
{
    //uses timer 2 to generate 1kHz interrupts

    cli();  //stop interrupts

    TCCR2A = 0; // set entire TCCR2A register to 0
    TCCR2B = 0; // same for TCCR2B
    TCNT2 = 0;  //initialize counter value to 0

    // set compare match register for 1khz increments
    OCR2A = 128; // = 16000000/(128*1000) - 1
    // turn on CTC mode
    TCCR2A |= (1 << WGM21);
    // Set CS21 bit for 128 prescaler
    TCCR2B |= (1 << CS22);
    TCCR2B |= (1 << CS20);
    // enable timer compare interrupt
    TIMSK2 |= (1 << OCIE2A);

    sei();      //enable interrupts
}


ISR(TIMER2_COMPA_vect)
{
    //passes the read temperature through a low pass filter
    //calculates the difference e and u
    uVal = pid.nextStep(setTemp - lpf.nextStep(currTemp));      
}

#endif