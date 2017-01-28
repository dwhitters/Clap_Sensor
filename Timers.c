/**
    @author David Whitters
    @date 1/24/17
    @title Timers.c
    @description This file contains the functions necessary to set up and control timer1 and timer2.
*/

#include "Timers.h"

void Timers_T1_Setup(uint16_t period)
{
    T1CONbits.TON = 0u;         // Disable Timer
    T1CONbits.TCS = 0u;         // Select internal instruction cycle clock
    T1CONbits.TGATE = 0u;       // Disable Gated Timer mode
    T1CONbits.TCKPS = 0b11u;    // Select 1:256 Prescalar
    TMR1 = 0u;                  // Clear timer register
    PR1 = period;                // Load the period value
}

void Timers_T2_Setup(uint16_t period)
{
    T2CONbits.TON = 0u;         // Disable Timer
    T2CONbits.TCS = 0u;         // Select internal instruction cycle clock
    T2CONbits.TGATE = 0u;       // Disable Gated Timer mode
    T2CONbits.TCKPS = 0b11u;    // Select 1:256 Prescalar
    TMR2 = 0u;                  // Clear timer register
    PR2 = period;                // Load the period value
}

void Timers_Start(void)
{
    T1CONbits.TON = 1u;     // Start timer 1
    T2CONbits.TON = 1u;     // Start timer 2
}

void Timers_Stop(void)
{
    T1CONbits.TON = 0u;     // Stop timer 1
    T2CONbits.TON = 0u;     // Stop timer 2
}

void Timers_T1_SetInt(void)
{
    IPC0bits.T1IP = 3u; // Set Timer1 interrupt priority to 3
    IFS0bits.T1IF = 0u;  // Reset Timer 1 interrupt flag
    IEC0bits.T1IE = 1u; // Timer 1 interrupt enabled
}

void Timers_T2_SetInt(void)
{
    IPC1bits.T2IP = 6u; // Set Timer2 interrupt priority to 6
    IFS0bits.T2IF = 0u;  // Reset Timer 2 interrupt flag
    IEC0bits.T2IE = 1u; // Timer 2 interrupt enabled
}
