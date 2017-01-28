/**
    @author David Whitters
    @date 1/24/17
    @title OSC.c
    @description This file contains the functions necessary to set up the oscillator on a dsPIC33F.
*/

#include "OSC.h"

void OSC_Init(void)
{
    // Set clock to 80MHz (40MIPs). Actually sets it to 40MHz...
    //CLKDIVbits.FRCDIV = 0u;
    CLKDIVbits.PLLPRE = 0u;         // Divide by 2
    PLLFBD = 41u;                   // PLL multiplier before 2 added
    CLKDIVbits.PLLPOST = 0u;        // Divide by 2

    // Tune FRC to a center frequency of 7.37 kHz
    OSCTUNbits.TUN = 0u;

    // Clock switching to incorporate PLL
    __builtin_write_OSCCONH(0x01);  // Initiate Clock Switch to Primary
                // Oscillator with PLL (NOSC=0b011)
    __builtin_write_OSCCONL(0x01);  // Start clock switching
    while (OSCCONbits.COSC != 0b001); // Wait for Clock switch to occur
   // Wait for PLL to lock
    while(OSCCONbits.LOCK!=1);
}

