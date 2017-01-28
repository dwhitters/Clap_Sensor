/**
    @author David Whitters
    @date 1/24/17
    @title Config.h
    @description This file contains the configuration bits necessary to set up a dsPIC33F.
*/

#ifndef CONFIG_H
#define CONFIG_H

/* Startup with oscillator (FRC) */
#pragma config FNOSC = FRCPLL

/*
    Primary Oscillator Disabled, OSC2 pin has clock out function,
    Allow Multiple Re-configurations, and Clock Switching enabled and Fail-Safe
    Clock Monitor is disabled.
 */
#pragma config OSCIOFNC = OFF
#pragma config FCKSM = CSECMD
#pragma config IOL1WAY = OFF
#pragma config POSCMD = NONE

/* Disable Watchdog */
#pragma config FWDTEN = OFF

/* Disable JTAG, enable debugging on PGx1 pins */
#pragma config JTAGEN = OFF
#pragma config ICS = PGD1

#endif  // CONFIG_H
