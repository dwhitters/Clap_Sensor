/**
    @author David Whitters
    @date 1/24/17
    @title OSC.h
    @description This file contains the functions necessary to set up the oscillator on a dsPIC33F.
*/

#ifndef OSC_H
#define OSC_H

#define CPU_CLK_RATE    40000000L
#define PERIPH_CLK_RATE (CPU_CLK_RATE/2L)

#include <p33Fxxxx.h>

void OSC_Init(void);

#endif  // OSC_H
