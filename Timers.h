/**
    @author David Whitters
    @date 1/24/17
    @title Timers.h
    @description This file contains the function prototypes necessary to set up and control timer1 and timer2.
*/

#ifndef TIMERS_H
#define TIMERS_H

#include <p33Fxxxx.h>
#include <stdint.h>

#define T2_ONE_MS           ((uint16_t)156u)
#define HALF_SEC_PERIOD     ((uint16_t)38941u)
#define THREE_QUARTERS_SEC_PERIOD   ((uint16_t)58412u)
#define MAX_TIME            ((uint16_t)65535u)

void Timers_Start(void);
void Timers_Stop(void);
void Timers_T1_Setup(uint16_t period);
void Timers_T2_Setup(uint16_t period);
void Timers_T1_SetInt(void);
void Timers_T2_SetInt(void);

#endif  // TIMERS_H
