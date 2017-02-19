/**
    @author David Whitters
    @date 1/24/17
    @title Peripherals.h
    @description This file contains the function prototypes necessary to set up and control PWM,
                 ADC, and Input Capture.
*/

#ifndef PERIPHERALS_H
#define PERIPHERALS_H

#include <p33Fxxxx.h>
#include <stdint.h>

#define ADC_EIGHTH_VOLT     ((uint16_t)162u)
#define ADC_QTR_VOLT        ((uint16_t)323u)
#define ADC_HALF_VOLT        ((uint16_t)646u)
#define ADC_ONE_VOLT        ((uint16_t)1292u)
#define ADC_TWO_VOLT        (2 * ADC_ONE_VOLT)
#define ADC_SAMPS_PER_MS    ((uint8_t)33u)
#define ADC_MAX_VAL         ((uint16_t)0x0FFFu)

void Peripherals_ADC_Init(void);
uint16_t Peripherals_ADC_Convert(void);
void Peripherals_IC2_Init(void);
void Peripherals_IC2_SetInt(void);
void Peripherals_PWM_Init(void);
void Peripherals_PWM_SetDC(uint8_t channel, uint8_t duty_cycle);
void Peripherals_EnableInterrupts(void);
void Peripherals_DisableInterrupts(void);

#endif  // PERIPHERALS_H
