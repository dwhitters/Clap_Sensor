/*
 * File:   main.c
 * Author: david
 *
 * Created on December 19, 2016, 11:13 PM
 */

#include <p33Fxxxx.h>
#include <stdint.h>
#include "Config.h"
#include "OSC.h"
#include "Peripherals.h"
#include "Timers.h"

/* 1.5 Volts */
#define FIRST_HIGH_THRESH  (ADC_ONE_VOLT) //((uint16_t)1938u)
/* 1 Volts */
#define SECOND_HIGH_THRESH  (ADC_ONE_VOLT)
/* 0.5 Volts */
#define LOW_THRESH  ((uint16_t)646u)

#define ADC_TRIS    (TRISAbits.TRISA0)
#define ADC_I       (PORTAbits.RA0)
#define ADC_O       (LATAbits.LATA0)

#define BTN_POW_TRIS    (TRISAbits.TRISA1)
#define BTN_POW_I       (PORTAbits.RA1)
#define BTN_POW_O       (LATAbits.LATA1)

#define RGB_R_TRIS      (TRISBbits.TRISB6)
#define RGB_G_TRIS      (TRISBbits.TRISB7)
#define RGB_B_TRIS      (TRISBbits.TRISB8)
#define RGB_R_O         (LATBbits.LATB6)
#define RGB_G_O         (LATBbits.LATB7)
#define RGB_B_O         (LATBbits.LATB8)

void SenseClaps(void);

typedef enum {
    HIGH = 1u,
    LOW
} SEARCH_t;

typedef enum {
    CLAP_STATE = 1u,
    BEAT_STATE
} STATE_t;

typedef enum {
    RED = 1u,
    BLU,
    GRN,
    WHITE
} RGB_t;

void __attribute__((__interrupt__, no_auto_psv)) _T2Interrupt(void);
void __attribute__((__interrupt__, no_auto_psv)) _IC2Interrupt(void);

/* The state that the sensor is currently in */
STATE_t Current_State = CLAP_STATE;
/* The number of claps that occur in a given time segment */
uint8_t Clap_Count = 0u;
/* Set to one when searching for spike, set to zero when searching for valley */
uint8_t Count_Claps;
uint8_t Timer_Running = 0u;

/*
 *
 */
int main(int argc, char** argv) {

    OSC_Init();
    AD1PCFGL = 0x0006u; // Disable analog functionality on pins 3 and 4

    Timers_T2_Setup(THREE_QUARTERS_SEC_PERIOD);
    Peripherals_ADC_Init();
    Peripherals_PWM_Init();

    // Disable interrupt nesting
    INTCON1bits.NSTDIS = 1u;

    Timers_T2_SetInt();

    BTN_POW_TRIS = 0u;      // Set RA1 to output
    BTN_POW_O = 1u;         // Set RA1 pin to HIGH

    Peripherals_EnableInterrupts();
    Timers_Start();

    while(1u)
    {
        switch(Current_State)
        {
            case CLAP_STATE:
                SenseClaps();
                break;
            case BEAT_STATE:
                break;
            default:
                Current_State = BEAT_STATE;
                break;
        }
    }

    return (0u);
}

void __attribute__((__interrupt__, no_auto_psv)) _T2Interrupt(void)
{
    if(Clap_Count >= 1u)
    {
        switch(State)
        {
            case RED:
                Peripherals_PWM_SetDC(1, 100);
                Peripherals_PWM_SetDC(2, 0);
                Peripherals_PWM_SetDC(3, 0);
                State = GRN;
                break;
            case GRN:
                Peripherals_PWM_SetDC(1, 0);
                Peripherals_PWM_SetDC(2, 100);
                Peripherals_PWM_SetDC(3, 0);
                State = BLU;
                break;
            case BLU:
                Peripherals_PWM_SetDC(1, 0);
                Peripherals_PWM_SetDC(2, 0);
                Peripherals_PWM_SetDC(3, 100);
                State = WHITE;
                break;
            case WHITE:
                Peripherals_PWM_SetDC(1, 100);
                Peripherals_PWM_SetDC(2, 100);
                Peripherals_PWM_SetDC(3, 100);
                State = RED;
                break;
            default:
                break;
        }
    }

    Timer_Running = 0u;
    Clap_Count = 0u;
    IFS0bits.T2IF = 0u;        // Clear flag
}

//void __attribute__((__interrupt__, no_auto_psv)) _IC2Interrupt(void)
//{
//    Clap_Count++;
//
//    IFS0bits.IC2IF = 0u;        // Clear flag
//}

/*
void _ISR _T2Interrupt(void)
{
    LED_O ^= 1u;
    IFS0bits.T1IF = 0u;         // Clear Timer1 Interrupt Flag
}
*/

void SenseClaps(void)
{
    static uint8_t high_found = 0u;
    static uint16_t adc_val = 0u;
    static SEARCH_t search_type = HIGH;

    // Sample until a measurement of interest occurs
    adc_val = Peripherals_ADC_Convert();
    switch(search_type)
    {
        case LOW:
            if(adc_val < LOW_THRESH)
            {
                if(high_found == 1u)
                {
                    high_found = 0u;
                    if(0u == Timer_Running)
                    {
                        Clap_Count++;
                        Timer_Running = 0u;
                    }
                }
                search_type = HIGH;
            }
            break;
        case HIGH:
            // High frequencies are attenuated due to the low pass filter in place
            if((adc_val > FIRST_HIGH_THRESH) || ((adc_val > SECOND_HIGH_THRESH) && (1u == Timer_Running)))
            {
                if(Timer_Running == 0u)
                {
                    // Set timer count register to 0
                    TMR2 = 0u;
                    Timer_Running = 1u;
                    high_found = 1u;
                    search_type = LOW;
                }
                else if(TMR2 > (20 * T2_ONE_MS))
                {
                    Timer_Running = 0u;
                    high_found = 1u;
                    search_type = LOW;
                }
            }
        default:
            break;
    }
}

void PeakDetection(void)
{
    static uint8_t high_found = 0u;
    static uint16_t adc_val = 0u;
    static SEARCH_t search_type = HIGH;

    // Sample until a measurement of interest occurs
    adc_val = Peripherals_ADC_Convert();
    switch(search_type)
    {
        case LOW:
            if(adc_val < LOW_THRESH)
            {
                if(high_found == 1u)
                {
                    high_found = 0u;
                    if(0u == Timer_Running)
                    {
                        Clap_Count++;
                        Timer_Running = 0u;
                    }
                }
                search_type = HIGH;
            }
            break;
        case HIGH:
            // High frequencies are attenuated due to the low pass filter in place
            if((adc_val > FIRST_HIGH_THRESH) || ((adc_val > SECOND_HIGH_THRESH) && (1u == Timer_Running)))
            {
                if(Timer_Running == 0u)
                {
                    // Set timer count register to 0
                    TMR2 = 0u;
                    Timer_Running = 1u;
                    high_found = 1u;
                    search_type = LOW;
                }
                else if(TMR2 > (20 * T2_ONE_MS))
                {
                    Timer_Running = 0u;
                    high_found = 1u;
                    search_type = LOW;
                }
            }
        default:
            break;
    }
}
