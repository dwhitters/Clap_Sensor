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
#define FIRST_HIGH_THRESH  (ADC_TWO_VOLT) //((uint16_t)1938u)
/* 1 Volts */
#define SECOND_HIGH_THRESH  (ADC_TWO_VOLT)
/* 0.5 Volts */
#define LOW_THRESH  ((uint16_t)646u)

#define BEAT_BUFF_LENGTH ((uint16_t)300u)
#define BEAT_THRESH (ADC_HALF_VOLT)

/** All peaks greater than this set the LED color to white */
#define MAX_PEAK        ((uint16_t)0xC00u)
/** ADC read range per color of the LED */
#define COLOR_RANGE     ((uint16_t)347u)

#define PEAKS_PER_PERIOD    ((uint8_t)5u)
#define NUM_STORED_PERIODS  ((uint8_t)56u)
#define NUM_STORED_PEAKS    ((uint8_t)3u)

/** PWM channel of the RED LEDs */
#define RED_CHANNEL     ((uint8_t)1u)
/** PWM channel of GRN LEDs */
#define GRN_CHANNEL     ((uint8_t)3u)
/** PWM channel of BLU LEDs */
#define BLU_CHANNEL     ((uint8_t)2u)
/** Amount of time between fades of the LEDs */
#define FADE_WAIT       ((uint16_t)(T2_ONE_MS * 15u))

#define BTN_POW_TRIS    (TRISAbits.TRISA1)
#define BTN_POW_I       (PORTAbits.RA1)
#define BTN_POW_O       (LATAbits.LATA1)

/* Function prototypes */
void FindPeak(uint16_t low_thresh, uint16_t high_thresh);
void SenseClaps(void);
void ProcessPeriods(void);
void SenseBeats(void);
void SetColor(void);
void ChooseColor(uint16_t beat_size);
void FadeColor(void);

/*
    The two main states of the system. Searches for two consecutive claps within 3/4 of a second
    during the clap state. The system then enters the beat state. The beat state searches for
    voltage spikes (beats), and changes the PWM of the LEDs accordingly.
*/
typedef enum {
    CLAP_STATE = 1u,
    BEAT_STATE
} STATE_t;

/* Used to set Timer_State when sensing claps. NEW_PERIOD is the default state. */
typedef enum {
    SAME_PERIOD = 1u,
    NEW_PERIOD
} PERIOD_t;

typedef enum {
    RED = 0u,
    GRN,
    BLU,
    RED_GRN,
    RED_BLU,
    BLU_GRN,
    RED_BLU_GRN
} COLOR_t;

/** Stores the current color. Used to fade the LEDs. */
struct Color {
    uint8_t Red;
    uint8_t Grn;
    uint8_t Blu;
} Curr_Color, Org_Color;

void __attribute__((__interrupt__, no_auto_psv)) _T2Interrupt(void);
void __attribute__((__interrupt__, no_auto_psv)) _IC2Interrupt(void);

/* The state that the sensor is currently in */
STATE_t Current_State = CLAP_STATE;
/* The number of claps that occur in a given time segment */
uint8_t Clap_Count = 0u;
/* Set to one when searching for spike, set to zero when searching for valley */
uint8_t Count_Claps;
/* State of the timer. Set to NEW_PERIOD every Timer2 interrupt. */
PERIOD_t Timer_State = NEW_PERIOD;
/** Time between fading the LEDs */
uint16_t Fade_Time = 0u;

/** The minimum value stored in Max_Peaks */
uint16_t Max_Comp_Peak = 0u;
/** Set to 1 when timer 2 rolls over */
uint8_t T2_Rollover = 0u;
/** The three largest peaks within the last 21 seconds 
      VAL    PERIOD 
    |  X    |  X   |
    |  X    |  X   |
    |  X    |  X   |
 */
uint16_t Max_Peaks[NUM_STORED_PEAKS][2u];
uint8_t Curr_Period = 0u;
uint8_t Peak_Count = 0u;

uint16_t Peaks[NUM_STORED_PERIODS][PEAKS_PER_PERIOD];
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

    Curr_Color.Red = 0u;
    Curr_Color.Grn = 0u;
    Curr_Color.Blu = 0u;
    
    Curr_Color.Red = 100;
    Peripherals_PWM_SetDC(RED_CHANNEL, Curr_Color.Red);

    while(1u)
    {
        switch(Current_State)
        {
            case CLAP_STATE:
                SenseClaps();
                break;
            case BEAT_STATE:
                SenseBeats();
                break;
            default:
                Current_State = CLAP_STATE;
                break;
        }
        
        if(T2_Rollover == 1u)
        {
            ProcessPeriods();
        }
    }
    return (0u);
}

void __attribute__((__interrupt__, no_auto_psv)) _T2Interrupt(void)
{
    if((Clap_Count >= 1u) && (Current_State == CLAP_STATE))
    {
        Current_State = BEAT_STATE;
        BTN_POW_O = 0u;     // Turn off RA1
    }
    
    if(Current_State == BEAT_STATE)
    {
        T2_Rollover = 1u;
    }
    else
    {
        Timer_State = NEW_PERIOD;
        Clap_Count = 0u;
    }

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

void ProcessPeriods(void)
{
    uint8_t i = 0u;
    uint8_t flag = 0u;
    uint16_t max_peak = 0u;
    uint8_t period_clear = Curr_Period + 1u;

    if(period_clear == NUM_STORED_PERIODS)
    {
        period_clear = 0u;
    }

    // Get max peak from current period
    for(i = 0u; i < PEAKS_PER_PERIOD; i++)
    {
        if(Peaks[Curr_Period][i] > max_peak)
        {
            max_peak = Peaks[Curr_Period][i];
        }

        // Clear upcoming period
        Peaks[period_clear][i] = 0u;
    }

    // If the current period matches a stored max peak period, replace the max peak
    for(i = 0; i < NUM_STORED_PEAKS; i++)
    {
        if(Max_Peaks[i][1u] == Curr_Period)
        {
            // Period remains the same
            Max_Peaks[i][0u] = max_peak;
            // Leave loop so periods of zero in the array are not all set
            i = NUM_STORED_PEAKS;
            flag = 1u;
        }
    }

    // If peak not already set...
    if(flag == 0u)
    {
        flag = 0u;
        uint8_t diff = 0u;
        uint8_t max_diff = 0u;
        uint8_t comp_period = Curr_Period;
        uint8_t oldest_period = 0u;
        for(i = 0u; i < NUM_STORED_PEAKS; i++)
        {
            comp_period = Curr_Period;
            if(comp_period < Max_Peaks[i][1u])
            {
                comp_period += NUM_STORED_PERIODS;
            }
            diff = comp_period - Max_Peaks[i][1u];

            if(diff > max_diff)
            {
                max_diff = diff;
                oldest_period = i;
            }
            // If the max peak found this period is greater than a stored max peak
            if(Max_Peaks[i][0u] < max_peak)
            {
                flag = 1u;
            }
        }

        if(flag == 1u)
        {
            Max_Peaks[oldest_period][0u] = max_peak;
            Max_Peaks[oldest_period][1u] = Curr_Period;
        }
    }

    // If a peak is being replaced, find new minimum max peak
    if(flag == 1u)
    {
        uint16_t min_peak = Max_Peaks[0u][0u];
        for(i = 0; i < NUM_STORED_PEAKS; i++)
        {
            if((Max_Peaks[i][0u] < min_peak) && (Max_Peaks[i][0u] != 0u))
            {
                min_peak = Max_Peaks[i][0u];
            }
        }

        Max_Comp_Peak = min_peak;
    }
    
    Peak_Count = 0u;
    T2_Rollover = 0u;
    
    Curr_Period++;
    if(Curr_Period == NUM_STORED_PERIODS)
    {
        Curr_Period = 0u;
    }
}

/*
    Find beats in the music.
*/
void SenseBeats(void)
{
    uint16_t adc_val = 0u;
    /*
        Array that stores sequential ADC reads to store the peaks. The buffer is populated
        for every spike, starting at 0.5V. The values are recorded until the voltage drops below 0.5V.
    */
    //static uint16_t beat_buffer[BEAT_BUFF_LENGTH] = {0u};

    adc_val = Peripherals_ADC_Convert();
    BTN_POW_O ^= 1u;     // Turn off RA1
    if((adc_val > Max_Comp_Peak))// || ((Max_Comp_Peak - adc_val) < (Max_Comp_Peak / 3)))
    {
        // Find top of peak by waiting until the adc reading is less than the previous one.
        uint16_t prev_adc_val;
        do
        {
            prev_adc_val = adc_val;
            adc_val = Peripherals_ADC_Convert();
        } while(prev_adc_val < adc_val);
        
        // Set to the highest found value
        Peaks[Curr_Period][Peak_Count] = prev_adc_val;
        Peak_Count++;
        
        ChooseColor(prev_adc_val);

        // Wait for at least 100ms to ensure the original signal isn't re-sampled
        uint16_t timer_val = TMR2;
        TMR2 = 0u;
        while(TMR2 < (100 * T2_ONE_MS));
        
        if(((timer_val + 100 * T2_ONE_MS) > THREE_QUARTERS_SEC_PERIOD) || (Peak_Count == PEAKS_PER_PERIOD))
        {
            IFS0bits.T2IF = 1u;  // Set interrupt flag as the timer would have triggered an interrupt
        }
        else
        {
            // Set timer to what it would have been.
            TMR2 = timer_val + (100 * T2_ONE_MS);
        }
    }
    
    if(FADE_WAIT < (TMR2 - Fade_Time))
    {
        Fade_Time = TMR2;
        FadeColor();
    }
}

/** 
 * Keeps the same color ratio, just dims the LEDS.
 */
void FadeColor(void)
{
    if((Curr_Color.Red >= (Org_Color.Red / 20)) && (Curr_Color.Grn >= (Org_Color.Grn / 20)) && (Curr_Color.Blu >= (Org_Color.Blu / 20)))
    {
        if(Curr_Color.Red || Curr_Color.Grn || Curr_Color.Blu)
        {
            Curr_Color.Red = Curr_Color.Red * 0.99;
            Curr_Color.Grn = Curr_Color.Grn * 0.99;
            Curr_Color.Blu = Curr_Color.Blu * 0.99;
            
            SetColor();
        }
    }
}

/**
    Find two peaks within 3/4 of a second of each other.
*/
void SenseClaps(void)
{
    // Search for a peak
    FindPeak(LOW_THRESH, FIRST_HIGH_THRESH);

    // Wait for at least 30ms to ensure the original signal isn't re-sampled
    TMR2 = 0u;
    while(TMR2 < (75 * T2_ONE_MS));
    Timer_State = SAME_PERIOD;

    // Search for a peak
    FindPeak(FIRST_HIGH_THRESH, LOW_THRESH);

    // If another peak is found within the same time period, increment the clap count
    if(SAME_PERIOD == Timer_State)
    {
        Clap_Count++;
    }
}

/*
    Find a voltage peak. This signifies a beat or a clap.

    @param low_tresh All voltages below this are considered lows.
    @param high_thresh All voltages above this are considered peaks.
*/
void FindPeak(uint16_t low_thresh, uint16_t high_thresh)
{
    uint16_t adc_val = 0u;

    // Wait for a high
    do{
        // Sample until a measurement of interest occurs
        adc_val = Peripherals_ADC_Convert();
    } while(adc_val < high_thresh);

    // Wait for a low
    do{
        adc_val = Peripherals_ADC_Convert();
    } while(adc_val > low_thresh);
}

/*
    Sets the color of the LED strip depending on the size of the peak.
*/
void ChooseColor(uint16_t beat_size)
{
    COLOR_t color;
    uint16_t shade;
    
    if(beat_size > Max_Comp_Peak)
    {
        color = RED_BLU_GRN;
        shade = 0u;
    }
    else
    {
        uint8_t i = 8u;
        uint16_t diff = Max_Comp_Peak - beat_size;
        
        for(i = 8u; i >= 3u; i--)
        {
            if(diff < (Max_Comp_Peak / i))
            {
                color = i - 3u;
                diff = (Max_Comp_Peak / i) - diff;
                // Break out of the loop
                i = 0u;
            }
        }
        
        // The shade will be used to set the duty cycle. Always between 0 and 100.
        shade = (diff * 101) / ((Max_Comp_Peak / (color + 3u)) - (Max_Comp_Peak / (color + 4u)));
        // Set color based on beat_size
    }
    
    switch(color)
    {
        // Used when color is greater than 7 (Voltage spike is greater than MAX_PEAK)
        default:
            color = RED_BLU_GRN;
            break;
        case RED:
            Org_Color.Red = shade;
            Org_Color.Grn = 0u;
            Org_Color.Blu = 0u;
            break;
        case GRN:
            Org_Color.Red = 0u;
            Org_Color.Grn = shade;
            Org_Color.Blu = 0u;
            break;
        case BLU:
            Org_Color.Red = 0u;
            Org_Color.Grn = 0u;
            Org_Color.Blu = shade;
            break;
        case RED_GRN:
            Org_Color.Red = 100u;
            Org_Color.Grn = shade;
            Org_Color.Blu = 0u;
            break;
        case RED_BLU:
            Org_Color.Red = 100u;
            Org_Color.Grn = 0u;
            Org_Color.Blu = shade;
            break;
        case BLU_GRN:
            Org_Color.Red = 0u;
            Org_Color.Grn = shade;
            Org_Color.Blu = 100u;
            break;
        case RED_BLU_GRN:
            Org_Color.Red = 100u;
            Org_Color.Grn = 100u;
            Org_Color.Blu = 100u;
            break;
    }
    
    Curr_Color.Red = Org_Color.Red;
    Curr_Color.Grn = Org_Color.Grn;
    Curr_Color.Blu = Org_Color.Blu;
    
    SetColor();
}

void SetColor(void)
{
    Peripherals_PWM_SetDC(RED_CHANNEL, Curr_Color.Red);
    Peripherals_PWM_SetDC(GRN_CHANNEL, Curr_Color.Grn);
    Peripherals_PWM_SetDC(BLU_CHANNEL, Curr_Color.Blu);
}