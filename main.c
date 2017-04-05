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
#define CLAP_THRESH  (ADC_ONE_VOLT)
/* 0.5 Volts */
#define LOW_THRESH  (ADC_QTR_VOLT)

#define BEAT_BUFF_LENGTH ((uint16_t)300u)
/** Voltage spikes must be at least 1.5V to be considered a beat. */
#define BEAT_THRESH (ADC_ONE_VOLT + ADC_QTR_VOLT)

/** All peaks greater than this set the LED color to white */
#define MAX_PEAK        ((uint16_t)0xC00u)
/** ADC read range per color of the LED */
#define COLOR_RANGE     ((uint16_t)347u)

#define PEAKS_PER_PERIOD    ((uint8_t)5u)
#define NUM_STORED_PERIODS  ((uint8_t)56u)
#define NUM_STORED_PEAKS    ((uint8_t)3u)

/** PWM channel of the RED LEDs */
#define RED_CHANNEL     ((uint8_t)3u)
/** PWM channel of GRN LEDs */
#define GRN_CHANNEL     ((uint8_t)2u)
/** PWM channel of BLU LEDs */
#define BLU_CHANNEL     ((uint8_t)1u)
/** Amount of time between fades of the LEDs */
#define FADE_WAIT       ((uint16_t)(T2_ONE_MS * 15u))

#define BTN_POW_TRIS    (TRISAbits.TRISA1)
#define BTN_POW_I       (PORTAbits.RA1)
#define BTN_POW_O       (LATAbits.LATA1)

#define AUX_POW_TRIS    (TRISAbits.TRISA4)
#define AUX_POW_I       (PORTAbits.RA4)
#define AUX_POW_O       (LATAbits.LATA4)

#define MIC_POW_TRIS    (TRISBbits.TRISB4)
#define MIC_POW_I       (PORTBbits.RB4)
#define MIC_POW_O       (LATBbits.LATB4)

/* Function prototypes */
void FindPeak(uint16_t low_thresh, uint16_t high_thresh);
void SenseClaps(void);
void ProcessPeriod(void);
void SenseBeats(void);
void SetColor(void);
void ChooseColor(void);
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

void __attribute__((__interrupt__, no_auto_psv)) _T1Interrupt(void);
void __attribute__((__interrupt__, no_auto_psv)) _T2Interrupt(void);

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
uint8_t Curr_Period = 0u;
uint8_t Peak_Count = 0u;
/** Just for debugging, REMOVE */
uint16_t Org_Adc_Val = 0u;
/** Number used to determine the color and state of the RGB LEDs */
uint16_t Led_Count = 0u;
/** The three largest peaks within the last 21 seconds 
      VAL    PERIOD 
    |  X    |  X   |
    |  X    |  X   |
    |  X    |  X   |
 */
uint16_t Max_Peaks[NUM_STORED_PEAKS][2u];

/** Flag used to restart learning when the song stops playing. */
uint8_t Song_Off = 1u;
uint16_t Peaks[NUM_STORED_PERIODS][PEAKS_PER_PERIOD];
/*
 *
 */
int main(int argc, char** argv) {

    OSC_Init();
    AD1PCFGL = 0x00FEu; // Disable analog functionality on pins 3 and 4

    Timers_T1_Setup(T2_ONE_MS * 10u);
    Timers_T2_Setup(THREE_QUARTERS_SEC_PERIOD);
    Peripherals_ADC_Init();
    Peripherals_PWM_Init();

    // Disable interrupt nesting
    INTCON1bits.NSTDIS = 1u;

    Timers_T2_SetInt();

    BTN_POW_TRIS = 0u;      // Set RA1 to output
    BTN_POW_O = 1u;         // Set RA1 pin to HIGH
    
    // Inverse logic to turn on each module
    MIC_POW_TRIS = 0u;
    MIC_POW_O = 0u;
    
    AUX_POW_TRIS = 0u;
    AUX_POW_O = 1u;

    Peripherals_EnableInterrupts();
    Timers_Start();

    Curr_Color.Red = 100;
    Curr_Color.Grn = 0u;
    Curr_Color.Blu = 0;
    
    Peripherals_PWM_SetDC(RED_CHANNEL, Curr_Color.Red);
    Peripherals_PWM_SetDC(GRN_CHANNEL, Curr_Color.Grn);
    Peripherals_PWM_SetDC(BLU_CHANNEL, Curr_Color.Blu);

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
            ProcessPeriod();
        }
    }
    return (0u);
}
void __attribute__((__interrupt__, no_auto_psv)) _T1Interrupt(void)
{
    Led_Count++;
    if(Led_Count > 1050)
    {
        Led_Count = 0u;
    }

    IFS0bits.T1IF = 0u;        // Clear flag
}

void __attribute__((__interrupt__, no_auto_psv)) _T2Interrupt(void)
{
    if((Clap_Count >= 1u) && (Current_State == CLAP_STATE))
    {
        Current_State = BEAT_STATE;
        Timers_T1_SetInt();
        BTN_POW_O = 0u;     // Turn off RA1
        MIC_POW_O = 1u;
        AUX_POW_O = 0u;
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

void ProcessPeriod(void)
{
    uint8_t i = 0u;
    uint8_t flag = 0u;
    uint16_t max_peak = 0u;
    uint8_t period_clear = Curr_Period + 1u;
    
    // Increment every period. When it reaches 10, learning restarts.
    Song_Off++;

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

    // If a peak is being replaced, find new minimum max peak
    if(flag == 1u)
    {
        Max_Peaks[oldest_period][0u] = max_peak;
        Max_Peaks[oldest_period][1u] = Curr_Period;
            
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
    
    // If the song has been off for longer than 2.25 seconds, clear the peaks.
    if(Song_Off >= 3u)
    {
        for(i = 0; i < NUM_STORED_PEAKS; i++)
        {
            Max_Peaks[i][0u] = 0u;
            Max_Peaks[i][1u] = 0u;
        }
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
    BTN_POW_O ^= 1u;
    if((adc_val > (Max_Comp_Peak - (Max_Comp_Peak / 30))) &&
        adc_val > BEAT_THRESH &&
        adc_val < (ADC_TWO_VOLT + ADC_HALF_VOLT))  // Max peak that will be seen. Peaks that are greater are disregarded.
    {
        Org_Adc_Val = adc_val;
        // Find top of peak by waiting until the adc reading is less than the previous one.
        uint16_t prev_adc_val;
        do
        {
            prev_adc_val = adc_val;
            adc_val = Peripherals_ADC_Convert();
        } while((prev_adc_val < adc_val) || 
                ((Org_Adc_Val > prev_adc_val) && 
                ((Org_Adc_Val - prev_adc_val) < ADC_QTR_VOLT) && 
                (prev_adc_val - adc_val < ADC_EIGHTH_VOLT)));
        
        if((Org_Adc_Val > prev_adc_val) && (Org_Adc_Val - prev_adc_val < ADC_QTR_VOLT))
        {
            prev_adc_val = Org_Adc_Val;
        }
        // Set to the highest found value
        Peaks[Curr_Period][Peak_Count] = prev_adc_val;
        Peak_Count++;
        
        Led_Count += 100u;
        ChooseColor();

        // Wait for at least 150ms to ensure the original signal isn't re-sampled
        uint16_t timer_val = TMR2;
        TMR2 = 0u;
        while(TMR2 < (150 * T2_ONE_MS));
        
        if(((timer_val + 150 * T2_ONE_MS) > THREE_QUARTERS_SEC_PERIOD) || (Peak_Count == PEAKS_PER_PERIOD))
        {
            IFS0bits.T2IF = 1u;  // Set interrupt flag as the timer would have triggered an interrupt
        }
        else
        {
            // Set timer to what it would have been.
            TMR2 = timer_val + (150 * T2_ONE_MS);
        }
    }
    else if((adc_val > (ADC_HALF_VOLT + ADC_QTR_VOLT)) && 
            (adc_val < (ADC_TWO_VOLT + ADC_HALF_VOLT)))
    {
        Song_Off = 0u;
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
    if((Curr_Color.Red >= (Org_Color.Red / 30)) && (Curr_Color.Grn >= (Org_Color.Grn / 30)) && (Curr_Color.Blu >= (Org_Color.Blu / 30)))
    {
        if(Curr_Color.Red || Curr_Color.Grn || Curr_Color.Blu)
        {
            Curr_Color.Red = Curr_Color.Red * 0.98;
            Curr_Color.Grn = Curr_Color.Grn * 0.98;
            Curr_Color.Blu = Curr_Color.Blu * 0.98;
            
            SetColor();
        }
    }
}

/**
    Find two peaks within 3/4 of a second of each other.
*/
void SenseClaps(void)
{
    Timer_State = SAME_PERIOD;
    TMR2 = 0u;
    // Search for a peak
    FindPeak(LOW_THRESH, CLAP_THRESH);

    // Wait for at least 30ms to ensure the original signal isn't re-sampled
    TMR2 = 0u;
    while(TMR2 < (30 * T2_ONE_MS));
    TMR2 = 0u;

    // Search for a peak
    FindPeak(LOW_THRESH, CLAP_THRESH);

    // If another peak is found within the same time period, increment the clap count
    if(SAME_PERIOD == Timer_State)
    {
        Clap_Count++;
    }
}

/*
    Find a voltage peak. This signifies a beat or a clap.

    @param low_thresh All voltages below this are considered lows.
    @param high_thresh All voltages above this are considered peaks.
*/
void FindPeak(uint16_t low_thresh, uint16_t high_thresh)
{
    uint16_t adc_val = 0u;

    // Wait for a high.
    // If in beat state or a T2 interrupt has occurred, exit.
    do{
        // Sample until a measurement of interest occurs
        adc_val = Peripherals_ADC_Convert();
    } while((adc_val < high_thresh) && (Current_State == CLAP_STATE) && 
            (Timer_State == SAME_PERIOD));

    // Wait for a low
    do{
        adc_val = Peripherals_ADC_Convert();
    } while((adc_val > low_thresh) && (Current_State == CLAP_STATE) &&
            (Timer_State == SAME_PERIOD));
}

/*
    Sets the color of the LED strip depending on the size of the peak.
*/
void ChooseColor(void)
{    
    if(Led_Count < 100u)
    {
        Org_Color.Red = 100u;
        Org_Color.Grn = Led_Count;
        Org_Color.Blu = 0u;
    }
    else if(Led_Count < 200u)
    {
        Org_Color.Red = 100u - (Led_Count - 100u);
        Org_Color.Grn = 100u;
        Org_Color.Blu = 0u;
    }
    else if(Led_Count < 300)
    {
        Org_Color.Red = 0u;
        Org_Color.Grn = 100u;
        Org_Color.Blu = Led_Count - 200u;
    }
    else if(Led_Count < 400)
    {
        Org_Color.Red = 0u;
        Org_Color.Grn = 100u - (Led_Count - 300u);
        Org_Color.Blu = 100u;
    }
    else if(Led_Count < 500)
    {
        Org_Color.Red = Led_Count - 400u;
        Org_Color.Grn = 0u;
        Org_Color.Blu = 100u;
    }
    else if(Led_Count < 600)
    {
        Org_Color.Red = 100;
        Org_Color.Grn = 0u;
        Org_Color.Blu = 100u - (Led_Count - 500u);
    }
    else if(Led_Count < 700)
    {
        Org_Color.Red = 100 - (Led_Count - 600u);
        Org_Color.Grn = 100u;
        Org_Color.Blu = Led_Count - 600u;
    }
    else if(Led_Count < 800)
    {
        Org_Color.Red = Led_Count - 700u;
        Org_Color.Grn = 100u - Org_Color.Red;
        Org_Color.Blu = 100;
    }
    else if(Led_Count < 900)
    {
        Org_Color.Red = 100;
        Org_Color.Grn = Led_Count - 800u;
        Org_Color.Blu = 100u - Org_Color.Grn;
    }
    else if(Led_Count < 1000)
    {
        Org_Color.Red = 100u;
        Org_Color.Grn = 100u;
        Org_Color.Blu = Led_Count - 900u;
    }
    else
    {
        Org_Color.Red = 100u;
        Org_Color.Grn = 100u;
        Org_Color.Blu = 100u;
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