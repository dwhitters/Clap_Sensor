/**
    @author David Whitters
    @date 1/24/17
    @title Peripherals.c
    @description This file contains the functions necessary to set up and control PWM, ADC, and Input Capture.
*/

#include "Peripherals.h"

void Peripherals_ADC_Init(void)
{
    AD1CSSL=0x0001;                 //select AN0 for analog input

    AD1CHS0 = 0;                    // channel input is AN0
    AD1CON1bits.ASAM = 1;           // SAMP bit auto set
    AD1CON1bits.SSRC = 7;           // auto sampling and convert
    AD1CON1bits.AD12B = 1;          // 12bit 1 channel ADC

    // No channel scan for CH0+, Use MUX A,  SMPI = 1 per interrupt, Vref =AVdd/AVss
    AD1CON2 = 0x0000;
    AD1CON3bits.SAMC=0x3;           // auto sample time
    AD1CON3bits.ADCS=0x1f;          // conversion clock select
    IFS0bits.AD1IF = 0;             // reset ADC interrupt flag
}

uint16_t Peripherals_ADC_Convert(void)
{
    AD1CON1bits.ADON = 1;                // turn on ADC module
    while(IFS0bits.AD1IF == 0);          // wait for conversion
    AD1CON1bits.ADON = 0;                // turn off ADC module
    IFS0bits.AD1IF = 0;                  // reset ADC interrupt flag
    return ADCBUF0;
}

void Peripherals_IC2_Init(void)
{
    RPINR7bits.IC2R = 0u;       // Tie IC2R peripheral to RP0

    IC2CONbits.ICM = 0u;        // Disable Input Capture 2 module
    IC2CONbits.ICTMR = 1u;      // Select Timer2 as the IC2 Time base
    IC2CONbits.ICI = 0u;        // Interrupt every capture event
    IC2CONbits.ICM = 0b011u;    // Generate capture event on every rising edge
}

void Peripherals_IC2_SetInt(void)
{
    IPC1bits.IC2IP = 6u;        // Setup IC2 priority level
    IFS0bits.IC2IF = 0u;        // Clear IC2 interrupt flag
    IEC0bits.IC2IE = 1u;        // Enable IC2 interrupt
}

void Peripherals_PWM_Init(void)
{
    P1TCONbits.PTSIDL = 0u; // PWM runs in idle mode
    P1TCONbits.PTOPS = 0u;  // PWM Postscale 1:1
    P1TCONbits.PTCKPS = 0u; // PWM Prescale 1:1
    P1TCONbits.PTMOD = 0u;  // PWM operates in Free-running mode

    P1TMRbits.PTDIR = 0u;   // PWM time base is counting up

    P1TPERbits.PTPER = 999u;    // Create a 40KHz frequency

    PWM1CON1bits.PMOD1 = 1u;    // PWM I/O pair is in independent PWM Output mode
    PWM1CON1bits.PMOD2 = 1u;
    PWM1CON1bits.PMOD3 = 1u;
    PWM1CON1bits.PEN1H = 1u;    // PWM1H is enabled for PWM output
    PWM1CON1bits.PEN2H = 1u;
    PWM1CON1bits.PEN3H = 1u;

    PWM1CON2bits.IUE = 0u;      // Updates to the active P1DC registers are synchronized
    PWM1CON2bits.OSYNC = 0u;    // Output overrides occur on next Tcy boundary.
    PWM1CON2bits.UDIS = 0u;     // Updates from the Duty cycle and period buffer
                                // registers are enabled.

    P1OVDCONbits.POUT1H = 1u;   // PWM I/O pin controlled by PWM generator
    P1OVDCONbits.POUT2H = 1u;
    P1OVDCONbits.POUT3H = 1u;

    P1DC1 = 0u;              // Set duty cycle to zero initially
    P1DC2 = 0u;
    P1DC3 = 0u;

    P1TCONbits.PTEN = 1u;   // Enable PWM time base
}

/**
    Sets the appropriate duty cycle register to the correct value based on the percent duty cycle
    passed in.

    @param channel The PWM channel to apply the new duty cycle to.
    @duty_cycle Varies from 0 to 100, the percent duty_cycle that the PWM signal should have.
*/
void Peripherals_PWM_SetDC(uint8_t channel, uint8_t duty_cycle)
{
    uint16_t duty_cycle_val;
    
    if(0u == duty_cycle)
    {
        duty_cycle_val = 0u;
    }
    else
    {
        // Duty cycle register set to up to double the period. Two added to smooth max PWM.
        duty_cycle_val = (duty_cycle * ((2u * P1TPERbits.PTPER) / 100u)) + 2u;
    }
    
    switch(channel)
    {
        case 1:
            P1DC1 = duty_cycle_val;
            break;
        case 2:
            P1DC2 = duty_cycle_val;
            break;
        case 3:
            P1DC3 = duty_cycle_val;
            break;
        default:
            break;
    }
}

void Peripherals_EnableInterrupts(void)
{
    /* Set CPU IPL to 0, enable level 1-7 interrupts */
    /* No saving of current CPU IPL setting performed here */
    SET_CPU_IPL(0u);
}

void Peripherals_DisableInterrupts(void)
{
    /* Set CPU IPL to 7, disable level 1-7 interrupts */
    /* No saving of current CPU IPL setting performed here */
    SET_CPU_IPL(7u);
}
