#include "adcHelper.h"
#include "messages.h"
#include "maindefs.h"

void adcInit() {

    TRISA = 0x2F;	// set RA3-RA0 and RA5 to inputs (AN0-AN4)
//    TRISB = 0x04;       // set RB2 to input (AN8)
    
    //Configure pins 2,3,4,5,7,23 as analog inputs
    ANCON0bits.PCFG0 = 0; // ADC0, PIN2
    ANCON0bits.PCFG1 = 0; // ADC1, PIN3
    ANCON0bits.PCFG2 = 0; // ADC2, PIN4
    ANCON0bits.PCFG3 = 0; // ADC3, PIN5
    ANCON0bits.PCFG4 = 0; // ADC4, PIN7
//    ANCON1bits.PCFG8 = 0; // ADC8, PIN23

    //Set voltage reference (0 = Vss/Vdd, 1 = Vref+/-)
    ADCON0bits.VCFG0 = 0;
    ADCON0bits.VCFG1 = 0;

    // Have reading right justified
    ADCON1bits.ADFM = 1;

    // Start with AN0 to read, incriment later
    ADCON0bits.CHS = 0;

    //Set acquisition time
    ADCON1bits.ACQT = 0x7;

    //Set conversion time
    ADCON1bits.ADCS = 0x1;
    
    //Enable ADC module
    ADCON0bits.ADON = 1;

    ADCON0bits.GODONE = 1;
    PIR1bits.ADIF = 0;
    PIE1bits.ADIE = 1;
}

void adcReadyNextRead() {
    // Start ADC
    ADCON0bits.GODONE = 1;
}

void adcIntHandler() {
    unsigned char msg[3];
    
    //ADC output 
    msg[1] = ADRESH;
    msg[2] = ADRESL;
    
    // Loop through ADC pins (5-7 are unimplimented)
    if(ADCON0bits.CHS == 0x4) {
        ADCON0bits.CHS = 0x0;
        msg[0] = 5;
    }
//    else if (ADCON0bits.CHS == 0x8) {
//        ADCON0bits.CHS = 0x0;
//        msg[0] = 6;
//    }
    else {
        ++ADCON0bits.CHS;
        msg[0] = ADCON0bits.CHS;
    }

    ToMainLow_sendmsg(3, MSG_ADC_DATA, msg);
}