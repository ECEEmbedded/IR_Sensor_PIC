#include "adcHelper.h"
#include "maindefs.h"
#include <stdio.h>
#include <string.h>
#ifndef __XC8
#include <usart.h>
#include <i2c.h>
#include <timers.h>
#else
//#include <plib/usart.h>
#include <plib/i2c.h>
#include <plib/timers.h>
#endif
#include "interrupts.h"
#include "messages.h"
#include "DebugOut.h"
//#include "my_uart.h"
#include "my_i2c.h"
//#include "uart_thread.h"
//#include "timer1_thread.h"
//#include "timer0_thread.h"

#define DEBUG

#ifdef __USE18F26J50

// PIC18F26J50 Configuration Bit Settings

// CONFIG1L
#pragma config WDTEN = OFF      // Watchdog Timer (Disabled - Controlled by SWDTEN bit)
#pragma config PLLDIV = 3       // PLL Prescaler Selection bits (Divide by 3 (12 MHz oscillator input))
#pragma config STVREN = OFF     // Stack Overflow/Underflow Reset  (Disabled)
#pragma config XINST = OFF       // Extended Instruction Set (Enabled)

// CONFIG1H
#pragma config CPUDIV = OSC1    // CPU System Clock Postscaler (No CPU system clock divide)
#pragma config CP0 = OFF        // Code Protect (Program memory is not code-protected)

// CONFIG2L
#pragma config OSC = HSPLL      // Oscillator (HS+PLL, USB-HS+PLL)
#pragma config T1DIG = OFF      // T1OSCEN Enforcement (Secondary Oscillator clock source may not be selected)
#pragma config LPT1OSC = OFF    // Low-Power Timer1 Oscillator (High-power operation)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor (Disabled)
#pragma config IESO = ON        // Internal External Oscillator Switch Over Mode (Enabled)

// CONFIG2H
#pragma config WDTPS = 32768    // Watchdog Postscaler (1:32768)

// CONFIG3L
#pragma config DSWDTOSC = T1OSCREF// DSWDT Clock Select (DSWDT uses T1OSC/T1CKI)
#pragma config RTCOSC = T1OSCREF// RTCC Clock Select (RTCC uses T1OSC/T1CKI)
#pragma config DSBOREN = OFF    // Deep Sleep BOR (Disabled)
#pragma config DSWDTEN = OFF    // Deep Sleep Watchdog Timer (Disabled)
#pragma config DSWDTPS = G2     // Deep Sleep Watchdog Postscaler (1:2,147,483,648 (25.7 days))

// CONFIG3H
#pragma config IOL1WAY = ON     // IOLOCK One-Way Set Enable bit (The IOLOCK bit (PPSCON<0>) can be set once)
#pragma config MSSP7B_EN = MSK7 // MSSP address masking (7 Bit address masking mode)

// CONFIG4L
#pragma config WPFP = PAGE_63   // Write/Erase Protect Page Start/End Location (Write Protect Program Flash Page 63)
#pragma config WPEND = PAGE_WPFP// Write/Erase Protect Region Select (valid when WPDIS = 0) (Page WPFP<5:0> through Configuration Words erase/write protected)
#pragma config WPCFG = OFF      // Write/Erase Protect Configuration Region (Configuration Words page not erase/write-protected)

// CONFIG4H
#pragma config WPDIS = OFF      // Write Protect Disable bit (WPFP<5:0>/WPEND region ignored)
#else
Something is messed up
#endif

void main(void) {
    char c;
    signed char length;
    unsigned char msgtype;
    unsigned char last_reg_recvd;
//    uart_comm uc;
    i2c_comm ic;
    unsigned char sensor_reading_count = 0;
    unsigned char msgbuffer[MSGLEN + 1];
    unsigned char i;
//    uart_thread_struct uthread_data; // info for uart_lthread
//    timer1_thread_struct t1thread_data; // info for timer1_lthread
//    timer0_thread_struct t0thread_data; // info for timer0_lthread

#ifdef __USE18F26J50
    OSCCON = 0xE0; // see datasheeet
    OSCTUNEbits.PLLEN = 1;
#else
    Something is wrong
#endif

    TRISC = 0x0;
    LATC = 0x0;

    // initialize the i2c code
    init_i2c(&ic);

    // init the timer1 lthread
//    init_timer1_lthread(&t1thread_data);

    //Setup ad
    adcInit();

    // initialize message queues before enabling any interrupts
    init_queues();

    // initialize Timers
    OpenTimer0(TIMER_INT_ON & T0_16BIT & T0_SOURCE_INT & T0_PS_1_128);
#ifdef __USE18F26J50
    // MTJ added second argument for OpenTimer1()
    OpenTimer1(TIMER_INT_ON & T1_SOURCE_FOSC_4 & T1_PS_1_4 & T1_16BIT_RW & T1_OSC1EN_OFF & T1_SYNC_EXT_OFF,0x0);
#else
    OpenTimer1(TIMER_INT_ON & T1_PS_1_8 & T1_16BIT_RW & T1_SOURCE_INT & T1_OSC1EN_OFF & T1_SYNC_EXT_OFF);
#endif

    // Decide on the priority of the enabled peripheral interrupts
    // 0 is low, 1 is high
    // Timer1 interrupt
    IPR1bits.TMR1IP = 0;
    // USART RX interrupt
    IPR1bits.RCIP = 0;
    // USART TX interrupt
    IPR1bits.TXIP = 0;
    // I2C interrupt
    IPR1bits.SSPIP = 1;

    // configure the hardware i2c device as a slave (0x9E -> 0x4F) or (0x9A -> 0x4D)
    i2c_configure_slave(0x9E);

    // must specifically enable the I2C interrupts
    PIE1bits.SSPIE = 1;

    // Peripheral interrupts can have their priority set to high or low
    // enable high-priority interrupts and low-priority interrupts
    enable_interrupts();

    // loop forever
    // This loop is responsible for "handing off" messages to the subroutines
    // that should get them.  Although the subroutines are not threads, but
    // they can be equated with the tasks in your task diagram if you
    // structure them properly.

    while (1) {
        // Call a routine that blocks until either on the incoming
        // messages queues has a message (this may put the processor into
        // an idle mode)
        block_on_To_msgqueues();

        // At this point, one or both of the queues has a message.  It
        // makes sense to check the high-priority messages first -- in fact,
        // you may only want to check the low-priority messages when there
        // is not a high priority message.  That is a design decision and
        // I haven't done it here.
        length = ToMainHigh_recvmsg(MSGLEN, &msgtype, (void *) msgbuffer);
        if (length < 0) {
            // no message, check the error code to see if it is concern
            if (length != MSGQUEUE_EMPTY) {
                // This case be handled by your code.
            }
        } else {
            switch (msgtype) {
                case MSGT_TIMER0:
                {
                    //timer0_lthread(&t0thread_data, msgtype, length, msgbuffer);
                    break;
                };
                case MSGT_I2C_DATA:
                {
                  unsigned char a = msgbuffer[4];
                  break;
                }
                case MSGT_I2C_DBG:
                {
                    // Here is where you could handle debugging, if you wanted
                    // keep track of the first byte received for later use (if desired)
                    last_reg_recvd = msgbuffer[0];
                    break;
                };
                case MSGT_I2C_RQST:
                default:
                {
                    // Your code should handle this error
                    break;
                };
            };
        }

        // Check the low priority queue
        length = ToMainLow_recvmsg(MSGLEN, &msgtype, (void *) msgbuffer);
        if (length < 0) {
            // no message, check the error code to see if it is concern
            if (length != MSGQUEUE_EMPTY) {
                // Your code should handle this situation
            }
        } else {
            switch (msgtype) {
                case MSGT_TIMER1:
                {
                    //timer1_lthread(&t1thread_data, msgtype, length, msgbuffer);
                    adcReadyNextRead();
                    break;
                };
                case MSG_ADC_DATA:
                {
                    unsigned short ADCreading = (msgbuffer[1] << 8) | msgbuffer[2];
//#ifdef DEBUG
//                    if (msgbuffer[0] <= 4 ){
//                        dbgPrintByte(msgbuffer[1]);
//                        dbgPrintByte(msgbuffer[2]);
//                    }
//#endif
                    ADCreading = 96563 / ADCreading - 16; // Convert V to Range
//#ifdef DEBUG
//                    if (msgbuffer[0] == 1 ){
//                        dbgPrintByte((char)(ADCreading >> 8));
//                        dbgPrintByte((char)(ADCreading & 0xFF));
//                    }
//#endif
                    msgbuffer[1] = msgbuffer[0]; // sensor #
                    msgbuffer[0] = 0x4F; // message data
                    msgbuffer[3] = sensor_reading_count++; // count
                    msgbuffer[4] = (char)(ADCreading >> 8);
                    msgbuffer[5] = (char)(ADCreading & 0x00FF);
                    msgbuffer[6] = 0;
                    msgbuffer[7] = 0;
                    msgbuffer[2] = msgbuffer[4] ^ msgbuffer[5] ^ msgbuffer[6] ^ msgbuffer[7]; // parity.

                    FromMainLow_sendmsg(8,MSG_ADC_DATA,msgbuffer);
                };
                case MSGT_OVERRUN:
                case MSGT_UART_DATA:
                default:
                {
                    // Your code should handle this error
                    break;
                };
            };
        }
    }
}