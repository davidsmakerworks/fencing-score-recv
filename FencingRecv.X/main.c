/*
 * Remote Fencing Score Display - Receiver
 * Copyright (c) 2019 David Rice
 *
 * Designed for PIC16F18323 or PIC16F18325
 * 
 * Interfaces with nRF24L01+ 2.4 GHz RF module for transmission
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

// PIC16F18323 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1
#pragma config FEXTOSC = OFF    // FEXTOSC External Oscillator mode Selection bits (Oscillator not enabled)
#pragma config RSTOSC = HFINT1  // Power-up default value for COSC bits (HFINTOSC (1MHz))
#pragma config CLKOUTEN = OFF   // Clock Out Enable bit (CLKOUT function is disabled; I/O or oscillator function on OSC2)
#pragma config CSWEN = ON       // Clock Switch Enable bit (Writing to NOSC and NDIV is allowed)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is disabled)

// CONFIG2
#pragma config MCLRE = ON       // Master Clear Enable bit (MCLR/VPP pin function is MCLR; Weak pull-up enabled)
#pragma config PWRTE = ON       // Power-up Timer Enable bit (PWRT enabled)
#pragma config WDTE = OFF       // Watchdog Timer Enable bits (WDT disabled; SWDTEN is ignored)
#pragma config LPBOREN = OFF    // Low-power BOR enable bit (ULPBOR disabled)
#pragma config BOREN = ON       // Brown-out Reset Enable bits (Brown-out Reset enabled, SBOREN bit ignored)
#pragma config BORV = LOW       // Brown-out Reset Voltage selection bit (Brown-out voltage (Vbor) set to 2.45V)
#pragma config PPS1WAY = ON     // PPSLOCK bit One-Way Set Enable bit (The PPSLOCK bit can be cleared and set only once; PPS registers remain locked after one clear/set cycle)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable bit (Stack Overflow or Underflow will cause a Reset)
#pragma config DEBUG = OFF      // Debugger enable bit (Background debugger disabled)

// CONFIG3
#pragma config WRT = OFF        // User NVM self-write protection bits (Write protection off)
#pragma config LVP = ON         // Low Voltage Programming Enable bit (Low Voltage programming enabled. MCLR/VPP pin function is MCLR. MCLRE configuration bit is ignored.)

// CONFIG4
#pragma config CP = OFF         // User NVM Program Memory Code Protection bit (User NVM code protection disabled)
#pragma config CPD = OFF        // Data NVM Memory Code Protection bit (Data NVM code protection disabled)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

#include "nRF24L01P.h"
#include "nRF24L01P-cfg.h"

/* ===== CONFIGURABLE OPTIONS START HERE ===== */
#define RF_CHANNEL      0x0CU
#define RF_RX_ADDR      strip2_addr
/* ====== CONFIGURABLE OPTIONS END HERE ====== */

#define _XTAL_FREQ      4000000

#define ADDR_LEN        5
#define PAYLOAD_WIDTH   1

#define LEFT_RED_LED    LATAbits.LATA0
#define RIGHT_GREEN_LED LATAbits.LATA1
#define LEFT_WHITE_LED  LATAbits.LATA2
#define RIGHT_WHITE_LED LATAbits.LATA4

#define LEFT_RED_MASK       0b00110000
#define RIGHT_GREEN_MASK    0b00001100
#define LEFT_WHITE_MASK     0b11000000
#define RIGHT_WHITE_MASK    0b00000011

const uint8_t strip1_addr[ADDR_LEN] = { 0x44, 0x4D, 0x57, 0x31, 0xAA };
const uint8_t strip2_addr[ADDR_LEN] = { 0x44, 0x4D, 0x57, 0x31, 0xBB };
const uint8_t mask1_addr[ADDR_LEN] = { 0x44, 0x4D, 0x57, 0x31, 0xCC };
const uint8_t mask2_addr[ADDR_LEN] = { 0x44, 0x4D, 0x57, 0x31, 0xDD };

volatile bool data_pending = false;

void __interrupt() isr(void) {
    /* Do NOT check PEIE in ISR because TMR0 and INT are not gated by PEIE */
    
    /* If external interrupt fires, data has been received by the RF module */
    if (PIE0bits.INTE && PIR0bits.INTF) {
        PIR0bits.INTF = 0;
        
        data_pending = true;
    }
    
    /* If timer interrupt fires, turn off all lights and stop timer */
    if (PIE0bits.TMR0IE && PIR0bits.TMR0IF) {
        PIR0bits.TMR0IF = 0;
        
        LEFT_RED_LED = 0;
        RIGHT_GREEN_LED = 0;
        LEFT_WHITE_LED = 0;
        RIGHT_WHITE_LED = 0;

        T0CON0bits.T0EN = 0;
    }
}

uint8_t transfer_spi(uint8_t data) {
    SSP1BUF = data;
    
    while (!SSP1STATbits.BF);
    
    data = SSP1BUF;
    
    return data;
}

void init_ports(void) {
    /* Disable all analog features */
    ANSELA = 0x00;
    ANSELC = 0x00;
    
    /* Initialize all ports as outputs except RC1 (SDI) and RC5 (RF_IRQ) */
    TRISA = 0x00;
    TRISC = _TRISC_TRISC1_MASK | _TRISC_TRISC5_MASK;
    
    /* Pull all outputs low except RF_CSN */
    LATA = 0x00;
    LATC = _LATC_LATC3_MASK;
    
    /* Set TTL input level on RC1 (SDI1) and RC5 (RF_IRQ) */
    INLVLCbits.INLVLC1 = 0;
    INLVLCbits.INLVLC5 = 0;
}

void init_osc(void) { 
    /* Set frequency of HFINTOSC to 4 MHz */
    OSCFRQbits.HFFRQ = 0b0011;
    
    /* Set clock divider to 1:1 to achieve Fosc of 4 MHz */
    OSCCON1bits.NDIV = 0b0000;
}

void init_spi(void) {
    /* Set MSSP1 to SPI Master mode, clock = Fosc / 4 = 1 MHz at Fosc = 4 MHz */
    SSP1CON1bits.SSPM = 0b0000;
    
    /* Transmit data on active-to-idle transition */
    SSP1STATbits.CKE = 1;
    
    /* Enable MSSP1 */
    SSP1CON1bits.SSPEN = 1;
}

void init_pps(void) {
    bool state;
    
    /* Preserve global interrupt state and disable interrupts */
    state = INTCONbits.GIE;
    INTCONbits.GIE = 0;
    
    /* Unlock PPS */
    PPSLOCK = 0x55;
    PPSLOCK = 0xAA;
    PPSLOCKbits.PPSLOCKED = 0;
    
    /* SCK1 on RC0 */
    RC0PPS = 0b11000;
    
    /* SDI1 on RC1 */
    SSP1DATPPS = 0b10001;
    
    /* SDO1 on RC2 */
    RC2PPS = 0b11001;
    
    /* INT on RC5 */
    INTPPS = 0b10101;
    
    /* Lock PPS */
    PPSLOCK = 0x55;
    PPSLOCK = 0xAA;
    PPSLOCKbits.PPSLOCKED = 1;
    
    /* Restore global interrupt state */
    INTCONbits.GIE = state;
}

void init_interrupts(void) {
    /* Interrupt on falling edge of external interrupt pin */
    INTCONbits.INTEDG = 0;
    
    /* Enable external interrupt */
    PIE0bits.INTE = 1;
    
    /* Enable Timer0 interrupt */
    PIE0bits.TMR0IE = 1;
}

void init_rf(void) {
    /* Allow for maximum possible RF module startup time */
    __delay_ms(100);
    
    /* Set 1-byte payload width */
    nrf24_write_register(NRF24_RX_PW_P0, PAYLOAD_WIDTH);
    
    /* Set RF power to 0 dBm and data rate to 1 Mbit/Sec */
    nrf24_write_register(NRF24_RF_SETUP, NRF24_RF_PWR_0DBM);
    
    /* Set 5-byte address width */
    nrf24_write_register(NRF24_SETUP_AW, NRF24_AW_5);
    
    /* Set initial RF channel */
    nrf24_write_register(NRF24_RF_CH, RF_CHANNEL);
    
    /* Enable receive on pipe 0 only */
    nrf24_write_register(NRF24_EN_RXADDR, NRF24_ERX_P0);
    
    /* Set receive address  */
    nrf24_write_register_multi(NRF24_RX_ADDR_P0, RF_RX_ADDR, ADDR_LEN);
    
    /* Mask RX_DR interrupt on RF module, enable CRC, power up RF module in transmit-standby mode */
    nrf24_write_register(NRF24_CONFIG, NRF24_MASK_TX_DS | NRF24_MASK_MAX_RT | NRF24_EN_CRC | NRF24_PWR_UP | NRF24_PRIM_RX);
    
    /* Clear any pending RF module interrupts */
    nrf24_write_register(NRF24_STATUS, NRF24_TX_DS | NRF24_MAX_RT | NRF24_RX_DR);
}

void init_timer(void) {
    /* Set Timer0 clock source to HFINTOSC so timer will run during sleep */
    T0CON1bits.T0CS = 0b011;

    /* Set Timer0 prescaler to 1:32768 = 122.1 ticks/sec at HFINTOSC = 4 MHz */
    T0CON1bits.T0CKPS = 0b1111;
    
    /* Timer0 runs asynchronously to allow operation during sleep */
    T0CON1bits.T0ASYNC = 1;
    
    /* Set Timer0 postscaler to 1:4 for approximately 8.4 second interrupt time */
    T0CON0bits.T0OUTPS = 0b0011;
    
    /* Set Timer0 period to maximum (TMR0H is timer period in 8-bit mode) */
    TMR0H = 0xFF;
    TMR0L = 0x00;
}

void main(void) {
    uint8_t buf;
    
    /* Initialize peripherals */
    init_ports();
    init_osc();
    init_timer();
    init_pps();
    init_spi();
    init_interrupts();
    init_rf();
    
    /* Clear any spurious external interrupt request */
    PIR0bits.INTF = 0;
    
    /* Enable global interrupts */
    INTCONbits.GIE = 1;
    
    NRF24_CE_ACTIVE();
    
    while(1) {       
        SLEEP();
        
        if (data_pending) {
            nrf24_read_payload(&buf, PAYLOAD_WIDTH);

            if (buf & LEFT_RED_MASK)
            {
                LEFT_RED_LED = 1;
            } else {
                LEFT_RED_LED = 0;
            }

            if (buf & RIGHT_GREEN_MASK)
            {
                RIGHT_GREEN_LED = 1;
            } else {
                RIGHT_GREEN_LED = 0;
            }

            if (buf & LEFT_WHITE_MASK)
            {
                LEFT_WHITE_LED = 1;
            } else {
                LEFT_WHITE_LED = 0;
            }

            if (buf & RIGHT_WHITE_MASK)
            {
                RIGHT_WHITE_LED = 1;
            } else {
                RIGHT_WHITE_LED = 0;
            }  

            nrf24_write_register(NRF24_STATUS, NRF24_RX_DR);
            
            if (buf != 0x00) {
                TMR0L = 0x00;
                T0CON0bits.T0EN = 1;
            } else {
                T0CON0bits.T0EN = 0;
            }
            
            data_pending = false;
        }
    }
}
