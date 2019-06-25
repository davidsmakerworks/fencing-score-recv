/*
 * FencingRecv.asm
 *
 * Receives signal from scoring box using nRF24L01+ transceiver module
 *
 * Configuration is located at beginning of CSEG
 *
 * Global registers:
 * r24 = bit flags to indicate interrupt source
 * r25 = watchdog counter value
 *
 * Set ATtiny2313 fuses:
 *
 * CKDIV8: disabled
 * SUT_CKSEL: INTRCOSC_4MHZ_14CK_65MS
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
 *
 */ 

.include "tn2313def.inc"
.include "nRF24L01P.inc" 

/* Define pins for RF module functions */
.equ RF_CE = PORTD0
.equ RF_CSN = PORTD1
.equ RF_IRQ = PORTD2

/* Define watchdog counter start value */
.equ WD_COUNT = 0x4D // Approximately 5 seconds at 4 MHz CLKio, prescaler = CLKio/1024, T0max = TOP

/* Define RF module address length */
.equ ADDR_LEN = 0x05 // 5-byte address length

/* Define payload width */
.equ PAYLOAD_WIDTH = 0x01

/* Channel and address are stored in RAM in case they need to be changed during execution */
.dseg
Channel:
	.byte 0x01

Addr:
	.byte 0x05

.cseg
.org 0x0000
	rjmp Reset

/* Wake up on INT0 (RF module external interrupt) */
.org INT0addr
	rjmp RFInt

/* Handle watchdog register when LEDs are on */
.org OVF0addr
	rjmp TimerTickInt

.org INT_VECTORS_SIZE

/* CONFIG STARTS HERE */

CHANNEL_PM: // RF channel, padded to word boundary
	.db 0x0C, 0x00

ADDR_PM: // RF module address, MSByte first, padded to word boundary
	.db 0x50, 0x46, 0x46, 0x43, 0xBB, 0x00
	
/* CONFIG ENDS HERE */

Reset:
	/* Disable MCU interrupts during initialization */
	cli

	/* Initlaize stack pointer */
	ldi r16, RAMEND
	out SPL, r16

	/* Set DO, SCK, and LED control lines on PORTB to outputs */
	ldi r16, (1<<PORTB7)|(1<<PORTB6)|(1<<PORTB3)|(1<<PORTB2)|(1<<PORTB1)|(1<<PORTB0)
	out DDRB, r16

	/* Set CE and CSN lines on PORTD to outputs */
	ldi r16, (1<<PORTD0)|(1<<PORTD1)|(1<<PORTD5) // REMOVETHIS D5
	out DDRD, r16

	/* Get initial channel value from program memory and store it in RAM */
	ldi ZL, low(CHANNEL_PM<<1)
	ldi ZH, high(CHANNEL_PM<<1)
	ldi XL, low(Channel)
	ldi XH, high(Channel)
	lpm r16, Z
	st X, r16

	/* Get initial address value (5 bytes) from program memory and store it in RAM */
	ldi ZL, low(ADDR_PM<<1)
	ldi ZH, high(ADDR_PM<<1)
	ldi XL, low(Addr)
	ldi XH, high(Addr)
	rcall InitAddr

	/* Deactivate chip select on RF module */
	sbi PORTD, RF_CSN

	/* Set receive payload width to 1 byte */
	ldi r17, RX_PW_P0
	ldi r18, PAYLOAD_WIDTH
	rcall SetRegisterSingle

	/* Set address width to 5 bytes */
	ldi r17, SETUP_AW
	ldi r18, AW_5
	rcall SetRegisterSingle

	/* Set RF channel to initial value */
	ldi r17, RF_CH
	ldi XL, low(Channel)
	ldi XH, high(Channel)
	ld r18, X
	rcall SetRegisterSingle

	/* Set RF power to 0 dBm and data rate to 1 mBit/sec */
	ldi r17, RF_SETUP
	ldi r18, RF_PWR_0DBM
	rcall SetRegisterSingle

	/* Enable receive pipe 0 only */
	ldi r17, EN_RXADDR
	ldi r18, (1<<ERX_P0)
	rcall SetRegisterSingle

	/* Set receive address on pipe 0 */
	ldi r17, RX_ADDR_P0
	ldi r18, ADDR_LEN
	ldi XL, low(Addr)
	ldi XH, high(Addr)
	rcall SetRegisterMulti

	/* Mask transmit-related RF module interrupts, enable CRC, and power up RF module in receive mode */
	ldi r17, CONFIG
	ldi r18, (1<<MASK_TX_DS)|(1<<MASK_MAX_RT)|(1<<EN_CRC)|(1<<PWR_UP)|(1<<PRIM_RX)
	rcall SetRegisterSingle

	/* Clear any pending interrupts on RF module */
	ldi r17, STATUS
	ldi r18, (1<<TX_DS)|(1<<MAX_RT)|(1<<RX_DR)
	rcall SetRegisterSingle

	/* 
	 * Set INT0 external interrupt to occur on low logic level (default behavior - included for clarity/future use)
	 * Set Idle sleep mode (default behavior - included for clarity/future use)
	 * Enable MCU sleep mode
	 *
	 */
	ldi r16, (1<<SE)|(0<<SM1)|(0<<SM0)|(0<<ISC01)|(0<<ISC00)
	out MCUCR, r16

	/* Enable INT0 external interrupt */
	ldi r16, (1<<INT0)
	out GIMSK, r16

	/* Enable Timer0 overflow interrupt */
	ldi r16, (1<<TOIE0)
	out TIMSK, r16
	
	/* Clear interrupt flag register */
	clr r24

	/* Enable MCU global interrupts */
	sei

	/* Activate RF module chip enable */
	sbi PORTD, RF_CE

Loop:
	/* Wait for interrupt (RF module or timer tick) */
	sleep

	/* Clear interrupts while routine is executing */
	cli
	
	/* If bit 1 in r24 is clear, that means this is a timer tick. Jump to Continue and don't get RF payload */
	sbrc r24, 1
	rjmp Continue

	/* Get payload byte from RF module */
	rcall GetPayload

	/* Discard unused input bits */
	andi r16, 0b00001111

	/* If the packet was 0 (i.e., all lights off) stop the watchdog counter */
	breq StopTm
StartTm:
	/* If the packet was not 0 (i.e., lights turned on) start the watchdog counter and jump to SetLEDs */
	rcall StartTimer
	rjmp SetLEDs
StopTm:
	rcall StopTimer
SetLEDs:
	/* Turn on LEDs based on received data */
	out PORTB, r16

	/* Clear RF module interrupt */
	ldi r17, STATUS
	ldi r18, (1<<RX_DR)
	rcall SetRegisterSingle
Continue:
	/* Clear interrupt source register, re-enable interrupts and repeat loop */
	clr r24

	sei
	rjmp Loop

/* 
 * GetPayload
 *
 * Inputs:
 * 
 * None
 *
 * Outputs:
 *
 * r16 = payload from nRF24L01+ (byte)
 *
 */

GetPayload:
	cbi PORTD, RF_CSN
	ldi r16, R_RX_PAYLOAD
	rcall SPITransfer
	ldi r16, SPI_NOP
	rcall SPITransfer
	sbi PORTD, RF_CSN
	ret

/*
 * SetRegisterSingle
 * 
 * Inputs:
 * 
 * r17 = register to set
 * r18 = desired register value (byte)
 *
 */

SetRegisterSingle:
	push r16
	cbi PORTD, RF_CSN
	ldi r16, W_REGISTER
	add r16, r17
	rcall SPITransfer
	mov r16, r18
	rcall SPITransfer
	sbi PORTD, RF_CSN
	pop r16
	ret

/*
 * SetRegisterMulti
 * 
 * Inputs:
 * 
 * r17 = register to set
 * r18 = number of bytes in register value
 * XH:XL = address of register value
 *
 * NOTE: Register value is stored in RAM with MSByte first. Value is sent to RF module with LSByte first.
 *
 */

SetRegisterMulti:
	push r16
	cbi PORTD, RF_CSN
	ldi r16, W_REGISTER
	add r16, r17
	rcall SPITransfer
	add XL, r18
	brcc SetRMLoop
	inc XH
SetRMLoop:
	ld r16, -X
	rcall SPITransfer
	dec r18
	brne SetRMLoop
	sbi PORTD, RF_CSN
	pop r16
	ret

/* 
 * GetRegisterSingle
 *
 * Inputs:
 * 
 * r17 = register to retrieve
 *
 * Outputs:
 *
 * r16 = register value (byte)
 *
 */

GetRegisterSingle:
	push r16
	cbi PORTD, RF_CSN
	ldi r16, R_REGISTER
	add r16, r17
	rcall SPITransfer
	ldi r16, SPI_NOP
	rcall SPITransfer
	sbi PORTD, RF_CSN
	pop r16
	ret

/*
 * GetRegisterMulti
 * 
 * Inputs:
 * 
 * r17 = register to get
 * r18 = number of bytes in register value
 * XH:XL = address of register value
 *
 * NOTE: Register value is stored in RAM with MSByte first. Value is received RF module with LSByte first.
 *
 */

GetRegisterMulti:
	push r16
	cbi PORTD, RF_CSN
	ldi r16, R_REGISTER
	add r16, r17
	rcall SPITransfer
	add XL, r18
	brcc GetRMLoop
	inc XH
GetRMLoop:
	ldi r16, SPI_NOP
	rcall SPITransfer
	st -X, r16
	dec r18
	brne GetRMLoop
	sbi PORTD, RF_CSN
	pop r16
	ret

/* 
 * SPITransfer
 *
 * Inputs:
 * 
 * r16 = data (byte) to transmit on SPI bus
 *
 * Outputs:
 *
 * r16 = data (byte) received from SPI bus
 *
 */
	
SPITransfer:
	out USIDR, r16
	ldi r16, (1<<USIOIF)
	out USISR, r16
	ldi r16, (1<<USIWM0)|(1<<USICS1)|(1<<USICLK)|(1<<USITC)
SPITransferLoop:
	out USICR, r16
	sbis USISR, USIOIF
	rjmp SPITransferLoop
	in r16, USIDR
	ret

/*
 *
 * StartTimer
 *
 * Inputs: None
 *
 * Outputs: None
 *
 */

StartTimer:
	push r16

	/* Initialize watchdog value to turn off LEDs if they are on for an unexpectedly long time */
	ldi r25, WD_COUNT

	/* Reset Timer0 value */
	ldi r16, 0x00
	out TCNT0, r16

	/* Start Timer0 with CLKio/1024 prescaling factor */
	ldi r16, (1<<CS02)|(1<<CS00)
	out TCCR0B, r16

	pop r16
	ret

/*
 *
 * StopTimer
 *
 * Inputs: None
 *
 * Outputs: None
 *
 */

StopTimer:
	/* Stop Timer0 (i.e., no clock source) */
	push r16
	ldi r16, (0<<CS02)|(0<<CS00)
	out TCCR0B, r16
	pop r16
	ret

/*
 *
 * RFInt
 *
 * Inputs: None
 *
 * Outputs: r24 = bit 0 set to signify RF module (i.e., external INT0)
 *
 */

RFInt:
	push r1
	in r1, SREG
	sbr r24, 0b01
	out SREG, r1
	pop r1
	reti

/*
 *
 * TimerTickInt
 *
 * Inputs: None
 *
 * Outputs: r24 = bit 1 set to signify timer tick interrupt
 *
 */

TimerTickInt:
	push r1
	in r1, SREG
	sbr r24, 0b10
	dec r25
	brne TimeOK

	/* Turn off LEDs and stop timer if watchdog counter has expired */
	push r16
	ldi r16, 0b00000000
	out PORTB, r16
	rcall StopTimer
	pop r16
TimeOK:
	out SREG, r1
	pop r1
	reti

/*
 *
 * InitAddr
 *
 * Inputs:
 * r17 = number of bytes in address
 * ZH:ZL = program memory address of initial value
 * XH:XL = RAM address to store value 
 *
 */

InitAddr:
	push r16
	push r17
	ldi r17, ADDR_LEN
InitAddrLoop:
	lpm r16, Z+
	st X+, r16
	dec r17
	brne InitAddrLoop
	pop r17
	pop r16
	ret