/*
 * m328_GPS_clock.asm
 *
 *  Created: 10/7/2015 9:28:39 AM
 *   Author: lynf
 */ 
;
;
;######################################################################################
; This software is Copyright by Francis Lyn and is issued under the following license:
;
; Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License
;
;######################################################################################
;
; Change Log:
; ===========
; 1. Revise <initz:> to enable pull-ups on all unused ports as precaution
;	to reduce unnecessary device current consupmtion.
; 
; 2. Set default TZoff = -4 h (was -5). Revise A0 (pin PC0) input to switch to DST when
;	 pin grounded, by increasing TZ by -1 h. Modify <gettz:>.
;
; 3. Change <gmt_lt_hr:> to use DST correction if A0 grounded. Modify <main:> to
;	 remove call to <gettz:> which is no longer used.
; 
; 
; All Versions of m328_GPS_clock.asm run at 9600 baud.
;
; Copyright F.Lyn 2016.07.26, all rights reserved
;
; NOTE: The 3.3 V version of the Pro-mini controller board uses an 8.000 MHz
;		clock compared to the 5 V version, so the parameters affected by the
;		Fosc have to be changed accordingly. This is done by setting the equate
;		Vcc_low = true for 3.3 V, else Vcc_low = false for 5 V devices.
;
; The default time zone is -4 h, corresponding to Ontario, Canada.
; The default TZoff is stored to TZb by <ld_eeprm1> during controller startup
;
;
; Change List
;=============
; 1. The TZ offset sign flag TZposf must be saved to EEPROM and restored when
;	 EEPROM read back.
;
;
;
; The GPS module interfaces to the controller via serial UART connection.
;
; ATmega328P cpu, 16.0000 MHz external crystal * Depends on controller board
;
; PC5 = SCL
; PC4 = SDA
;
; PC0 = DST_en, enable DST correction by grounding this pin

.list		; Listing on

;
; General equates
;
.equ	FALSE = 0x0			; Logical 0
.equ	TRUE = !FALSE		; Logical 1
.equ	Vcc_low = false		; False for 5 V controller board
;.equ	Vcc_low = true		; True for 3.3 V controller board
.equ	debug = true		; Turn on debugging routines
;.equ	debug = false		; Turn on debugging routines
.equ	baud_low = true		; 9600 baud for later boards
;.equ	baud_low = false	; 19200 baud for 1st board
;
; GPS equates
;
.equ	LED = PD4			; D4, test LED
.equ	time_sz = 9			; Time string length
.equ	date_sz = 6			; Date string length
.equ	fieldskip = 8		; Fields to skip
.equ	TZoff = 4			; TZ offset, default value for Ontario, Canada
;.equ	TZoff = 8			; TZ offset, default value for Beijing, China
.equ	DST_en = PC0		; Enable DST correction
;
; Processor operating voltage
;

.if	Vcc_low
.equ F_CPU = 8000000
.else
.equ F_CPU = 16000000
.endif

;
; $GPRMC header data
;
.equ	hdr_sz = 6
;
;
; TWI specific equates
;
.equ	STARTC = 0x08		; START condition sent
.equ	RSTART = 0x10		; REPEATED START condition sent
.equ	SLAw_ACK = 0x18		; Slave address write sent, ACK returned
.equ	SLAw_NAK = 0x20		; Slave address write sent, NAK returned
.equ	SLAr_ACK = 0x40		; Slave address write sent, ACK returned
.equ	SLAr_NAK = 0x48		; Slave address write sent, NAK returned
.equ	DATAr_ACK = 0x50	; Data byte received, ACK returned
.equ	DATAw_ACK = 0x28	; Data byte sent, ACK returned
.equ	DATAw_NAK = 0x30	; Data byte sent, NAK received
.equ	DATAr_NAK = 0x58	; Data byte received, NAK returned
.equ	RTC_MSLA = 0xfe		; Master's own slave address
;
;
; LCD backpack slave twi
;
.equ	LCD_SLA = 0b01001110	; LCD slave address (0x27), 7 bits + R/W bit
.equ	LCD_RS = 0				; LCD register select control
.equ	LCD_RW = 1				; LCD read/write* control
.equ	LCD_E = 2				; LCD enable control
.equ	LCD_BT = 3				; LCD backlight control
;
;
;
; UART definitions
;
;
.if		baud_low
.equ	BAUD = 9600			; Baud rate
.if		Vcc_low
.equ	BAUD_PRE = 51		; Baud rate prescaler - 8.00 MHz clock, 9600
.else
.equ	BAUD_PRE = 103		; Baud rate prescaler - 16.00 MHz clock, 9600
.endif
;
.else
;
.equ	BAUD = 19200		; Baud rate
.if		Vcc_low
.equ	BAUD_PRE = 25		; Baud rate prescaler - 8.00 MHz clock
.else
.equ	BAUD_PRE = 51		; Baud rate prescaler - 16.00 MHz clock
.endif
;
.endif
;
.equ	NULL = 0x0			; Null terminator
.equ	BELL = 0x07			; Bell
.equ	BS = 0x08			; Backspace
.equ	HT = 0x09			; Tab
.equ	LF = 0x0a			; Linefeed
.equ	CR = 0x0d			; Carriage return
.equ	ctlW = 0x17			; Control W
.equ	ctlX = 0x18			; Control X
.equ	ctlZ = 0x1a			; Control Z
.equ	SP = 0x20			; Space
.equ	ESC = 0x1b			; Escape
.equ	DEL = 0x7f			; Delete
.equ	CMA	= 0x2c			; Comma
.equ	at = 0x40			; '@'
;


.if		Vcc_low

; Timer0, Timer1 and Timer2 parameters for 8.00 MHz clock
;
; Prescaler:	1		8		64			256			1024
; TCNTn clk:	8 MHz	1 MHz	125 kHz		31.25 kHz	7.8125 kHz
; Period:				1 us	8 us		32 us		128 us
;
; TCNT0 prescaler = 1024, clk_T0 = 8 MHz/1024 = 7.8125 kHz, 128 us
; TCNT1 prescaler = 256, clk_T1 = 8 MHz/1024 = 7.8125 kHz, 128 us
; TCNT2 prescaler = 1024, clk_T2 = 8 MHz/1024 = 7.8125 kHz, 128 us
;
;
.equ	OCR0Aload = 32		; OCR0A 8 bit register, 32 x 128 us = 4.096 ms
.equ	OCR1A64us = 1		; OCR1A 16 bit register, 128 us
.equ	OCR2Aload = 39		; OCR2A 8 bit register, 39 x 128 us = 4.992 ms
.equ	OCR1A1728us = 14	; OCR1A 16 bit register, 1562 x 128 us = 1792 us
.equ	OCR1A5ms = 39		; OCR1A 16 bit register, 39 x 128 us = 5 ms
.equ	OCR1A100ms = 781	; OCR1A 16 bit register, 781 x 128 us = 100 ms
;

.else

;
; Timer0, Timer1 and Timer2 parameters for 16.00 MHz clock
;
; Prescaler:	1		8		64			256			1024
; TCNTn clk:	16 MHz	2 MHz	250 kHz		62.5 kHz	15.625 kHz
; Period:				0.5 us	4 us		16 us		64 us
;
; TCNT0 prescaler = 1024, clk_T0 = 16 MHz/1024 = 15.625 kHz, 64 us
; TCNT1 prescaler = 256, clk_T1 = 16 MHz/1024 = 15.625 kHz, 64 us
; TCNT2 prescaler = 1024, clk_T2 = 16 MHz/1024 = 15.625 kHz, 64 us
;
;
.equ	OCR0Aload = 64		; OCR0A 8 bit register, 64 x 64 us = 4.096 ms
.equ	OCR1A64us = 1		; OCR1A 16 bit register, 64 us
.equ	OCR2Aload = 78		; OCR2A 8 bit register, 78 x 64 us = 4.992 ms
.equ	OCR1A1728us = 28	; OCR1A 16 bit register, 1562 x 64 us = 1728 us
.equ	OCR1A5ms = 78		; OCR1A 16 bit register, 78 x 64 us = 5 ms
.equ	OCR1A100ms = 1562	; OCR1A 16 bit register, 1562 x 64 us = 100 ms

.endif
;
;###########################################################################
;
; SPI interface to serial output module
;
.equ	SS = PB2			; SS output, serial relay driver
.equ	MOSI = PB3			; Master output to slave
.equ	MISO = PB4			; Master input from slave
.equ	SCK = PB5			; Master clock output
;	
;
;###########################################################################
;
;
;
; Flag register flaga
;
.equ	numfl = 0			; Valid byte number flaga
.equ	crf = 1				; Carriage return key flaga
.equ	escf = 2			; Escape key flaga
.equ	kyf	= 3				; Control key flaga
.equ	xclinf = 4			; Delayed line clear flaga
.equ	tcnt1fa = 5			; TCNT1 flaga software timer flag
;
; Flag register flagb
;
.equ	LCD_LEDb = 0		; LCD backlight control flagb
.equ	paceb = 1			; Clock display pace flagb
.equ	pacec = 2			; LCD update pace flag
.equ	TZposf = 3			; TZ + offset ready flagb
.equ	twi_erfb = 4		; TWI write error flagb
.equ	t100mf = 5			; 200ms timer flag
;
;
;
.equ	maxbits = 0xff		; High byte mask
.equ	nbits = 16			; Convert 16 bits
.equ	ndig = 3			; Digit pair bytes
.equ	ndec = 5			; Digits to display/convert
;
;
; --- Line input buffer ---
;
.equ	linsz = 8			; Line buffer size
;
; --- Register definitions ---
;
; Low registers
;
.def	count = R2			; Counter for line buffer
.def	asav = R3			; rga save register
.def	SRsav = R4			; SREG save
.def	res0 = R5			; result register 0
.def	res1 = R6			; result register 1
.def	res2 = R7			; result register 2
.def	rtmr = R8			; Temporary timer
;
;
; High registers
;
.def	rmp = R16			; Multipurpose register
.def	rga = R17			; GP register RGA
.def	rgb = R18			; GP register RGB
.def	rgc = R19			; GP register RGC
.def	rgd = R20			; GP register RGD
.def	rge	= R21			; GP register RGE
.def	rgv	= R22			; Variable register
.def	flaga = R23			; Flag A register, 8 flags
.def	flagb = R24			; Flag B register, 8 flags
;
;
;
; --- Macro definitions ---
;
.macro	ldzptr				; Load ZH:ZL pointer with address*2
		ldi		ZH,high(@0*2)
		ldi		ZL,low(@0*2)
.endm
;
.macro	ldxptr				; Load XH:XL pointer with address to access data memory
		ldi		XH,high(@0)
		ldi		XL,low(@0)
.endm
;
.macro	ldyptr					; Load YH:YL pointer with address to access data memory
		ldi		YH,high(@0)
		ldi		YL,low(@0)
.endm
;
; Exchange contents of registers
;
.macro	xchreg					; Exchange registers
		push	@0
		push	@1
		pop		@0
		pop		@1
.endm
;
;
; --- SRAM Data Segment ---
;
.DSEG
.ORG	0X0100				; 2 Kb SRAM space
;
;
; RTC clock module data buffers
;
stbuf:						; Space for GPS data buffers
;
timeb:
.byte		9				; time buffer byte
;
dateb:
.byte		6				; Date
;
TZb:
.byte		1				; TZ offset value
;
flagb_img:
.byte		1				; Offset sign
;
LT_hourb:
.byte		2				; Local time hours
;
enbuf:
;
linbuf:
.byte	linsz				; Character input line buffer
;
;
; Data buffer for word and byte
;
wdbuf:
.byte	2					; Word byte buffer
;
dba:
.byte	1					; Data byte buffer (Keep this buffer below wdbuf)
;
;
buffend:
;
;
; ============================================
;   R E S E T   A N D   I N T   V E C T O R S
; ============================================
;
;
; --- Code Segment ---
;
.CSEG
.ORG	$0000						; Interrupt vectors go here
;
		jmp			start			; Reset vector
;
.ORG	INT0addr					; INT0 handler for RTC clock module
		jmp			ExtINT0
;
;.ORG	INT1addr					; INT1 handler
;		jmp
;
.ORG	OC0Aaddr
		jmp			Timer0_COMPA	; Timer 0 Output Compare A handler
;
.ORG	OC1Aaddr
		jmp			Timer1_COMPA	; Timer 1 Output Compare A handler
;
;
; End of interrupt vectors, start of program code space
;
;
.ORG	0x0034					; Program begins here
;
;
;###########################################################################
;
;
;
; ============================================
;     I N T E R R U P T   S E R V I C E S
; ============================================
;
;
; --- Timer 0 interrupt handler ---
;
; Used for I/O scanning
; 
; TCNT0 run in Output Compare mode, using OCR0A register to
; generate output compare interrupt every 64 x 64 us = 4.096 ms.
;
; TCNT0 operates in Clear Timer on Compare Match (WGM02:0 = 2).
; On Compare Match, TCNT0 counter is cleared.
; OCR0A (set to 64) defines the counter's TOP value.
;
; Clk_T0 = 16 MHz/1024 = 15.625 kHz, 64 us period, 
;
Timer0_COMPA:
;
		push	rmp					; Save registers
		in		SRsav,SREG
;
; 100 ms timer, 
;
t_100m:
		sbrc	flagb,t100mf		; Flag set?
		rjmp	t_100m1				;	Yes, exit timer
		dec		rtmr				;	No, run timer
		brne	t_100m1
		sbr		flagb,(1<<t100mf)	; Set flag at end of timer count down
t_100m1:
;
		out		SREG,SRsav			; Restore SREG
		pop		rmp
		reti
;
; --- Timer 1 interrupt handler ---
;
; Used to generate LCD time delays
;
; TCNT1 (16 bit) run in output compare mode, using OCR1A register to
; generate output compare interrupt every 64 us.
;
; TCNT1 operates in mode 5 CTC (WGM13:10 = 4).
; On compare match, TCNT1 is cleared.
; OCR1A (set to 1) define the counter's TOP value.
;
; Clk_tb = 16 MHz/1024 = 15.625 kHz, 64 us period, 
;
Timer1_COMPA:
;
		push	rmp
		in		SRsav,SREG					; Save SREG
;
		lds		rmp,TCCR1B					; Fetch control register
		cbr		rmp,(1<<CS12)|(1<<CS10)		; Stop timer by clearing CS12 & CS10
		sts		TCCR1B,rmp
;
		sbr		flaga,(1<<tcnt1fa)			; Foreground flag
;
		out		SREG,SRsav					; Restore SREG
		pop		rmp
		reti		
;
;
; External interrupt handler for INT0 (PD2) falling edge 
;
ExtINT0:
		in		SRsav,SREG
;
		sbr		flagb,(1<<paceb)	; Set clock display pace flag
		sbr		flagb,(1<<pacec)	; Set LCD update pace flag
;
		out		SREG,SRsav			; Restore registers
		reti
;
;
;
;
;###########################################################################
;
;
; ============================================
;         Initialization routines
; ============================================
;
;
; Turn off watchdog
;
wdt_off:
		cli							; Clear global interrupts
;
; Reset WD timer
;
		wdr
;
		in		rmp,MCUSR				; Clear WDRF bit
		andi	rmp,(0xff & (0<<WDRF))	; WDRF bit = 0
		out		MCUSR,rmp
;
; Set WDCE and WDE bits, keep old prescaler setting
;
		lds		rmp,WDTCSR
		ori		rmp,(1<<WDCE)|(1<<WDE)
		sts		WDTCSR,rmp
;
; Turn off WDT
;
		ldi		rmp,(0<<WDE)			; Clear WD system reset enable
		sts		WDTCSR,rmp
;
		sei								; Set global interrupts
		ret
;
; --- Initialization Routines ---
;
initz:
		rcall	zbuf			; Clear data space buffers
;
		rcall	zregs			; Clear lower registers R0,..,R15
		clr		flaga			; Clear flag registers
		clr		flagb
;
;
; Activate pull-up resistors on all input pins, used and unused
;
		ldi		rmp,0b00011101
		out		PORTB,rmp
;
		ldi		rmp,0b00111111
		out		PORTC,rmp
;
		ldi		rmp,0b11101100
		out		PORTD,rmp
;
;
; Initialize PD3,2 as inputs for INT0 and INT1 inputs
;
		sbi		PORTD,PD2			; Enable PD2 pull-up
		sbi		PORTD,PD3			; Enable PD3 pull-up
;
; Enable TZ entry menu
;
	;	sbi		PORTC,PC0			; Enable PC0 (A0 input) pull-up
;
; PD4 test LED driver output 
;
		sbi		DDRD,led			; Enable led output
;
; SPI module, enable SPI, master mode, fosc/64
;
		ldi		rmp,(1<<SPE)|(1<<MSTR)|(1<<SPR1)	; fosc/64
		out		SPCR,rmp
;
.if		Vcc_low

		ldi		rmp,(1<<SPI2X)		; Enable SPI double speed if 8 MHz clock
		out		SPSR,rmp			; on 3.3 V controller board

.endif
;
;
;
; --- Timers Initialization ----
;
;
; === TCNT0 Initialization === 
;
; Setup TCNT0 prescaler = 1024, clock period = 64 us
;
InitTimer0:
		ldi		rmp,(1<<CS02)|(1<<CS00)	; Divide by 1024 prescaler, Fclk = 15.625 kHz
		out		TCCR0B,rmp				; Timer/Counter0 control register B
;
; Setup TCNT0 for CTC mode
;
		ldi		rmp,(1<<WGM01)			; CTC mode
		out		TCCR0A,rmp				; Timer/Counter0 control register A
;
; Initialize OCR0A output compare register
;
		ldi		rmp,OCR0Aload			; Set OCR0A = 64 for 4.096 ms period
		out		OCR0A,rmp
;
; Enable Timer/Counter0 Compare A Match Interrput in TIMSK0
;
		lds		rmp,TIMSK0
		sbr		rmp,(1<<OCIE0A)			; Enable Timer/Counter0 Output Compare A Match Interrupt
		sts		TIMSK0,rmp
;
;
; === TCNT1 Initialization === (OK)
;
; Setup 16 bit Timer/Counter1 Compare A in CTC mode. This timer
; runs when started and stops at end of timer cycle when TCNT1 = OCR1A
; and OC1A interrupt occurs. Timer is stopped inside interrupt handler.
;
; Setup TCNT1 prescaler for 1024, clock period = 64 us and CTC mode
; Note: User to explicitly start TCNT1 by setting CS12 in TCCR1B
; 
InitTimer1:
		ldi		rmp,(1<<WGM12)
		sts		TCCR1B,rmp				; Set CTC1 mode
;
; Initialize OCR1A output compare register
;
		ldi		rmp,high(OCR1A64us)		; Set OCR1A = 64 us period
		ldi		rga,low(OCR1A64us)
		sts		OCR1AH,rmp				; 16 bit write
		sts		OCR1AL,rga
;
; Enable TCNT1 Compare A Match Interrputs in TIMSK1
;
		lds		rmp,TIMSK1
		sbr		rmp,(1<<OCIE1A)		; Enable Output Compare A Match Interrupt
		sts		TIMSK1,rmp
;
;
; === INT0 Initialization ===
;
;
;
; --- Enable GLobal Interrupts
;
		sei							; Set global interrupts
		ret
;
;
; Initialize TWI module - set up as bus master
;
TWI_init:
;
		ldi		rmp,48			; 250 kHz TWI bus rate
		sts		TWBR,rmp		; Load bit rate generator divider
		ret
;
;
; Initialize the UART for 19,200 baud asynchronous operation
;
inzuart:
		cli							; Clear global interrupts
		ldi		rmp,high(BAUD_PRE)
		sts		UBRR0H,rmp			; Load baud rate register high
		ldi		rmp,low(BAUD_PRE)
		sts		UBRR0L,rmp			; Load baud rate register low
;
; Setup frame for 1 start, 8 data, 1 stop and no parity
;
		ldi		rmp,(1<<UCSZ00)|(1<<UCSZ01)
		sts		UCSR0C,rmp
;
; Enable the UART
;
		ldi		rmp,(1<<RXEN0)|(1<<TXEN0)
		sts		UCSR0B,rmp			; Enable RX and TX
		sei							; Set global interrupts
		ret
;
;
;
;###########################################################################
;
;     M A I N    P R O G R A M    S T A R T
;
;###########################################################################
;
;
;
; Controller startup and initialization
;
start:
;
; Initialize the stack pointer to end of SRAM
;
		ldi		rmp,high(RAMEND)	; Init MSB stack
		out		SPH,rmp
		ldi		rmp,low(RAMEND)		; Init LSB stack
		out		SPL,rmp
;
; Initialize the engine
;
		rcall	wdt_off			; Disable watchdog. Must be done soon after a reset
		rcall	initz			; Initialize engine
		rcall	TWI_init		; Initialize the TWI module		* LCD *
		call	LCD_ledon		; Turn on LCD backlight bit
		rcall	inzuart			; Initialize the UART
;
; The LCD is a slow start-up device, add 1 s delay after power-up
;
		rcall	d1s				;		* LCD *

;
; LCD module start-up. Home, clear display, turn off cursor
;
		rcall	LCD_start		; Open LCD TWI slave
		rcall	LCD_home
		rcall	LCD_clear
		rcall	LCD_cur_off		; Cursor off
;
;
; Main program control loop
;
main:
		rcall	prscn			; Print main screen and menu
;
		rcall	pxy				; Time and date headings
		.db		4,16
		ldzptr	thdr
		rcall	pptr
;
		rcall	pxy
		.db		5,13
		ldzptr	thdr1
		rcall	pptr
;
; Get EEPROM stored TZoff and flagb image
;
;		rcall	ld_eeprm			; Get TZoff from EEPROM
;		lds		flagb,flagb_img		; Restore flagb image
;
; This version of GPS-clock uses hard coded TZ offset and explicit
; setting TXposf = 1 in flagb register for positive offset,
; else TXposf = 0 in flagb register for negative offset by default. No call to
; <gettz:> used.
;
	;;	sbr		flagb,(1<<TZposf)	; Set + offset flag if positive
		ldi		rmp,TZoff			; Use default TZ offset
		sts		TZb,rmp				; Load buffer <-- rmp
;
; Check if DST_en input pin grounded to enable DST correction.
;
	;	sbis	PINC,DST_en		; Test if DST_en pin jumpered to ground
	;	rjmp	gettz			;	Yes, get TZ offset
;
;  Dump GPS time and date data to screen and LCD module.
;
main0:
		rcall	scn_hdr			; Get GPS NMEA sentence header
		brcs	main0			; Wrong header, keep scanning
		ldxptr	timeb			; Store time string to timeb
		ldi		rgb,time_sz		; time string size
;
; Save time string from $GPSRMC sentence
;
main1:
		rcall	ci				; Get incoming time string
		st		X+,rga			; timeb <-- @X+
		dec		rgb
		brne	main1			; Continue saving time string		
;
; Skip over to date string field
;
		ldi		rgb,fieldskip	; Fields to skip
main2:
		rcall	ci
		cpi		rga,','			; Comma field delimiter?
		brne	main2			; No, keep going
		dec		rgb
		brne	main2
;
; Save date string field
;
		ldxptr	dateb			; Store time string to timeb
		ldi		rgb,date_sz
main3:
		rcall	ci				; Get incoming date string
		st		X+,rga			; clkb <-- @X+
		dec		rgb
		brne	main3			; Continue saving date string		
;
; GPS $GPRMC time and date date now saved in timeb and dateb buffers
;
;
		rcall	pxy
		.db		6,14
;
		rcall	gmt_lt_hr		; Convert GMT to local time
		rcall	prtime			; Display time string on screen
		rcall	dblsp
		rcall	space
		rcall	prdate			; Display date string on screen
;
		rcall	lcd_prtime		; Display time string on LCD
		rcall	lcd_prdate		; Display date string on LCD
		 rcall	slinc
;
		rjmp	main0
;
;
; Print main screen
;
prscn:
		call	clrscn			; Home cursor and clear screen
		ldzptr	scrnm			; Print main banner
		call	pptr
		ret
;
; Print time string
;
prtime:
		ldxptr	LT_hourb		; Local time hours 
		ld		rga,X+
		rcall	co
		ld		rga,X
		rcall	co
		ldi		rga,':'
		rcall	co
;
		ldxptr	timeb+2			; Show minutes & seconds
		ldi		rgb,(time_sz-5)
prtime1:
		ldi		rgc,2
prtime1a:
		ld		rga,X+
		rcall	co
		dec		rgb
		dec		rgc
		brne	prtime1a
		tst		rgb
		breq	prtime2
		ldi		rga,':'
		rcall	co
		rjmp	prtime1
prtime2:
		ret
;
; Print date string
;
prdate:
		ldi		rgb,date_sz
		ldxptr	dateb
prdate1:
		ldi		rgc,2
prdate1a:
		ld		rga,X+
		rcall	co
		dec		rgb
		dec		rgc
		brne	prdate1a
		tst		rgb
		breq	prdate2
		ldi		rga,'.'
		rcall	co
		rjmp	prdate1
prdate2:
		ret
;
; Show time on LCD line 1
;
lcd_prtime:
		rcall	LCD_line1
		ldzptr	time_hdr		; Time header
		rcall	LCD_wrln
		ldxptr	LT_hourb		; Show local time hours 
		ld		rmp,X+
		rcall	LCD_wrdat		; Data to LCD slave
		ld		rmp,X
		rcall	LCD_wrdat
		ldi		rmp,':'
		rcall	LCD_wrdat
;
		ldxptr	timeb+2			; Show minutes & seconds
		ldi		rgb,(time_sz-5)
lcd_prtime1:
		ldi		rgc,2
lcd_prtime1a:
		ld		rmp,X+
		rcall	LCD_wrdat
		dec		rgb
		dec		rgc
		brne	lcd_prtime1a
		tst		rgb
		breq	lcd_prtime2
		ldi		rmp,':'
		rcall	LCD_wrdat
		rjmp	lcd_prtime1
lcd_prtime2:
		ret
;
; Show date on LCD line 2
;
lcd_prdate:
		rcall	LCD_line2
		ldzptr	date_hdr		; Date
		rcall	LCD_wrln
		ldxptr	dateb
		ldi		rgb,date_sz
;
lcd_prdate1:
		ldi		rgc,2
lcd_prdate1a:
		ld		rmp,X+
		rcall	LCD_wrdat		; Master to LCD slave
		dec		rgb
		dec		rgc
		brne	lcd_prdate1a
		tst		rgb
		breq	lcd_prdate2			
		ldi		rmp,'.'
		rcall	LCD_wrdat
		rjmp	lcd_prdate1
lcd_prdate2	:
		ret
;
;
;###########################################################################
;
;
; Convert GMT time to Local time by adjusting hours to time zone difference.
; Read in the hours ascii characters from timeb (first two digits), convert
; to packed BCD number. Add/subtract TZoff by inc/dec hours BCD by time zone
; amount. Convert result back to ascii chars and store in LT_hourb buffer.
;
; (See m328-Simple-RTC.asm for set_clk_hr example code)
;
gmt_lt_hr:
		ldyptr	LT_hourb		
		lds		XH,timeb			; Get timeb tens hours
		ldi		rmp,'0'				; Ascii '0'
		sub		XH,rmp				; Convert to binary
		swap	XH					; Position as high BCD nibble
		lds		XL,(timeb+1)		; Get timeb units hours
		sub		XL,rmp				; Convert to binary, low nibble
		or		XH,XL				; Merge to form packed BCD byte in XH
		mov		rga,XH				; Packed BCD hours in rga
		lds		rgb,TZb				; Get TZ offset
;
; Check if DST_en pin grounded to apply DST correction
;
		sbis	PINC,DST_en		; Test if DST_en pin jumpered to ground
		inc		rgb				;	Yes, apply DST correctiont
;
		sbrs	flagb,TZposf		; Check if + or - TZ correction
		rjmp	dec_hr
;
inc_hr:
		inc		rga					
		rcall	daa
		cpi		rga,0x24			; Rollover at 24 h
		brne	gmt_lt_hr2
		clr		rga					; Clear to 00
;
gmt_lt_hr2:
		dec		rgb					; Apply time zone correction
		brne	inc_hr
		rjmp	gmt_lt_hr3
;
;
dec_hr:
		tst		rga					; Test if hours = 00
		breq	dec_hr1				;	Yes, rollunder to 23 hours
		dec		rga
		rcall	das
		rjmp	gmt_lt_hr1
dec_hr1:
		ldi		rga,0x23
;
gmt_lt_hr1:
		dec		rgb					; Apply time zone correction
		brne	dec_hr
;
gmt_lt_hr3:
		push	rga					; Convert packed BCD to ascii
		swap	rga					; Process MSD nibble first
		rcall	cv2asc				; Convert to ascii
		pop		rga
;
; Convert packed BCD number (2 digits) into two ascii digits
;
cv2asc:
		andi	rga, 0x0f			; Mask off higher nibble
		ldi		rgv, 0x30 			; Add ascii '0' to convert
		add		rga, rgv			; Convert to ascii in rga
		st		Y+,rga				; Update LT_hourb buffer
		ret
;		
;
;
; --- Decimal Adjust after Addition ---
;
; rga has result of 2 packed packed BCD digits after addition on entry.
; DAA routine adjusts rga for proper BCD representation. C flag has result
; of carry-out to allow for multiple precision additions.
;
; Registers: rga
; Entry:	rga = result of prior packed bcd addition
; Exit:		rga = decimal adjusted result for 2 digits
;  
daa:
		push	rmp
		ldi		rmp,0x66	; Adjustment for both BCD digits
		add		rga,rmp		; Add adjustment to BCD pair
		brcc	danoc		; C=0
		andi	rmp,0x0f	; C=1, high nibble adjustment removed
danoc:
		brhc	danoh		; H=0
		andi	rmp,0xf0	; H=1, low nibble adjustment removed
danoh:
		sub		rga,rmp		; Final adjustment
		pop		rmp
		ret
;
; DAS decimal adjust subtraction of two packed BCD numbers
; Based on Intel IA-32 instruction code for DAS
;
das:
		push	rgb					; Save working registers
		push	rgc
;
; Save entry data
;
		mov		rgb,rga				; copy of rga for lo nibble testing
		mov		rgc,rga				; rgc has rga for hi nibble testing
;
; Test low nibble. If ((rga & 0x0f) > 9)) --> daa_adjlo
;
das_testlo:
		andi	rgb,0x0f
		cpi		rgb,9+1
		brcc	das_adjlo		; Low nibble is > 9
		rjmp	das_testhi
;
das_adjlo:
		ldi		rgb,0x06		; Decimal adjust for low nibble
		sub		rga,rgb			; Add 0x06 to entry_rga
;
; Test high nibble. If ((entry_rga > 0x99) or (entry_C = 1)) --> daa_adjhi
;
das_testhi:
		cpi		rgc,0x99+1		; Test entry_rga
		brcc	das_adjhi		; entry_rga > 0x99
		rjmp	das_x
;
das_adjhi:
		ldi		rgb,0x60		; Decimal adjust for high nibble
		sub		rga,rgb			; Add 0x60 to entry_rga
;
das_x:
;
		pop		rgc
		pop		rgb
		ret
;
;
;
;
;###########################################################################
;
;
; --- Message scan routines ---	(OK)
;
;
;	Registers used:
;	rmp, rga, rgb, rgc, ZHL
;
; Exit: C = 1 if fail, 
;		C = 0 if success
;
; Desired sentence type ($GPRMC), has time and date information
;
scn_hdr:
		rcall	ci				; Get a character
		cpi		rga,'$'			; Test if '$'
		brne	scn_hdr			;	No, wait '$'
;
; Incoming characters following '$'in rga
;
		ldzptr	RMC_hdr			; Header string pointer
		ldi		rgb,hdr_sz		; Length of header string
;
scn_hdr0:
		rcall	ci				; Get next character
		lpm		rgc,Z+			; Get a RMC_hdr string character
;
; Check for match
;
		eor		rga,rgc			; Match?
		breq	scn_hdr1		; Yes
		sec						; No, error exit
		ret
;
; Check remaining characters
;
scn_hdr1:
		dec		rgb				; Count characters compared
		brne	scn_hdr0
		clc
		ret						; Exit match found
;
; GPRMC header string
;
RMC_hdr:
		.db		"GPRMC,",ctlZ	; Sentence header
;
;
;###########################################################################
;
; RX UART input is wired to GPS module TX output in normal clock operation.
; For setting TZ offset data, disconnect the GPS TX output from the UART
; channel RX input to force controller to look only at the terminal's keyboard
; output. DST_en pin must be grounded to run the <gettz> code.
;
; When TZ data is entered and successfully stored in EEPROM, restore
; GPS connection to RX input, remove the DST_en jumper, and reset the
; controller.
;
; (Based on <ldt> routine in Tile)
;
; Get time zone offset and limit check to less than equal 23. C = 1 and
; rgd:rge has packed BCD decimal digits if ok else C = 0. Arrow keys exit
; immediately and ESCF set.
;
;	Exit: rga = 0 if no entry, rga = 0xff if HH entry
;	      C = 1 if entry good, else C = 0.
;        rgd <-- HH setting if 1 or 2 digit entry
;
gettz:
		rcall	clean
		rcall	prtzbuf				; Show TZ offset in stored buffer
		rcall	prtzm
;
gettz1:
		rcall	enter
gettz1a:
		rcall	glin				; Get line input
;
; Test first character for '+' or '-' entry. If no sign, assume '+'
;
		ldi		rmp,'0'				; Ascii '0', for stuffing
		ld		rga,X				; Get 1st character from line buffer
		tst		rga					; Blank line?
		brne	gettz1b				;	No, continue checking
		cbr		flaga,(1<<xclinf)	;	Yes, clear line and redo entry
		call	prtzm				; Show TZ message, erase rest of line
		rjmp	gettz1
;
gettz1b:
		cpi		rga,'+'				; Test for a '+'
		breq	gettz3
gettz2:
		cpi		rga,'-'				; Test for a '-'
		brne	gettz4				; No signs found, continue
		cbr		flagb,(1<<TZposf)	; Clear + offset flag
		st		X,rmp				; Stuff '0' to linbuf, overwrite - input
		rjmp	gettz5
gettz3:
		sbr		flagb,(1<<TZposf)	; Set + offset flag
		st		X,rmp				; Stuff '0' to linbuf, overwrite + input
		rjmp	gettz5
;
; Neither + or -, check for 'S'. If not 'S', process TZ offset. Note TZposf set
; as default when no sign entered.
;
gettz4:
		rcall	case
		cpi		rga,'S'				; Save to EEPROM?
		breq	gettzs				; Yes, store to EEPROM
		sbr		flagb,(1<<TZposf)	; Set + offset flag as default
;
; Check linbuf for possible TZ offset entry
;
gettz5:
		rcall	gnum				; Get possible number, set numfl if good
		sbrs	flaga,numfl			; numfl set?
		rjmp	gettzerr			;	No, input error, get new input
		cbr		flaga,(1<<numfl)	;	Yes, clear numfl, valid entry
;
; Load new TZ offset if ready
;
		sbrs	flaga,xclinf		; Keep status line?
		rjmp	gettz6				; Yes, leave status line
		cbr		flaga,(1<<xclinf)
		call	prtzm				; Show TZ message, erase rest of line
;
; Valid TZ offset entry
;
gettz6:
		lds		rmp,dba				; Get new TZ offset
;
; limit value to -12, +14
;
		sbrs	flagb,TZposf		; Limit according to offset sign
		rjmp	gettz7				; Negative offsets
;
		cpi		rmp,15				; Limit check
		brcs	gettz7				;	14 and under
		ldi		rmp,14				;	>= 15
		rjmp	gettz7
;
gettz7:
		cpi		rmp,13				; Limit check
		brcs	gettz8				;	12 and under
		ldi		rmp,12				;	>= 13
;
gettz8:
		sts		TZb,rmp				; Save new TZ offset entry
		rcall	prtzbuf				; Show new TZ value
		rjmp	gettz1
;
; Input error handler
;
gettzerr:
		rcall	slind
		ldzptr	err1				; Input error message
		call	pptr
		sbr		flaga,(1<<xclinf)	; Delayed line clear
		rjmp	gettz1				; Redo input
;
; Save TZoff entry to EEPROM
;
gettzs:
		rcall	slinc
		ldzptr	savEEms
		rcall	pptr				; Show message
		rcall	str_eeprm			; Store TZ offset and TZposf state to EEPROM
		rcall	sak					; Wait for any key input
		rcall	ld_eeprm			; Load TZ offset from EEPROM
		rjmp	gettz				; Main menu

;
;
; Print TZ offset buffer
;
prtzbuf:
		rcall	slina
;
		ldzptr	TZ_offm
		call	pptr
;
; Issue +/- TZ offset sign
;
prtzbuf1:
		ldi		rga,'+'				; Assume positive offset
		sbrs	flagb,TZposf
		ldi		rga,'-'				; Negative '-'
		rcall	co					; Show offset sign
;
		lds		rga,TZb				; TZ buffer contents
		mov		YL,rga
		clr		YH
		rcall	bn2bcd				; Convert to packed BCD
		rcall	p2dg
		ret
;
; Print TZ enter message, clear rest of line
;
prtzm:
		call	slinc
		ldzptr	TZm				; Show TZm command message
		call	pptr
		call	ceol			; Clear to eol
		ret
;
;
;
;###########################################################################
;
;
;************************************************
;
; Store and load TZ offset to EEPROM
;
;************************************************
;
; Registers: rga, rgb, rmp, Y, Z
;
str_eeprm:
;
		ldi		rmp,0xff
		out		EEARL,rmp				; EEPROM start address = -1 as
		out		EEARH,rmp				;  EEAR is pre-decremented
;
str_eeprm1:
		lds		rmp,TZb					; Get source byte from data buffer
		rcall	EEWrSeq					; Write to EEPROM
		mov		rmp,flagb				; Save state of flagb
		rcall	EEWrSeq					; Write to EEPROM
		ret
;
; Load EEPROM TZ 0ffset setting to TZb.
;
ld_eeprm:
		ldi		rmp,0xff
		out		EEARL,rmp				; EEPROM start address = -1
		out		EEARH,rmp
;
ld_eeprm1:
		rcall	EERdSeq					; Get source byte from EEPROM
		cpi		rmp,0xff				; Check for 0xff, EEPROM  erased?
		brne	ld_eeprm2				;	No, data valid
		ldi		rmp,TZoff				;	Yes, use default TZ offset
ld_eeprm2:
		sts		TZb,rmp					; Load buffer <-- rmp
		rcall	EERdSeq					; Get flagb_img byte from EEPROM
		cpi		rmp,0xff				; Check for 0xff, EEPROM  erased?
		breq	ld_eeprm3				; EEPROM erased
		sts		flagb_img,rmp			; Load flagb_img <-- rmp
		ret
ld_eeprm3:
		sts		flagb_img,flagb
		ret
;
		ret
;
;
;
;
;
;###########################################################################
;
;	L C D   P O L L E D   D R I V E R   M O D U L E   R O U T I N E S
;
;###########################################################################
;
; The LCD routines interface to the TWI interface via <twi_wrbyt> routine
;
; ==== LCD Support Routines ====
;
;
; ==================================================
;	Setup TWI Master to slave write to LCD backpack	
; ==================================================
;
; Setup master to slave write to LCD backpack TWI interface
; Master sends START then SLA+W 7 bit slave address plus write bit.
;
; Registers:	rga, rmp
;
; Entry:		rgb = slave address, 7 bits + R/W bit (SLA value)
; Exit:			C= 0 OK, else C=1 on error
;
LCD_start:
		rcall	twi_S			; Send a START
		brcs	LCD_S_err		; Error
		ldi		rgb,LCD_SLA		; Slave address + W
		rcall	LCD_SLAw		; Send slave address + write
		brcs	LCD_S_err		; Error
		ret
;
; On S or SLA+W error, send S, set error flag and exit
;
LCD_S_err:
		ret
;
; Send Slave Address + W. Based on <twi_SLAw> but does not check for NAK.
; This is needed to prevent the LCD driver from hanging up the TWI bus if
; the slave is not on-line.
;
; Entry:		rgb = slave address, 7 bits + R/W bit (SLA value)
;
LCD_SLAw:
		mov		rmp,rgb			; Slave address + W bit, 0
		sts		TWDR,rmp		; Load twi data register
		ldi		rmp,(1<<TWINT)|(1<<TWEN)	; Clear TWINT, enable bus
		sts		TWCR,rmp
LCD_SLAw1:
		lds		rmp,TWCR		; Wait for TWINT flag set, indicating START sent
		sbrs	rmp,TWINT
		rjmp	LCD_SLAw1
;
		lds		rmp,TWSR		; Check status code in status register
		andi	rmp,0xf8		; Mask off prescaler bits
		cpi		rmp,SLAw_ACK	; Slave address write sent?
		brne	LCD_SLAwerr		; Not sent, error condition
		clc						; C=0 if OK
		ret
LCD_SLAwerr:
		sec						; C=1 on error
		ret	
;
; ==== LCD Data Write Routines ====
;
;
; Write data byte in rmp to LCD backpack TWI interface.
;
; Data is written as two 4 bit nibbles, high followed by low nibble.
;
; LCD_RS = 0
; LCD_RW = 0
; LCD_E is toggled high then low to write to LCD.
;
; Registers:	rmp, rgd temporary storage
; Entry:		rmp has data to transmit
;
; Call <LCD_start> to enable TWI to LCD backpack communications.
;
; At end of transmission to session LCD, invoke the TWI STOP <twi_P>
; to release the backpack slave from TWI bus.
;
LCD_wrins:
		push	rmp				; Save data
		andi	rmp,0b11110000	; Mask off control bits
		rcall	LCD_wrnib		; Send high nibble
;	
		pop		rmp				; Fetch data
		swap	rmp				; Swap nibbles
		andi	rmp,0b11110000	; Mask off control bits
		rcall	LCD_wrnib		; Send low nibble
		ret
;
; Write rmp to LCD RAM via LCD backpack TWI interface.
;
; LCD_RS = 1
; LCD_RW = 0
; LCD_E is toggled high then low to write to LCD.
;
; Registers:	rmp, rgd temporary storage
; Entry:		rmp has data to transmit
;
;
LCD_wrdat:
		push	rmp				; Save data
		andi	rmp,0b11110000	; Mask off control bits
		sbr		rmp,(1<<LCD_RS)	; Set RS bit for write to CGRAM
		rcall	LCD_wrnib		; Send high nibble
;	
		pop		rmp				; Fetch data
		swap	rmp				; Swap nibbles
		andi	rmp,0b11110000	; Mask off control bits
		sbr		rmp,(1<<LCD_RS)	; Set RS bit for write to CGRAM
		rcall	LCD_wrnib		; Send low nibble
		ret
;
; Write the byte passed in rmp to TWI in two nibbles, high nibble
; followed by low nibble. Data passed in (rmp) to LCD_wrnib TWI
; low nibble is ignored, only high nibble is used. The low nibble is
; replaced by control bits for LCD, LCD_RS, LCD_RW, LCD_E and BT bits.
;
; Registers:	rmp, rgd
; Entry:		rmp = (data_nibble.00000)
;
LCD_wrnib:
		bst		flagb,LCD_LEDb	; T <-- LCD_LEDb backlight flag
		bld		rmp,LCD_BT		; rmp(LCD_BT) <-- T
		push	rmp				; Save a copy
		rcall	twi_wrbyt		; Write high nibble to TWI bus
		pop		rmp
		push	rmp
		sbr		rmp,(1<<LCD_E)	; Set E bit
		rcall	twi_wrbyt		; Write to TWI bus
;
		pop		rmp
		cbr		rmp,(1<<LCD_E)	; Clear E
		rcall	twi_wrbyt		; Write to TWI bus
		ret
;
;
; ==== LCD Control Routines ====
;
;
; Turn on LCD LED backlight by flag setting flag LCD_LEDb = 1
;
LCD_ledon:
		sbr		flagb,(1<<LCD_LEDb)	; Set backlight flag
		ret
;
; Turn off LCD LED backlight by flag clearing flag LCD_LEDb = 0
;
LCD_ledoff:
		cbr		flagb,(1<<LCD_LEDb)	; Clear backlight flag
		ret
;
;
; Home cursor on LCD display (1.64 ms execution time max).
;
LCD_home:
		ldi		rmp,0b00000010		; Home cursor command
		rcall	LCD_wrins			; Master to LCD slave
		rcall	d1728us
		ret
;
;  Clear display and home cursor
;
LCD_clear:
		ldi		rmp,0b00000001		; Clear and home cursor command
		rcall	LCD_wrins			; Master to LCD slave
		rcall	d1728us
		ret
;
;  Display OFF
;
LCD_off:
		ldi		rmp,0b00001000		; Display off command
		rcall	LCD_wrins			; Master to LCD slave
		rcall	d64us				; 100 us delay
		ret
;
;  Display ON
;
LCD_on:
		ldi		rmp,0b00001100		; Display on command
		rcall	LCD_wrins			; Master to LCD slave
		rcall	d64us				; 100 us delay
		ret
;
;  Shift LCD display left
;
LCD_left:
		ldi		rmp,0b00011000		; Display shift left command
		rcall	LCD_wrins			; Master to LCD slave
		rcall	d64us				; 100 us delay
		ret
;
;  Shift LCD display right
;
LCD_right:
		ldi		rmp,0b00011100		; Display shift right command
		rcall	LCD_wrins			; Master to LCD slave
		rcall	d64us				; 100 us delay
		ret
;
; Set display RAM address to 00, line 1 start
;
LCD_line1:
		ldi		rmp,0b10000000		; Display address 00 command
		rcall	LCD_wrins			; Master to LCD slave
		rcall	d64us				; 64 us delay
		ret
;
; Set display RAM address to 64, line 2 start
;  
LCD_line2:
		ldi		rmp,0b11000000		; Display address 64 command
		rcall	LCD_wrins			; Master to LCD slave
		rcall	d64us				; 64 us delay
		ret
;
; Set display RAM address to 78, line
;  
LCD_c75:
		ldi		rmp,0b11001110		; Display address 78 command
		rcall	LCD_wrins			; Master to LCD slave
		rcall	d64us				; 64 us delay
		ret
;
; Write message pointed to by Z, ctrlZ terminated, to LCD display.
;
LCD_wrln:
		push	rmp
LCD_wrln1:
		lpm		rmp,Z+			; String byte to rga, Z+
		cpi		rmp,ctlZ		; byte ^Z?
		brne	LCD_wrln2		; Print if not ^Z
		pop		rmp
		ret
LCD_wrln2:
		cpi		rmp,NULL		; Skip any nulls in string
		breq	LCD_wrln1
		rcall	LCD_wrdat
		rjmp	LCD_wrln1
;
; Display character passed in rmp as two hexadecimal digits
;
; Registers:	rmp, rga
;
LCD_pahex:
		push	rmp				; Save data
		swap	rmp				; Show high nibble (MSD) first
		rcall	LCD_pahex1
		pop		rmp
;
LCD_pahex1:
		andi	rmp,0x0f		; Mask off high nibble
		ldi		rga,0x30		; '0'
		add		rmp,rga			; Convert hex number to character
		cpi		rmp,0x3a		; Subtract ':' to check if > 9?
		brcs	LCD_pahex2		;  No, it is 0 ... 9
		ldi		rga,7			;  Yes, convert to A ... F
		add		rmp,rga
LCD_pahex2:
		rcall	LCD_wrdat		; Write to LCD hex digit
		ret
;
; Send a ':' character to LCD display
;
LCD_colon:
		ldi		rmp,0x3a		; ':' character
		rcall	LCD_wrdat
		ret
;
; Position cursor right to column (1 to 20). Cursor position specified in
; register rga.
;
; User calls HOME or sets line 1 or 2 by SLN1 or SLN2 prior to calling LCD_cur.
;
; Registers:	rmp, rga
;
LCD_cur:
		dec		rga				; Cursor starts at position 1
LCD_cur1:
		rcall	LCD_right
		dec		rga				; Count cursor position
		brne	LCD_cur1
		ret
;
; Send a space character to LCD display
;
LCD_space:
		ldi		rmp,0x20		; Space character
		rcall	LCD_wrdat
		ret
;
; Send 3 spaces to LCD display
;
LCD_3space:
		ldi		rga,3
LCD_3sp:
		ldi		rmp,0x20		; Space character
		rcall	LCD_wrdat
		dec		rga
		brne	LCD_3sp
		ret
;
; Turn off LCD cursor
;
LCD_cur_off:
		ldi		rmp,0x0c		; No cursor
		rcall	LCD_wrins		; Master to LCD slave
		ret
;
;
; Clean up LCD lines 1 and 2
;
LCD_clean:
		rcall	LCD_line2
		ldzptr	blankm				; Erase line
		rcall	LCD_wrln
		rcall	LCD_line1
		ldzptr	blankm				; Erase line
		rcall	LCD_wrln
		ret
;
;
;###########################################################################
;
;		T W I   P O L L E D   D R I V E R   M O D U L E
;
;###########################################################################
;
;
; ============
;	TWI Start
; ============
;
; Send start condition. Atomic operation. Usually the first twi operation to
; start a twi master read/write sequence.
;
twi_S:
		ldi		rmp,(1<<TWINT)|(1<<TWSTA)|(1<<TWEN)	; Clear TWINT, set START, enable bus
		sts		TWCR,rmp
twi_S1:
		lds		rmp,TWCR		; Wait for TWINT flag set, indicating START sent
		sbrs	rmp,TWINT
		rjmp	twi_S1
;
		lds		rmp,TWSR		; Check status code in status register
		andi	rmp,0xf8		; Mask off prescaler bits
		cpi		rmp,STARTC		; START condition sent?
		breq	twi_S2			;	Yes
		cpi		rmp,RSTART		; Repeated start condition sent?
		brne	twi_error		; Not sent, error condition
twi_S2:
		clc						; C=0 if OK
		ret
;
twi_error:
		sec						; C=1 on error
		ret
;
; ===========================
;	TWI Repeated Start	(OK)
; ===========================
;
; Send repeated start condition. Atomic operation. 
;
twi_RS:
		ldi		rmp,(1<<TWINT)|(1<<TWSTA)|(1<<TWEN)	; Clear TWINT, set START, enable bus
		sts		TWCR,rmp
twi_RS1:
		lds		rmp,TWCR		; Wait for TWINT flag set, indicating RSTART sent
		sbrs	rmp,TWINT
		rjmp	twi_RS1
;
		lds		rmp,TWSR		; Check status code in status register
		andi	rmp,0xf8		; Mask off prescaler bits
		cpi		rmp,RSTART		; START condition sent?
		brne	twi_RSerr		; Not sent, error condition
		clc						; C=0 if OK
		ret
;
twi_RSerr:
		sec						; C=1 on error
		ret
;
		ret	
;
; =========================
;	TWI Stop	(OK)
; =========================
;
; Send stop condition. Atomic operation
;
twi_P:
		ldi		rmp,(1<<TWINT)|(1<<TWSTO)|(1<<TWEN)
		sts		TWCR,rmp
		ret
;
; Send one data byte to slave. Atomic operation.
; Sends a byte, waits for and checks for ACK, C=1 on error
; else C=0 if all OK.
;
; Entry:	rmp = data to write to TWDR
; 
twi_wrbyt:
		sts		TWDR,rmp		; Load twi data register
		ldi		rmp,(1<<TWINT)|(1<<TWEN)	; Clear TWINT, enable bus
		sts		TWCR,rmp
twi_wrbyt1:
		lds		rmp,TWCR		; Wait for TWINT flag set, indicating byte sent
		sbrs	rmp,TWINT
		rjmp	twi_wrbyt1
;
		lds		rmp,TWSR		; Check status code in status register
		andi	rmp,0xf8		; Mask off prescaler bits
		cpi		rmp,DATAw_ACK	; Data sent?
		brne	twi_wrerr		; Not sent, error condition
		clc						; C=0 if OK
		ret
;
; On TWI read error, set RTC_error flag bit and exit
;
twi_wrerr:
		sec
		ret		
;
;
;
;
;###########################################################################
;
; Timers - general purpose
;
;
; Start 100 ms timer
;
start_t100m:
		ldi		rmp,24
		mov		rtmr,rmp
		cbr		flagb,(1<<t100mf)
		ret
;
; 10 us delay	(OK)
;
d_10u:
;
.if		Vcc_low
		ldi		rgc,24				; 10.1 us delay
.else
		ldi		rgc,50
.endif
;
d_10u1:
		dec		rgc
		brne	d_10u1
		ret
;
; 15 us delay	(OK)
;
d_15u:
;
.if		Vcc_low
		ldi		rgc,38
.else
		ldi		rgc,75
.endif
;
d_15u1:
		dec		rgc
		brne	d_15u1
		ret
;
; 130 us delay	(OK)
;
d_130u:
		ldi		rgb,13
d_130u1:
		rcall	d_10u
		dec		rgb
		brne	d_130u1
		ret
;
; 1 ms delay
;
d_1m:
		ldi		rgb,100
d_1m1:
		rcall	d_10u
		dec		rgb
		brne	d_1m1
		ret
;
; Various time delays used while accessing the LCD
; One-shot Timers based on TCNT1 interrupt handler, 64 us interrupt rate.
; 
;
d64us:
;
; Initialize OCR1A output compare register with new reload values
;
		ldi		rmp,high(OCR1A64us)		; Set OCR1A = 64 us period
		ldi		rga,low(OCR1A64us)
		sts		OCR1AH,rmp				; 16 bit write
		sts		OCR1AL,rga
;
		cbr		flaga,(1<<tcnt1fa)			; Clear timer flag in case it is set
		lds		rmp,TCCR1B					; Fetch control register
		sbr		rmp,(1<<CS12)|(1<<CS10)		; Start timer by setting CS12 & CS10
		sts		TCCR1B,rmp
d64us1:
		sbrs	flaga,tcnt1fa
		rjmp	d64us1
		ret
;
; 1.73 ms delay timer 
;
d1728us:
;
; Initialize OCR1A output compare register with new reload values
;
		ldi		rmp,high(OCR1A1728us)		; Set OCR1A = 1728 us period
		ldi		rga,low(OCR1A1728us)
		sts		OCR1AH,rmp					; 16 bit write
		sts		OCR1AL,rga
;
		cbr		flaga,(1<<tcnt1fa)			; Clear timer flag in case it is set
		lds		rmp,TCCR1B					; Fetch control register
		sbr		rmp,(1<<CS12)|(1<<CS10)		; Start timer by setting CS12 & CS10
		sts		TCCR1B,rmp
d1728us1:
		sbrs	flaga,tcnt1fa
		rjmp	d1728us1
		ret
;
; 5 ms delay timer 
;
d5ms:
;
; Initialize OCR1A output compare register with new reload values
;
		ldi		rmp,high(OCR1A5ms)			; Set OCR1A = 100 ms period
		ldi		rga,low(OCR1A5ms)
		sts		OCR1AH,rmp					; 16 bit write
		sts		OCR1AL,rga
;
		cbr		flaga,(1<<tcnt1fa)			; Clear timer flag in case it is set
		lds		rmp,TCCR1B					; Fetch control register
		sbr		rmp,(1<<CS12)|(1<<CS10)		; Start timer by setting CS12 & CS10
		sts		TCCR1B,rmp
d5ms1:
		sbrs	flaga,tcnt1fa
		rjmp	d5ms1
		ret
;
; 100 ms delay timer 
;
d100ms:
;
; Initialize OCR1A output compare register with new reload values
;
		ldi		rmp,high(OCR1A100ms)	; Set OCR1A = 100 ms period
		ldi		rga,low(OCR1A100ms)
		sts		OCR1AH,rmp				; 16 bit write
		sts		OCR1AL,rga
;
		cbr		flaga,(1<<tcnt1fa)			; Clear timer flag in case it is set
		lds		rmp,TCCR1B					; Fetch control register
		sbr		rmp,(1<<CS12)|(1<<CS10)		; Start timer by setting CS12 & CS10
		sts		TCCR1B,rmp
d100ms1:
		sbrs	flaga,tcnt1fa
		rjmp	d100ms1
		ret
;
; General 0.5 s timer
;
d500ms:
		ldi		rgb,5			; 5 x 0.1 s = 0.5 s
		rjmp	d1s1
d1s:
		ldi		rgb,10			; 10 x 0.1 s = 1.0 s
d1s1:
		call	d100ms
		dec		rgb
		brne	d1s1
		ret
;
;
; Zero RAM data space
;
zbuf:
		ldi		rgb,(enbuf-stbuf)
		ldxptr	stbuf			; Start at linbuf
		clr		rmp
zbuf1:
		st		X+,rmp
		dec		rgb
		brne	zbuf1
		ret
;
; Zero lower registers R0...R15
;
zregs:
		ldi		rga,16
		clr		rmp
		ldxptr	0x0			; Register file base address
zregs1:
		st		X+,rmp
		dec		rga
		brne	zregs1
		ret
;
; Print 5 digits in res2:res1:res0 stored as packed bcd		(OK)
;
p5dg:
		mov		rga,res2		; Fetch 1st of 3 bytes
		rcall	pdg				; Show digit 5 only
;
; Print 4 digits in res2:res1:res0 stored as packed bcd		(OK)
;
p4dg:
		mov		rga,res1		; Fetch 2nd of 3 bytes
		call	pacc			; Show digits 4,3
;
; Print 2 digits in res2:res1:res0 stored as packed bcd		(OK)
;
p2dg:
		mov		rga,res0		; Fetch 3rd of 3 bytes
		call	pacc
		ret
;
; Print 3 digits in res2:res1:res0 stored as packed bcd		(OK)
;
p3dg:
		mov		rga,res1		; Fetch 2nd of 3 bytes
		call	pdg				; Show digit 3 only
		rjmp	p2dg			; Show digits 2,1
;
; Convert packed BCD digits in rga to ascii and display		(OK)
;
pacc:
		mov		asav,rga		; Save the data
		swap	rga				; Hi nibble first
		call	pdg				; Convert to ascii and display
		mov		rga,asav
		call	pdg				; Show lo nibble
		ret
;
; Display bcd low nibble as ascii digit			(OK)
;
pdg:
		andi	rga,0x0f		; Mask hi nibble
		ldi		rmp,'0'			; Convert by adding '0'
		add		rga,rmp
		call	co				; Show the digit
		ret
;
;
;###########################################################################
;
; --- Conversion and testing routines ---
;
;  Convert rga to upper case
;
case:
		cpi		rga,0x61		; Ascii 'a'
		brcs	case1			; < a
		cpi		rga,0x7b		; Ascii 'z'
		brcc	case1			; > z
		clc
		sbci	rga,SP			; Subtract SP to convert to UC
case1:
		ret
;
;
; gnum used to process possible good ascii number from linbuf, convert it to		(OK)
; binary number in YH:YL using gdbn.
;
; Save data byte in DBA if number less than 256, or data word
; in WDBUF if number is 256 to 65,535. The numfl is set if the input
; data is good byte value. User to clear numfl after test.
;
; Registers:	flaga, rgv, YH, YL, rmp, XH:XL
;
gnum:
		cbr		flaga,(1<<numfl)	; Clear the good number flag
		mov		rgv,count			; Load counter with number of characters in linbuf
		cpi		rgv,ndec+1			; Compare to limit
		brcc	gnum2				; Too many digits error
		call	gdbn				; Convert to binary in YH:YL
		brcc	gnum2				; Error
		ldxptr	wdbuf				; Point to data word buffer
		st		X+,YH				; Save result high byte
		st		X+,YL				; Save result low byte
		st		X,YL				; Save result low to data byte buffer
		tst		YH					; Test result high byte if zero
		breq	gnum1				;	Yes, word value
		ldi		rmp,0xff			;	No, cap byte value to 0xff
		st		X,rmp				;   Save in dba
gnum1:
		sbr		flaga,(1<<numfl)	; Mark as byte value
gnum2:
		ret
;
; Convert ASCII decimal string in LINBUF to binary number in  YH:YL.		(OK)
; The number to be converted is maximum allowed 5 ascii digits long, 
; set by ndec, equivalent to (0xffff) binary number.
;
; This routine called by gnum to convert ascii numbers for data entry
;
; Entry: rgv = ndec, number of ascii digits to convert
; Exit:  YH:YL <-- 16 bit result if ok, C = 1
;        YH:YL <-- 00, C = 0 if error
; Regs:  rga, rgb, rgc, YH, YL, rgv, XH:XL
;
gdbn:
		clr		YH			; Clear result registers
		clr		YL
		ldxptr	linbuf		; Setup line buffer pointer
gdbn1:
		ld		rga,X+		; Fetch a character
		call	decdg		; Convert to BCD
		brcc	gdbnx		; Error exit
		mov		asav,rga	; Save character
		call	dex10		; Value * 10, result in YH:YL
		brcs	gdbnov		; Overflow
;
		mov		rgc,asav	; Add original digit in
		clr		rgb
		call	adebc		; YH:YL = YH:YL + rgb:rgc
		brcs	gdbnov		; Overflow error
		dec		rgv			; All characters processed?
		brne	gdbn1		;	No, continue
		sec
		ret					;	Yes, normal exit
;
gdbnx:
		clc					; Error exit
		clr		YH
		clr		YL
		ret
;
gdbnov:
		sec					; Overflow condition
		ldi		YH,0xff
		ldi		YL,0xff	; Limit to 0xFFFF
		ret
;
; Convert ASCII 0.....9 to BCD, C = 1 if ok, else		(OK)
; C = 0 and rga unchanged
; Registers:	rga, asav
;
decdg:
		mov		asav,rga	; Save ascii digit
		subi	rga,'0'		; Char less than char '0'?
		brcs	ddgx		;	Yes, error exit
		cpi		rga,LF		;  Char from 0...9?
		brcc	ddgx		;	No, error exit
		ret					; Is 0...9
ddgx:
		clc					; Not 0...9
		mov		rga,asav
		ret
;
; Convert 16 bit binary in YH:YL to packed bcd in res2:res1:res0		(OK)
;
; Registers: rgb, rgc, YH, YL, res0, res1, res2
;
bn2bcd:
		ser		rgc					; rgc = 0xff
		mov		res2,rgc
;
; Process 10,000's digit
;
cvde_L10k:
		inc		res2
		subi	YL,low(10000)
		sbci	YH,high(10000)
		brcc	cvde_L10k			; Loop until C set
		subi	YL,low(-10000)		; Correct last subtraction
		sbci	YH,high(-10000)
		ldi		rgb,(256-16)
;
; Process 1000's digit
;
cvde_L1k:
		subi	rgb,(-16)
		subi	YL,low(1000)
		sbci	YH,high(1000)
		brcc	cvde_L1k			; Loop until C set
		subi	YL,low(-1000)		; Correct last subtraction
		sbci	YH,high(-1000)
		mov		res1,rgc
;
; Process 100's digit
;
cvde_L100:
		inc		res1
		subi	YL,low(100)
		sbci	YH,high(100)
		brcc	cvde_L100			; Loop until C set
		subi	YL,low(-100)		; Correct last subtraction
		or		res1,rgb
		ldi		rgb,(256-16)
;
; Process 10's digit
;
cvde_L10:
		subi	rgb,(-16)
		subi	YL,10
		brcc	cvde_L10			; Loop until C set
		subi	YL,-10				; Correct last subtraction
		mov		res0,rgb
		or		res0,YL
		ret
;
;
; Convert rga to hex digit and set C, else clear C if character
; not an ascii hexadecimal digit. On error, return with character in rga
;
; Registers:	rga
;
hexdg:
		mov		asav,rga	; Save char
		rcall	case		; Fold to UC
		subi	rga,'0'		; rga < '0'?
		brcs	hexdg1		;	Yes, exit
		cpi		rga,LF		; rga from 0...9?
		brcs	hexdg2		;	Yes
		subi	rga,7		; Dump funny chars
		cpi		rga,LF		; Char from 9...A?
		brcs	hexdg1
		cpi		rga,0x10	; Char above F?
		brcc	hexdg1		;	Yes
hexdg2:
		ret					; Normal exit, C=1
hexdg1:
		mov		rga,asav	; Restore char
		clc
		ret
;
;
;###########################################################################
;
;
; --- Line input and initialization routines ---	(OK)
;
;  An 8 byte line input buffer is supported. The buffer is initially
;  cleared to zeroes, and pointed to by XH:XL. COUNT maintains a
;  count of characters entered. Entry is terminated by <'CR'>, <^X> 
;  erases current line and starts over, and <BS> or <DEL> erases
;  previous character. XH:XL is reserved for use as LINBUF pointer
;  to allow multiple GCHR calls.
;
;	Registers used:
;	rmp, rga, rgb, rgc, X
;
glin:
		rcall	inzln			; Zero the line buffer and count register
glin1:
		rcall	ci				; Get a character
		cpi		rga,CR			; Test if <CR>
		brne	glin5			;	No, look for next special key
		ldxptr	linbuf			;	Yes, reset linbuf pointer
		sbr		flaga,(1<<crf)	; And set CR flag
		ret
;
; Look for a ^X key, if so do a line delete
;
glin2:
		cpi		rga,ctlX		; Test if <^X>
		brne	glin3			;	No, look for next special key
		mov		rgb,count		; Load character counter
		tst		rgb				; Count = 0?
		breq	glin
glin2a:
		call	bksp			; Move cursor back one space
		dec		rgb
		brne	glin2a			; back to start
		rjmp	glin			; Restart
;
; Look for a BS key, if so do a delete character at cursor
;
glin3:
		cpi		rga,BS			; Test if backspace
		brne	glin5			;	No, look for next special key
glin3a:
		mov		rgb,count		; Load character counter
		tst		rgb				; Count = 0?
		breq	glin1			;	Yes, fetch another character
		dec		rgb
		mov		count,rgb
		call	bksp			; Move cursor back one space
		ldi		rmp,0			; Backup pointer and insert 0 
		st		-X,rmp
		rjmp	glin1
;
; Look for a Tab key, if so expand tab to spaces
;
glin5:
		cpi		rga,HT			; Test if tab
		brne	glin6			;	No,  look for next special key
		ldi		rgc,7			; Temp counter
		ldi		rga,SP			; Space character
glin5a:
		rcall	ldlin
		dec		rgc
		brne	glin5a
		rjmp	glin1
;
; Look for a Escape key, if so set escf
;
glin6:
		cpi		rga,ESC			; Test if esc
		brne	glin7			;	No, look for other control key
		sbr		flaga,(1<<escf)	; Set esc flag
		ret
;
; Look for other control key.
;
glin7:
		rcall	fctl			; Test for other control key
		sbrs	flaga,kyf
		rjmp	glin8			;	kyf = 0
		ret						;	kyf = 1
;
; Arrive here is valid key entry
;
glin8:
		rcall	ldlin			; Load the input buffer and show
		rjmp	glin1
;
; Load character in rga to LINBUF, update pointer and character counter		(OK)
;
ldlin:
		mov		rgb,count		; Get current count
		cpi		rgb,linsz		; End of buffer?
		brne	ldlin1			;	No
		ret						;	Yes, exit
ldlin1:
		inc		rgb
		mov		count,rgb		; Update count
		st		X+,rga			; Store entered key to buffer
		rcall	co				; Show it
		ret
;
;  Get linbuf character, increment XH:XL pointer and set C if
;  not 'CR', else clear C, rga = 0. 
;
gchr:
		ld		rga,X+			; Get character from line buffer, advance pointer
		cpi		rga,0			; Test for 0
		brne	gchr1			;	rga >= 0, means ascii printable character
		clc
		ret
gchr1:
		sec
		ret
;
; Clear input line buffer	(OK)
;
inzln:
		clr		rmp				; Fill byte
		clr		count			; Initialize count to 0
		ldi		rgb,linsz		; Buffer size
		ldxptr	linbuf			; Point to line buffer
inzln1:
		st		X+,rmp
		dec		rgb
		brne	inzln1
		ldxptr	linbuf			; Point to line buffer
		cbr		flaga,(1<<crf)|(1<<escf)	; Clear exit flags
		ret
;
;  Test rga for control key, 0...19H, 7FH..FFH, and set KYF		(OK)
;  if true, else clear KYF. rga preserved
;
fctl:
		sbr		flaga,(1<<kyf)
		cpi		rga,SP				; rga < SP?
		brcs	fctl1				;	Yes
		cpi		rga,DEL				; rga >= SP?
		brcc	fctl1				;	No
		cbr		flaga,(1<<kyf)		; Clear kyf
fctl1:
		ret
;
;
; --- Data Buffer control and Math routines ---
;
; Multiply YH:YL by 10, called by gdbn ascii to binary converter routine		(OK)
; YH:YL = YH:YL * 10, C = 0 if ok, C = 1 on error
; Registers:	rga, rgb, rgc, YH, YL
;
dex10:
		call	dex2		; YH:YL * 2
		brcs	dexx		; Error exit, overflow and C=1
		push	YH			; Copy YH:YL to rgb:rgc
		pop		rgb
		push	YL
		pop		rgc
;
		rcall	dex2		; * 4
		brcs	dexx
		rcall	dex2		; * 8
		brcs	dexx
		call	adebc		; YH:YL = YH:YL + rgb:rgc
dexx:
		ret
;
; YH:YL: = YH:YL * 2
; 
dex2:
		clc
		rol		YL
		rol		YH
		ret
;
; YH:YL = YH:YL + rgb:rgc, C = 0 if ok else C = 1 on overflow
;
adebc:
		add		YL,rgc
		adc		YH,rgb
		ret
;
; Decrement XH:XL pointer by 1		(OK)
;
decxptr:
		push	rmp
		ldi		rmp,-1
		add		XL,rmp
		adc		XH,rmp
		pop		rmp
		ret
;
; "div8u" - 8/8 Bit Unsided Division				(OK)
;
; This subroutine divides the two register variables "rga" (dividend) and 
; "rgb" (divisor). The result is placed in "rga" and the remainder in "rgb".
;  
; High registers used:	4 (rga,rgb,rgc,rgv)
;
;                                  
; Register Variables:
;	rgc	remainder
;	rga	dividend & result
;	rgb divisor
;	rgv	loop counter
;
; Entry:	(rga) = dividend
;			(rgb) = divisor
; Exit:		(rga) = integer part of quotient
;			(rgb) = integer remainder 
;                                    
div8u:	
		push	rgc
		push	rgv
		sub		rgc,rgc			; clear remainder and carry
        ldi		rgv,9			; init loop counter
d8u_1:	rol		rga				; shift left dividend
        dec		rgv				; decrement counter
        brne	d8u_2			; if done
		mov		rgb,rgc			; move remainder to rgb
		pop		rgv
		pop		rgc
        ret						;    return
;
d8u_2:	rol		rgc				; shift dividend into remainder
        sub		rgc,rgb			; remainder = remainder - divisor
        brcc	d8u_3			; if result negative
        add		rgc,rgb			;    restore remainder
        clc						;    clear carry to be shifted into result
        rjmp	d8u_1			; else
d8u_3:	sec						;    set carry to be shifted into result
        rjmp	d8u_1
;
;
;###########################################################################
;
;
; --- General Screen Routines ---
;
slina:
		call	pxy
		.db		4,15
		rjmp	slin
slinb:
		call	pxy				; Cursor to messagess line B
		.db		8,2				; Cursor to status line A
		rjmp	slin
slinc:
		call	pxy				; Cursor to messagess line C
		.db		10,2
		rjmp	slin
slind:
		call	pxy				; Cursor to messagess line C
		.db		10,45
;
slin:
		call	ceol			; Clear the rest of line
		ret
;
; Clear display screen
;
clean:
		call	pxy				; Wipe screen clean
		.db		2,1
		call	clin			; Clear lines
		.dw		15
		call	pxy
		.db		2,1
		ret
;
; Clear lines specified immediately following rcall to clin
;
clin:
	pop		ZH				; Point to data word
	pop		ZL
	lsl		ZL				; Z*2 for word address
	rol		ZH
	andi	ZL,0xfe			; Fetch lower byte of word
	lpm		rgb,Z+			; Get word
	adiw	Z,1
	lsr		ZH
	ror		ZL				; Z/2
	push	ZL				; Return address to stack
	push	ZH
clin1:
	call	ceol			; Clear lines
	call	cdown			; Move cursor down 1 row
	dec		rgb
	brne	clin1
	ret
;
; Display 'Enter:' prompt
;
enter:
		call	slinb
		call	ceol
		ldzptr	entm
		call	pptr
		ret
;
sak:
		call	slind
		ldzptr	sakm			; Strike any key
		call	pptr
		call	ci				; Wait for any key
		ret
;
; --- Video routines ---
;
;
; --- Low level video drivers ---
;
; Register rga used to pass data to console output routine
;
; Print rga data as two hexadecimal digits.			(OK)
;
pahex:
	push	rga
	swap	rga				; Show MSD nibble first
	rcall	pahex1
	pop		rga
pahex1:
	andi	rga, 0x0f		; Mask off higher nibble
	ldi		rgv, 0x30 		; Add ascii '0' to convert
	add		rga, rgv		; Convert to ascii
	cpi		rga, 0x3a		; Check if > 9
	brcs	pahex2			;  No, it is 0 ... 9
	ldi		rgv, 0x07		;  Yes, convert to A ... F
	add		rga, rgv
pahex2:
	call	co
	ret
;
; Print rga contents as decimal (0...255). Leading			(OK)
; zero suppression is provided only on the 100's
; digit, so at least two digits are always printed.
;
; Registers rga, rgb not saved
;
pdec:
	ldi		rgb,100			; Get 100's digit
	call	div8u
	tst		rga				; Do leading zero suppression
	breq	pdec1
	call	pnum
pdec1:
	ldi		rga,10			; Get 10's digit
	xchreg	rga,rgb
	call	div8u			; rgb has units
	call	pnum
	xchreg	rga,rgb
pnum:
	ori		rga,0x30		; Ascii "0"
	call	co				; Show ascii decimal
	ret
;
; Scan for keyboard input and return char in rga if any,
; else rga=0.
;
getc:
	lds		rmp,UCSR0A		; Get UART control status register
	sbrs	rmp,RXC0		; Test receiver complete flag
	rjmp	getc1
	lds		rga,UDR0		; rga <-- UDR0
	ret
getc1:
	clr	rga
	ret
;
; Load rga from UDR0 register. Waits until data byte is received.		(OK)
;
ci:	
	lds		rmp,UCSR0A		; Get UART control status register
	sbrs	rmp,RXC0		; Test receiver complete flag
	rjmp	ci
;
; Fetch data
;
	lds		rga,UDR0		; rga <-- UDR0
	ret
;
; Load UDR0 from rga. Wait until transmitter is empty before loading.		(OK)
;
co:	
	lds		rmp,UCSR0A		; Get UART control status register
	sbrs	rmp,UDRE0		; Test if UDR0 is empty
	rjmp	co
;
; Send data
;
	sts		UDR0,rga		; UDR0 <-- rga
	ret
;
; Print CR and LFs	(OK)
;
crllf:
	rcall	crlf			; Two CRLF
crlf:
	push	rga
	ldi		rga,CR			; Carriage return
	call	co
	ldi		rga,LF			; Linefeed
	call	co
	rjmp	cco
;
; Print spaces	(OK)
;
dblsp:
	call	space
space:
	push	rga
	ldi		rga,SP			; Space
cco:
	call	co
	pop		rga
	ret
;
; Print comma	(OK)
;
prcma:
	push	rga
	ldi		rga,cma
	rjmp	cco
;
; Print delete character at cursor	(OK)
;
bksp:
	push	rga
	call	cbak			; Delete character at cursor
	call	ceol			; Clear cursor to end of line
	pop		rga
	ret
;
; Print message string, ^Z terminated. Routine is called with		(OK)
; code address of string loaded in ZH:ZL.
;
pptr:
	push	rga
pptr1:
	lpm		rga,Z+			; String byte to rga, Z+
	cpi		rga,ctlZ		; byte ^Z?
	brne	pptr2			; Print if not ^Z
	pop		rga
	ret
pptr2:
	cpi		rga,NULL		; Skip any nulls in string
	breq	pptr1
	rcall	co
	rjmp	pptr1
;
;
; --- Video and Cursor control routines ---
;
; Clear screen	(OK)
;
clrscn:
	push	zh
	push	zl
	ldzptr	scrn		; Home cursor
	call	pptr
	ldzptr	clrs		; Clear entire screen
	rjmp	video
;
; --- Move cursor down ---
;
cdown:
	push	zh
	push	zl
	ldzptr	cudn		; Cursor down one row
	rjmp	video
;
; --- Clear to end of line ---
;
ceol:
	push	zh
	push	zl
	ldzptr	eol			; Clear to end of line
	rjmp	video
;
; --- Cursor back one column ---
;
cbak:
	push	zh
	push	zl
	ldzptr	cubk			; Cursor back 1 column
	rjmp	video
;
; --- Highlight on ---
;
vhi:
	push	zh
	push	zl
	ldzptr	hi			; Highlight on
	rjmp	video
;
; --- Normal ---
;
vlo:
	push	zh
	push	zl
	ldzptr	lo			; Normal - attributes off
	rjmp	video
;
; --- Reverse ---	(OK)
;
vrev:
	push	zh
	push	zl
	ldzptr	rev			; Reverse on
video:
	rcall	pptr
	pop		zl
	pop		zh
	ret
;
; --- Video position cursor sequences ---
; Lead-in sequence
;
vpxy1:
	push	zh
	push	zl
	ldzptr	pxy1			; Lead-in sequence
	rjmp	video
;
; Middle sequence
;
vpxy2:
	push	zh
	push	zl
	ldzptr	pxy2			; Middle sequence
	rjmp	video
;
; End sequence
;
vpxy3:
	push	zh
	push	zl
	ldzptr	pxy3			; Trailing sequence
	rjmp	video
;
; --- Save cursor position ---
;
vscp:
	push	zh
	push	zl
	ldzptr	scp			; Save cursor position
	rjmp	video
;
; --- Restore cursor position ---
;
vrcp:
	push	zh
	push	zl
	ldzptr	rcp					; Restore cursor position
	rjmp	video
;
; --- Position cursor at row, column immediately following rcall to pxy ---
;
; Row & column values must be given as ascii decimal.			(OK)
;
pxy:
	call	vpxy1			; Lead-in sequence
	pop		ZH				; Point to string start address
	pop		ZL
	clc
	rol		ZL				; 16 bit multiply by 2 for word address
	rol		ZH
;
	lpm		rga,Z+			; Pick up row value
	call	pdec			; Print it and ..
	call	vpxy2			; Middle sequence		+++++ Uses Z pointer, must save Z +++
	lpm		rga,Z+			; Pick up column value
	call	pdec			; Print it and ..
	call	vpxy3			; End sequence
;
	clc
	ror		ZH
	ror		ZL
	push	ZL				; Return to caller
	push	ZH
	ret
;
; Position cursor at (YH)-->row, (YL)-->col		(OK)
;
gotoxy:
	call	vpxy1			; Send lead-in string
	mov		rga,YH			; Get row value
	call	pdec			; Send row
	call	vpxy2			; Send middle string
	mov		rga,YL			; Get col value
	call	pdec			; Send col
	call	vpxy3			; Send trailing string
	ret
;
;
; --- Message strings data area ---
;
; Terminal control sequences
;
cudn:	.db	ESC,"[B",ctlZ		; Move cursor down
cubk:	.db	ESC,"[D",ctlZ		; Cursor back one column
scrn:	.db	ESC,"[H",ctlZ		; Home cursor
eos:	.db	ESC,"[0J",ctlZ		; Clear from cursor to end of screen
clrs:	.db	ESC,"[J",ctlZ		; Clear entire screen
eol:	.db	ESC,"[K",ctlZ		; Erase to end of line
hi:		.db	ESC,"[1m",ctlZ		; Highlight on
lo:		.db	ESC,"[m",ctlZ		; Normal - attributes off
rev:	.db	ESC,"[7m",ctlZ		; Reverse on
pxy1:	.db	ESC,"[",ctlZ		; Lead-in sequence
pxy2:	.db	";",ctlZ			; Middle sequence
pxy3:	.db	"H",ctlZ			; Trailing sequence
dlc:	.db	ESC,"[1M",ctlZ		; Delete line at cursor
scp:	.db	ESC,"7",ctlZ		; Save cursor position
rcp:	.db	ESC,"8",ctlZ		; Restore cursor position
;
;
;
; --- Message strings data area ---
;
;
;###########################################################################
;
.include	"eeprom_module.asm"
;
;###########################################################################
;
; --- Screen displays ---
;
scrnm:
			.db		"  ======<<< m328-GPS-clock module >>>======",cr,lf,ctlZ
thdr:
			.db		"Time       Date",ctlZ
thdr1:
			.db		"========== ==========",ctlz
linem:
			.db		"--------------------------------------------",ctlZ
;
entm:		.db		"Enter: ",ctlZ
err1:		.db		"  *** Unrecognized Input! ***",ctlZ
sakm:		.db		"  Strike any key --> ",ctlZ
;
blankm:		.db		"                                  ",ctlZ		; Erase line on LCD
time_hdr:	.db		"Time: ",ctlZ		; Time line on LCD
date_hdr:	.db		"Date: ",ctlZ		; Date line on LCD
;
TZ_offm:	.db		"TZ offset: ",ctlZ
TZm:		.db		"*** Enter new TZ offset, -12 to +14, 'S' to save ***",ctlZ
savEEms:	.db		"*** Saving to EEPROM ... ",ctlZ
;
.exit
;
;
; --- End of source code ---
;
;

