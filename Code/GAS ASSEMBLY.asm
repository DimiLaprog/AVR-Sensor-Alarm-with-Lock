;
; ASSEMBLY GAS DETECTION.asm
;
;
; Authors : Dimitrios Lampros
;		  : Marios Mitropoulos

.include "m16def.inc"

.DSEG

_tmp_: .byte 2

.CSEG

.org 0x0
rjmp reset

.org 0x1C
rjmp ADC_READY ;interrupt handler for ADC conversion completion

ADC_READY:
	push r24
	in	r24,SREG			;save SREG
	push r24
	cli						;disable Global Interrupt so as not to have
							;an interrupt within an interrupt
	ldi r24, (1<<TOV1)		;CLEAR TIMER1 OVERFLOW BIT
	out TIFR,r24		

	in r20,ADCL			;move ADCL to r20
	in r21,ADCH			;move ADCH to r21
	andi r21,0x03		;keep 2 lsbs of adch
	ldi r24,0xFC ;init TCNT1
	out TCNT1H ,r24 ;overflow after 100ms
	ldi r24 ,0xF2
	out TCNT1L ,r24
	pop r24
	out SREG,r24
	pop r24
	sei
	reti

;----------------------------------------------APPLICATION CODE---------------------------------

reset:
	ldi	r24,low(RAMEND)		;initialize stack pointer
	out	SPL,r24
	ldi	r24,high(RAMEND)
	out	SPH,r24
	ser r24
	out	DDRD, r24	
	out	DDRB, r24			;PORT B and D are OUTPUT ports
	ldi	r24, 0xf0
	out DDRC, r24			;prepare Port C for keyboard
	clr r24
	call lcd_init_sim
	clr r24
	call timer_init
	clr r24
	call ADC_init
	clr r17					;led flag
	clr r18					;lcd flag
	clr r19					;output storage
	clr r20					;ADC input
	clr r21					;ADC input
	clr r22					;alarm flag
	clr r24					;general purpose
	sei
	
start:
	ldi r23, 0x04			;flash counter
first_digit:
	rcall scan_keypad_rising_edge_sim
	rcall keypad_to_ascii_sim
	call function1			;levels while reading 2nd digit
	cpi	r24, 0x00
	breq first_digit
	mov	r28, r24			;save first digit in r28
second_digit:
	rcall scan_keypad_rising_edge_sim
	rcall keypad_to_ascii_sim
	call function1			;levels while reading 2nd digit
	cpi	r24, 0x00
	breq second_digit
	mov	r29, r24			;save second digit in r29
	rcall scan_keypad_rising_edge_sim
check:						;keyword B12
	cpi	r28, '1'
	brne incorrect_code
	cpi	r29, '2'
	brne incorrect_code
correct_code:
	clr r18
	ldi r24, 0x01
    rcall lcd_command_sim	;clear LCD
	ldi	r24, 'W'
	rcall lcd_data_sim
	ldi	r24, 'E'
	rcall lcd_data_sim
	ldi	r24, 'L'
	rcall lcd_data_sim
	ldi	r24, 'C'
	rcall lcd_data_sim
	ldi	r24, 'O'
	rcall lcd_data_sim
	ldi	r24, 'M'
	rcall lcd_data_sim
	ldi	r24, 'E'
	rcall lcd_data_sim
	cpi r22, 0				;check if alarm is on with correct password
	brne alarm_on
alarm_off:
	ldi r24, 0x80
	add r24, r19			;get previous led state + PB7 ON
	out PORTB, r24
	ldi	r24, low(4000)
	ldi r25, high(4000)
	rcall wait_msec
	jmp first_digit
alarm_on:
	ldi r24, 0x80
	out PORTB, r24			;only PB7 ON
	ldi	r24, low(4000)
	ldi r25, high(4000)
	rcall wait_msec
	clr r24
	out PORTB, r24
	jmp first_digit
incorrect_code:
	ldi r24, 0x80
	add r24, r19			;get previous led state + PB7 ON
	out PORTB, r24
	ldi	r24, low(500)
	ldi r25, high(500)		;wait
	rcall wait_msec
	cpi r22, 0x01
	breq leds_off			;check ALARM. If alarm is off, just print all the normal leds, except PB7
	out PORTB, r19
	jmp delay
leds_off:
	clr r24
	out PORTB, r24
delay:
	ldi	r24, low(500)
	ldi r25, high(500)
	rcall wait_msec
	ldi r17, 0x01			;led flag to check if we need to flash or not
	call function1
	dec	r23
	cpi	r23, 0x00
	brne incorrect_code
	clr r17
	jmp start

function1:
	push r24
	mov r26,r20
	mov r27,r21
	
	cpi r27,0x00			;checks to determine level of CO
	brne more_than_255_ADC	;based on ADC value
	cpi r26,0x14
	brlo less_than_20_ADC
	cpi r26,0x49
	brlo led0
	cpi r26,0x7E	
	brlo led1
	cpi r26,0xB2
	brlo led2
	cpi r26,0xCD
	brlo led3
	jmp led4
less_than_20_ADC:
	clr r22					;alarm flag
	clr r17					;led flag
	clr r18					;lcd flag
	ldi r24, 0x01			;Load Clear Instruction in R24
    rcall lcd_command_sim
	clr r19
	clr r25
	out PORTB,r25
	ldi	r22, 0x00
	pop r24
	ret
more_than_255_ADC:			;(ADCH>0)
	cpi r26,0x03
	brlo led4
	cpi r26,0x38
	brlo led5
	jmp led6
led0:
	ldi r19, 0x01
	ldi r25,0x01			;00000001 output
	out PORTB,r25
	cpi r18, 0x01			;if CLEAR is already on display, skip
	breq skip0
	call display_clear
skip0:
	ldi	r22, 0x00
	pop r24
	ret
led1:
	ldi r19, 0x02
	ldi r25,0x02			;00000010 output
	out PORTB,r25
	cpi r18, 0x01			;if CLEAR is already on display, skip
	breq skip1
	call display_clear
skip1:
	ldi	r22, 0x00
	pop r24
	ret
led2:
	ldi r19, 0x04
	ldi r25,0x04			;00000100 output
	out PORTB,r25
	cpi r18, 0x01			;if CLEAR is already on display, skip
	breq skip2
	call display_clear
skip2:
	ldi	r22, 0x00
	pop r24
	ret
led3:
	ldi r19, 0x08
	ldi r25,0x08			;00001000 output
	out PORTB,r25
	cpi r18, 0x01			;if CLEAR is already on display, skip
	breq skip3
	call display_clear
skip3:
	ldi	r22, 0x00
	pop r24
	ret
led4:
	ldi r19, 0x10
	ldi r25,0x10			;00010000 output
	cpi r18, 0x02			;if GAS DETECTED is already on display, skip
	breq skip4
	call display_gas_detected
skip4:
	cpi r17, 0x01
	breq skip4_flash
	call flash
skip4_flash:
	ldi	r22, 0x01
	pop r24
	ret
led5:
	ldi r19, 0x20
	ldi r25,0x20			;00100000 output
	cpi r18, 0x02			;if GAS DETECTED is already on display, skip
	breq skip5
	call display_gas_detected
skip5:
	cpi r17, 0x01
	breq skip5_flash
	call flash
skip5_flash:
	ldi	r22, 0x01
	pop r24
	ret
led6:
	ldi r19, 0x40
	ldi r25,0x40			;01000000 output
	cpi r18, 0x02			;if GAS DETECTED is already on display, skip
	breq skip6
	call display_gas_detected
skip6:
	cpi r17, 0x01
	breq skip6_flash
	call flash
skip6_flash:
	ldi	r22, 0x01
	pop r24
	ret

display_clear:
	ldi r18, 0x01			;lcd flag
	ldi r24, 0x01			;Load Clear Instruction in R24
    rcall lcd_command_sim
	ldi	r24, 'C'
	rcall lcd_data_sim
	ldi	r24, 'L'
	rcall lcd_data_sim
	ldi	r24, 'E'
	rcall lcd_data_sim
	ldi	r24, 'A'
	rcall lcd_data_sim
	ldi	r24, 'R'
	rcall lcd_data_sim
	ret

display_gas_detected:
	ldi r18, 0x02			;lcd flag
	ldi r24, 0x01			;Load Clear Instruction in R24
    rcall lcd_command_sim
	ldi	r24, 'G'
	rcall lcd_data_sim
	ldi	r24, 'A'
	rcall lcd_data_sim
	ldi	r24, 'S'
	rcall lcd_data_sim
	ldi	r24, ' '
	rcall lcd_data_sim
	ldi	r24, 'D'
	rcall lcd_data_sim
	ldi	r24, 'E'
	rcall lcd_data_sim
	ldi	r24, 'T'
	rcall lcd_data_sim
	ldi	r24, 'E'
	rcall lcd_data_sim
	ldi	r24, 'C'
	rcall lcd_data_sim
	ldi	r24, 'T'
	rcall lcd_data_sim
	ldi	r24, 'E'
	rcall lcd_data_sim
	ldi	r24, 'D'
	rcall lcd_data_sim
	ret

flash:								;leds on-off function for r25 value 
	push r24
	out PORTB, r25
	ldi	r24, low(500)
	ldi r25, high(500)
	rcall wait_msec
	clr r25
	out PORTB, r25
	ldi	r24, low(500)
	ldi r25, high(500)
	rcall wait_msec
	pop r24
	ret

; Routine: usart_init 
; Description: 
; This routine initializes the
; ADC as shown below. 
; ------- INITIALIZATIONS ------- ; 
; Vref: Vcc (5V for easyAVR6) 
; Selected pin is A0 
; ADC Interrupts are Enabled 
; Prescaler is set as CK/128 = 62.5kHz 
; -------------------------------- 
; parameters: None. 
; return value: None. 
; registers affected: r24 
; routines called: None 
ADC_init:
	ldi r24,(1<<REFS0) ;choose external reference voltage Vref: Vcc 
	out ADMUX,r24 ;MUX4:0 = 00000 for A0. 
	;ADC is Enabled (ADEN=1) 
	;ADC Interrupts are Enabled (ADIE=1) 
	;Set Prescaler CK/128 = 62.5Khz (ADPS2:0=111)
	clr r24
	ldi r24,(1<<ADTS2)|(1<<ADTS1) ;ADTS=110 ,TIMER1 OVERFLOW 
	out SFIOR,r24  ;to be used in ADATE --> auto-trigger enable: source timer 1 overflow
	clr r24
	ldi r24,(1<<ADEN)|(1<<ADIE)|(1<<ADATE)|(1<<ADSC)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0)
	;using ADSC to init the ADC. 
	out ADCSRA,r24 
	ret

timer_init:
	;;no interrupt routine needed. ADC ISR will be called on overflow of timer1
	ldi r24 ,(1<<CS12) | (0<<CS11) | (1<<CS10) ; CK/1024
	out TCCR1B ,r24 
	;Counter Frequency=7812.5 Hz
	;100(ms)*7812.5(cycles)=781.25 cycles needed 
	;Therefore, Init value=65536-781.25=64754.75 ,=~ 64754 
	;64754=0xFCF2
	ldi r24,0xFC ;init TCNT1
	out TCNT1H ,r24 ;overflow after 100ms
	ldi r24 ,0xF2
	out TCNT1L ,r24
	ret

;-------------------------------------------Hidden_Functions--------------------------------------------
	
wait_usec:
sbiw r24 ,1 ; 2  (0.250 ?sec)
nop ; 1  (0.125 ?sec)
nop ; 1  (0.125 ?sec)
nop ; 1  (0.125 ?sec)
nop ; 1  (0.125 ?sec)
brne wait_usec ; 1 ? 2  (0.125 ? 0.250 ?sec)
 ret ; 4  (0.500 ?sec)

wait_msec:
 push r24 ; 2 (0.250 ?sec)
 push r25 ; 2 
ldi r24 , low(998) ;  r25:r24  998 (1  - 0.125 ?sec)
 ldi r25 , high(998) ; 1  (0.125 ?sec)
 rcall wait_usec ; 3  (0.375 ?sec),  998.375 ?sec
 pop r25 ;  (0.250 ?sec)
 pop r24 ; 
 sbiw r24 , 1 
 brne wait_msec 
 ret 

scan_row_sim:
out PORTC, r25 
push r24 
push r25 
ldi r24,low(500) 
ldi r25,high(500)
rcall wait_usec
pop r25
pop r24 
nop
nop 
in r24, PINC 
andi r24 ,0x0f 
ret 

scan_keypad_sim:
push r26 
push r27 
ldi r25 , 0x10 ; (PC4: 1 2 3 A)
rcall scan_row_sim
swap r24 
mov r27, r24 
ldi r25 ,0x20 ;  (PC5: 4 5 6 B)
rcall scan_row_sim
add r27, r24 
ldi r25 , 0x40
rcall scan_row_sim
swap r24 
mov r26, r24 
ldi r25 ,0x80 ;  (PC7: * 0 # D)
rcall scan_row_sim
add r26, r24 
movw r24, r26 
clr r26 
out PORTC,r26 
pop r27 
pop r26
ret

scan_keypad_rising_edge_sim:
push r22
push r23 
push r26
push r27
rcall scan_keypad_sim 
push r24 
push r25
ldi r24 ,15 
ldi r25 ,0 
rcall wait_msec
rcall scan_keypad_sim 
pop r23 
pop r22
and r24 ,r22
and r25 ,r23
ldi r26 ,low(_tmp_) 
ldi r27 ,high(_tmp_) 
ld r23 ,X+
ld r22 ,X
st X ,r24 
st -X ,r25 
com r23
com r22 
and r24 ,r22
and r25 ,r23
pop r27 
pop r26 
pop r23
pop r22
ret

keypad_to_ascii_sim:
push r26 
push r27 
movw r26 ,r24 

ldi r24 ,'*'
sbrc r26 ,0
rjmp return_ascii
ldi r24 ,'0'
sbrc r26 ,1
rjmp return_ascii
ldi r24 ,'#'
sbrc r26 ,2
rjmp return_ascii
ldi r24 ,'D'
sbrc r26 ,3 
rjmp return_ascii 
ldi r24 ,'7'
sbrc r26 ,4
rjmp return_ascii
ldi r24 ,'8'
sbrc r26 ,5
rjmp return_ascii
ldi r24 ,'9'
sbrc r26 ,6
rjmp return_ascii ;
ldi r24 ,'C'
sbrc r26 ,7
rjmp return_ascii
ldi r24 ,'4' 
sbrc r27 ,0 
rjmp return_ascii
ldi r24 ,'5'
sbrc r27 ,1
rjmp return_ascii
ldi r24 ,'6'
sbrc r27 ,2
rjmp return_ascii
ldi r24 ,'B'
sbrc r27 ,3
rjmp return_ascii
ldi r24 ,'1'
sbrc r27 ,4
rjmp return_ascii ;
ldi r24 ,'2'
sbrc r27 ,5
rjmp return_ascii
ldi r24 ,'3' 
sbrc r27 ,6
rjmp return_ascii
ldi r24 ,'A'
sbrc r27 ,7
rjmp return_ascii
clr r24
rjmp return_ascii
return_ascii:
pop r27 
pop r26
ret

write_2_nibbles_sim:
push r24 
push r25 
ldi r24 ,low(6000) ; Can't read Busy Flag, so let's wait 6 ms
ldi r25 ,high(6000)
rcall wait_usec
pop r25
pop r24 
push r24 ; TRANSFERRING 4 bits through R24 MSB
in r25, PIND 
andi r25, 0x0f ; R25 HOLDS CONTROL BITS - ENABLE (PD3)--RS- (PD2)
andi r24, 0xf0 
add r24, r25 
out PORTD, r24 
sbi PORTD, PD3 
cbi PORTD, PD3 ;enable LCD. PD3 HIGH-->LOW
push r24 ; REPEAT THE PROCESS TO LOAD THE REST 4 BITS
push r25 
ldi r24 ,low(6000) ; 
ldi r25 ,high(6000)
rcall wait_usec
pop r25
pop r24 
pop r24 
swap r24 
andi r24 ,0xf0 
add r24, r25
out PORTD, r24
sbi PORTD, PD3 ; Enable
cbi PORTD, PD3
ret

lcd_data_sim:
push r24 
push r25 
sbi PORTD, PD2 ; ENABLE WRITE (PD2=1)
rcall write_2_nibbles_sim ; SEND byte
ldi r24 ,43 
ldi r25 ,0 
rcall wait_usec
pop r25 
pop r24
ret

lcd_command_sim:
push r24 
push r25 
cbi PORTD, PD2 ; SET INSTRUCTION WRITE (RS=0) (PD2=0)
rcall write_2_nibbles_sim ; SEND BYTES and WAIT
ldi r24, 39 
ldi r25, 0 ;  clear display  return home
rcall wait_usec 
pop r25 
pop r24
ret

lcd_init_sim:  ;;-----------------------SEND 0X30 2 TIMES TO GO IN 8BIT MODE--->THEN CHANGE TO 4 BIT MDOE, THEN INIT THE REST
push r24 
push r25 
ldi r24, 40 
ldi r25, 0 
rcall wait_msec 
ldi r24, 0x30 
out PORTD, r24 
sbi PORTD, PD3 
cbi PORTD, PD3 
ldi r24, 39
ldi r25, 0 
rcall wait_usec 
 
push r24 
push r25 
ldi r24,low(1000) 
ldi r25,high(1000)
rcall wait_usec
pop r25
pop r24 
ldi r24, 0x30
out PORTD, r24
sbi PORTD, PD3
cbi PORTD, PD3
ldi r24,39
ldi r25,0
rcall wait_usec 
push r24 
push r25 
ldi r24 ,low(1000) 
ldi r25 ,high(1000)
rcall wait_usec
pop r25
pop r24 
ldi r24,0x20 
out PORTD, r24
sbi PORTD, PD3
cbi PORTD, PD3
ldi r24,39
ldi r25,0
rcall wait_usec
push r24 
push r25 
ldi r24 ,low(1000) 
ldi r25 ,high(1000)
rcall wait_usec
pop r25
pop r24
ldi r24,0x28 
rcall lcd_command_sim 
ldi r24,0x0c 
rcall lcd_command_sim
ldi r24,0x01 
rcall lcd_command_sim
ldi r24, low(1530)
ldi r25, high(1530)
rcall wait_usec
ldi r24 ,0x06 
rcall lcd_command_sim 

pop r25 
pop r24
ret