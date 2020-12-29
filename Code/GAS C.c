/*
 * GAS DETECTION IN C
 *
 * 
 * Authors : Dimitrios Lampros
		   : Marios Mitropoulos
 */ 

#include <avr/io.h>
#define F_CPU 8000000UL
#include <util/delay.h>
#define _NOP() do { asm volatile ("nop"); } while (0)
#include <avr/interrupt.h>
//#define _XTAL_FREQ 8000000
//#define sei() asm volatile("sei"::)
//#define cli() asm volatile("cli"::)

char R24, R25, old_pressed_button0 = 0, old_pressed_button1 = 0, R20, R21, R26, R27, output = 0, led_flag = 0;

void scan_row_sim()
{
	PORTC = R25;
	_delay_ms(0.5);
	_NOP();
	_NOP();
	R24 = PINC & 0x0f; //keep 4 LSBs of PINC
	return;
}

void scan_keypad_sim()
{
	char temp0, temp1;
	
	R25 = 0x10;
	scan_row_sim();
	temp0 = R24<<4;
	
	R25 = 0x20;
	scan_row_sim();
	temp0 = temp0 + R24;
	
	R25 = 0x40;
	scan_row_sim();
	temp1 = R24<<4;
	
	R25 = 0x80;
	scan_row_sim();
	R24 = temp1 + R24;
	R25 = temp0;
	PORTC = 0;
	return;
}

void scan_keypad_rising_edge_sim()
{
	scan_keypad_sim();
	char A = R24, B = R25, temp0, temp1;
	_delay_ms(15);
	scan_keypad_sim();
	R24 = R24 & A;					//debounced current pressed buttons
	R25 = R25 & B;
	temp0 = old_pressed_button0;
	temp1 = old_pressed_button1;
	old_pressed_button0 = R24;		//save current state as old state
	old_pressed_button1 = R25;
	R24 = R24 & (~temp0);			//compare previous with current state;
	R25 = R25 & (~temp1);			//if previous state of button was 0 and now it is 1, then the button
	//was just pressed.
	return;
}

char keypad_to_ascii_sim()
{
	if (R24 & 1) return '*';
	if (R24 & 2) return '0';
	if (R24 & 4) return '#';
	if (R24 & 8) return 'D';
	if (R24 & 16) return '7';
	if (R24 & 32) return '8';
	if (R24 & 64) return '9';
	if (R24 & 128) return 'C';
	if (R25 & 1) return '4';
	if (R25 & 2) return '5';
	if (R25 & 4) return '6';
	if (R25 & 8) return 'B';
	if (R25 & 16) return '1';
	if (R25 & 32) return '2';
	if (R25 & 64) return '3';
	if (R25 & 128) return 'A';
	return 0;
}

void timer_init()
{
	TIMSK = (1<<TOIE1);								// Enable interrupt of TIMER 1
	TCCR1B = (1<<CS12)|(0<<CS11)|(1<<CS10);			// CLK/1024
	TCNT1H = 0xFC;
	TCNT1L = 0xF2;
	//Counter Frequency=7812.5 Hz
	//100(ms)*7812.5(cycles)=781.25 cycles needed 
	//Therefore, Init value=65536-781.25=64754.75 ,=~ 64754 
	//64754=0xFCF2
	return;
}

void ADC_init()
{
	ADMUX = (1<<REFS0); //analog reference
	ADCSRA = (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);  //enable adc,clock/128
	return;
}

ISR(TIMER1_OVF_vect)
{
	cli();
	ADCSRA |= (1<<ADSC);					//begin conversion when timer is interrupted
	while (ADCSRA & (1<<ADSC));				//wait until conversion is done (polling)
	R20 = ADCL;
	R21 = ADCH & 0x03;
	TIFR = (1<<TOV1);						//reset timer
	TCNT1H = 0xFC;
	TCNT1L = 0xF2;
	sei();
}

int main2()
{
	R26 = R20;
	R27 = R21;
	char alarm = 0;
	if (R27 == 0)
	{
		if (R26 < 20) {
			output = 0;
			PORTB = 0;
		}
		else if (R26 < 73) {
			output = 1;
			PORTB = 1;
		}
		else if (R26 < 126) {
			output = 2;
			PORTB = 2;
		}
		else if (R26 < 178) {
			output = 4;
			PORTB = 4;
		}
		else if (R26 < 205) {
			output = 8;
			PORTB = 8;
		}
		else // 204 < ADC < 256
		{
			alarm = 1;
			output = 16;
			if (led_flag != 1) {
				PORTB = 16;
				_delay_ms(500);
				PORTB = 0;
				_delay_ms(500);
			}
		}
	}
	else
	{
		if (R26 < 3)
		{
			alarm = 1;
			output = 16;
			if (led_flag != 1) {
				PORTB = 16;
				_delay_ms(500);
				PORTB = 0;
				_delay_ms(500);
			}
		}
		else if (R26 < 56)
		{
			alarm = 1;
			output = 32;
			if (led_flag != 1) {
				PORTB = 32;
				_delay_ms(500);
				PORTB = 0;
				_delay_ms(500);
			}
		}
		else // ADC > 310
		{
			alarm = 1;
			output = 64;
			if (led_flag != 1) {
				PORTB = 64;
				_delay_ms(500);
				PORTB = 0;
				_delay_ms(500);
			}
		}
	}
	return alarm;
}

int main(void)
{
	char KEYB_ONE, KEYB_TWO, alarm, i;
	DDRB = 0xff;
	DDRC = 0xf0;
	timer_init();
	ADC_init();
	sei();
    while (1)
    {
		do
		{
			alarm = main2();
			scan_keypad_rising_edge_sim();
			KEYB_ONE = keypad_to_ascii_sim();
		} while (KEYB_ONE == 0);
		
		do
		{
			alarm = main2();
			scan_keypad_rising_edge_sim();
			KEYB_TWO = keypad_to_ascii_sim();
		} while (KEYB_TWO == 0);
		
		scan_keypad_rising_edge_sim();
		
		if (KEYB_ONE == '1' && KEYB_TWO == '2')
		{
			if (alarm == 0)
			{
				PORTB = output + 128;
				_delay_ms(4000);
			}
			else
			{
				PORTB = 128;
				_delay_ms(4000);
				PORTB = 0;
			}
		}
		else 				//WRONG PASSWORD
		{
			for (i = 0; i < 4; i++)
			{
				PORTB = output + 128;
				_delay_ms(500);
				if (alarm == 1)		//if alarm, then turn all leds off so we can synchronise flashing
					PORTB = 0;
				else
					PORTB = output; //if not alarm, print regular output and then flash
				_delay_ms(500);
				led_flag = 1;		//indicating current flashing of danger leds +PB7
				main2();			//check values but no flashing
			}
			led_flag = 0;			//we have finished flashing, return to normal operation
		}
    }
	return 0;
}
