/*
 * CPE301_Assignment04_partc.c
 *C program to control the position of the Servo Motor using a
 *potentiometer connected to PC0 the analog-in port. When pot value is 0 the servo is
 *at position 0 deg. and when pot value is max (approx. 5V) the servo is at position 180 deg
 */ 
#define ISDEBUGGING
#ifdef ISDEBUGGING
 #include <string.h>
 #include <avr/interrupt.h>
 #include <stdio.h>
#endif

#define F_CPU 16000000 //16MHz
#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#define PWM_PRESCALLER 64
#define TOP (double)F_CPU/(PWM_PRESCALLER*50)-1; // 4999. (or Period*F_CPU/PWM_PRESCALLER - 1, where period is 20ms at 50 Hz)
//#define PULSE_MIN 800
//#define PULSE_MAX 2200
#define PULSE_MIN 544     // According to data sheet minimum pulse width is 800
#define PULSE_MAX 2200    // According to data sheet maximum pulse with is 2200

volatile unsigned long adc_value=0; 
volatile unsigned long old_ocr_value=0;

#ifdef ISDEBUGGING
#define FOSC 16000000 // Clock Speed
#define BAUD 9600
#define MYUBRR FOSC/16/BAUD - 1
volatile char ReceivedChar;
int USART0SendByte(char u8Data);
void usart_init(void);
char printBuffer[128];
volatile uint8_t chartoSend;
float temp;
#endif

void adc_init(void)
{
	ADMUX = 0; // use ADC0
	ADMUX |= (1 << REFS0); // use AVcc as the reference. Input will be right justified
	ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // 128 pre-scale for 16Mhz
	ADCSRA |= (1 << ADEN); // Enable the ADC
	
	ADCSRB = 0; // 0 for free running mode
}

int main(void)
{
	adc_init();
	#ifdef ISDEBUGGING
	usart_init();
	#endif

    DDRB |= (1<<DDB1); // Set  pin PB1 on Atmega328P Xplainedmini as output
	// DDRB |= (1<<DDB0); // Set  pin PB1 on Atmega328P Xplainedmini as output
	// Timer1 setup:
	//COM1A1/COM1B1 = None-inverted mode (HIGH at bottom, LOW on match), WGM11=Fast PWM
	TCCR1A|=(1<<COM1A1)|(1<<COM1B1)|(1<<WGM11);
	//TCCR1A|=(1<<COM1A1)|(1<<COM1B0)|(1<<WGM11);
	// WGM12 & WGM13=Fast PWM, Prescaler=64
	TCCR1B|=(1<<WGM13)|(1<<WGM12)|(1<<CS11)|(1<<CS10);
	// Set ICR1 register = PWM period
	ICR1 = TOP; //Period = 20 ms 

    while (1) 
    {
		ADCSRA |= (1 << ADSC); // Start the ADC conversion
		//while ((ADCSRA & (1<<ADIF))==0);
		while(	ADCSRA	&	(1<<ADSC)	); 
		adc_value=ADCL;
		adc_value = (ADCH<<8) + adc_value;
	
		unsigned long pulse = PULSE_MIN+(double)adc_value*(PULSE_MAX-PULSE_MIN)/1024;
		unsigned long ocr_value=(double)pulse * 16/64 -1;

		#ifdef ISDEBUGGING
			sprintf(printBuffer, "adc_value=%d ocr_value is %d\n", (int)adc_value, (int)ocr_value);
			int thesize=strlen(printBuffer);
			for (uint8_t i=0; i<thesize;i++)
			{
				USART0SendByte(printBuffer[i]);
			}
		#endif
		if (abs(ocr_value-old_ocr_value)>1)
		{
			old_ocr_value=ocr_value;
			OCR1A = ocr_value;
		}
		_delay_ms(1000);
    }
}
#ifdef ISDEBUGGING
void usart_init()
{
	UBRR0H = (MYUBRR) >> 8;
	UBRR0L = MYUBRR;
	UCSR0B |= (1 << RXEN0) | (1 << TXEN0); // Enable receiver and transmitter
	UCSR0B |= (1 << RXCIE0); // Enable receiver interrupt
	UCSR0C |=  (1 << UCSZ01) | (1 << UCSZ00); // Set frame: 8data, 1 stop
	sei();
}

ISR (USART_RX_vect)
{
	ReceivedChar = UDR0; // Read data from the RX buffer
	
	UDR0 = ReceivedChar; // Write the data to the TX buffer
}

int USART0SendByte(char u8Data)
{
	//wait while previous byte is completed
	while(!(UCSR0A&(1<<UDRE0))){};
	// Transmit data
	//UDR0 = u8Data;
	UDR0=u8Data;
	return 0;
}
#endif

