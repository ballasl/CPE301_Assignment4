/*
 * CPE301_Assignment04_parta.c
  * Created: 3/31/2018 10:01:49 AM
 *Write an AVR C program to control the speed of the DC Motor using a potentiometer
 *connected to any of the analog-in port. Use an interrupt on a button to stop and start
 *the motor at each click. The minimum speed of the motor should be 0 when pot is
 *minimum and maximum should be 95% of PWM value.
 */ 
#define ISDEBUGGING
 #ifdef ISDEBUGGING
 #include <string.h>
  #include <stdio.h>
 
 #define FOSC 16000000 // Clock Speed
 #define BAUD 9600
 #define MYUBRR FOSC/16/BAUD - 1
 volatile char ReceivedChar;
 int USART0SendByte(char u8Data);
 void USARTSendStr(char* _str);
 void usart_init(void);

 volatile uint8_t chartoSend;
 float temp;
 #endif
 
#define F_CPU 16000000 //16MHz
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <util/delay.h>
#define PWM_PRESCALLER 64
#define TOP (double)F_CPU/(PWM_PRESCALLER*50)-1 // 4999. (or Period*F_CPU/PWM_PRESCALLER - 1, where period is 20ms at 50 Hz)

volatile uint16_t debouncing =0;
volatile uint16_t switched_on = 0;
volatile uint16_t adc_value=0; 
volatile uint16_t old_adc_value=0;
volatile uint16_t interrupt_counter=0; // used for debouncing puposes;

void enable_external_interrupt(void)
{
	PCICR = 1<<PCIE1;           // Enables PCMSK0 scan
	PCMSK1 = 1<<PCINT9;			// Triggers interrupt whenever pin PC1 changes state
}

void adc_init(void)
{
	ADMUX = 0; // use ADC0
	ADMUX |= (1 << REFS0); // use AVcc as the reference. Input will be right justified
	ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // 128 pre-scale for 16Mhz
	ADCSRA |= (1 << ADEN); // Enable the ADC
	
	ADCSRB = 0; // 0 for free running mode
}

void bounce_polling(void)
{
	unsigned long temp_interrupt_counter;
	do 
	{
		temp_interrupt_counter = interrupt_counter;
		_delay_ms(500);
	} while (temp_interrupt_counter !=interrupt_counter); // If still bouncing the interrupt_counter will change during the delay
};

int main(void)
{
#ifdef ISDEBUGGING
 char printBuffer[128];
#endif
	
	DDRC = 0x0;     // Clear all pins;
	PORTC |= (1 << PORTC1) ; // Enables the pull-up

	DDRB |= (1<<DDB1); // Set  pin PB1 for output

    #ifdef ISDEBUGGING
		usart_init();
    #endif
	enable_external_interrupt();
	adc_init();
	// Timer1 setup:
	//COM1A1/COM1B1 = None-inverted mode (HIGH at bottom, LOW on match), WGM11=Fast PWM
	TCCR1A|=(1<<COM1A1)|(1<<COM1B1)|(1<<WGM11);
	// WGM12 & WGM13=Fast PWM, Prescaler=64
	TCCR1B|=(1<<WGM13)|(1<<WGM12)|(1<<CS11)|(1<<CS10);
	// Set ICR1 register = PWM period
	ICR1 = TOP; 

	sei();
	
    while (1) 
    {
			ADCSRA |= (1 << ADSC); // Start the ADC conversion
			while(	ADCSRA	&	(1<<ADSC)	);
			if(switched_on)
			{
				adc_value=ADCL;
				adc_value = (ADCH<<8) + adc_value;
				if (abs(adc_value-old_adc_value)>1)
				{
					old_adc_value=adc_value;
					uint16_t ocr_value = 0.95*(double)adc_value*((double)TOP)/1024;
					
					OCR1A = ocr_value;
				
				#ifdef ISDEBUGGING
							
							sprintf(printBuffer, "adc_value %d ocr_value is %d\n",(int)adc_value, (int)ocr_value );
							USARTSendStr(printBuffer);

				#endif
				}
			}
			else
			{
				if(old_adc_value)
				{
					OCR1A=0;
					old_adc_value=0;
				}
				
			}
			if (debouncing) // wait a little if debouncing
			{
				bounce_polling();
				debouncing=0;
			}
    }
}

ISR (PCINT1_vect)
{

	if (bit_is_clear(PINC, 1))
	{
#ifdef ISDEBUGGING
			USARTSendStr("interrupt\n");
#endif
			interrupt_counter++;

			if (!debouncing)
			{
				debouncing=1;
				switched_on = !switched_on; 
			}
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
	UDR0=u8Data;
	return 0;
}

void USARTSendStr(char* _str)
{
			int thesize=strlen(_str);
			for (uint8_t i=0; i<thesize;i++)
			{
				USART0SendByte(_str[i]);
			}
}
#endif
