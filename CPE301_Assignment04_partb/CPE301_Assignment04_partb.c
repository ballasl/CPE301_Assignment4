/*
* CPE301_Assignment04_partb.c
*AVR C program to control the speed of the Stepper Motor using a
*potentiometer connected to any of the analog-in port. Using a timer in CTC mode to
*control the delay.
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
#define FORWARD 1
#define REVERSE -1
/*
	For motor PK243-02AA, Stepper Motor:
	Step Angle = 1.8º
	Number of step per 360º revolution:
	TOTALSTEPS = 360 / 1.8 = 200
*/
#define TOTALSTEPS 200
volatile uint16_t adc_value=0;
volatile uint16_t old_adc_value=0;
volatile uint8_t interrupted =0;
volatile uint16_t steps_togo = TOTALSTEPS - 1;
volatile uint16_t step = 0;
volatile uint8_t direction = FORWARD;
volatile uint8_t running = 0;
#define STARTSTOPBTN 0
#define REVERSEBTN 1

void adc_init(void)
{
	ADMUX = 0; // use ADC0
	ADMUX |= (1 << REFS0); // use AVcc as the reference. Input will be right justified
	ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // 128 pre-scale for 16Mhz
	ADCSRA |= (1 << ADEN); // Enable the ADC

	ADCSRB = 0; // 0 for free running mode
}

void timer1_init()
{
	TCCR1A = 0;

	// set up timer with CTC mode and prescaling = 64
	TCCR1B |= (1 << WGM12)|(1 << CS11)|(1 << CS10);
	
	// initialize counter
	TCNT1 = 0;
	/*
	 Initialize compare value
	 With prescalar = 64, the frequency 16,000,000 Hz / 64 -> period = 0.000004 s
	 TimerCount=Requireddelay/period -1
	 For example for 0.0020 s delay -> TimerCount = 0.002/0.000004 -1 = 499
	 Let's consider this as our temporary start value.
	*/
	OCR1A = 499; // value for 20 millisecond delay
	// enable compare interrupt
	TIMSK1 |= (1 << OCIE1A);
}

void debounce_polling(uint8_t btn)
{
	if (btn==STARTSTOPBTN)
	{
		while (bit_is_clear(PINC, 2))
			_delay_ms(20);
	}
	else if(btn==REVERSEBTN)
	{
		while (bit_is_clear(PINC, 1))
		_delay_ms(20);
	}
}

int main(void)
{
	DDRC = 0x0;     // Clear all pins;
	PORTC |= (1 << PORTC1) ; // Enables the pull-up of the reverse direction button
	PORTC |= (1 << PORTC2) ; // Enables the pull-up of start stop button
	DDRB |= (1<<DDB0) |  (1<<DDB1) |  (1<<DDB2) |  (1<<DDB3); // Set  PB0 to PB3 for output

#ifdef ISDEBUGGING
	char printBuffer[128];
	usart_init();
#endif
	timer1_init();
	adc_init();
	sei();
	

	while (1)
	{
		ADCSRA |= (1 << ADSC); // Start the ADC conversion
		while (ADCSRA	&	(1<<ADSC));
		
		if (bit_is_clear(PINC,2))
		{
			debounce_polling(STARTSTOPBTN);
			running=!running;
		}
		if (bit_is_clear(PINC,1))
		{
			debounce_polling(REVERSEBTN);
			direction = direction == FORWARD ? REVERSE : FORWARD;
			steps_togo = TOTALSTEPS - steps_togo - 1;
			step=0;
		}
		
		adc_value=ADCL;
		adc_value = (ADCH<<8) + adc_value;
			
		if (abs(adc_value-old_adc_value)>1)
		{
			old_adc_value=adc_value;
			/*
				We establish a range between 2 milliseconds and 20 milliseconds delay.
				This corresponds to OCR1A values between 499 and 4999 (see above our reasoning)
			*/
			uint16_t ocr_value = 499+(double)adc_value*((double)(4999-499)/1024);
			TCNT1 = 0;
			OCR1A = ocr_value;
			
			#ifdef ISDEBUGGING
				sprintf(printBuffer, "adc_value %d ocr_value is %d\n",(int)adc_value, (int)ocr_value );
				USARTSendStr(printBuffer);
			#endif
		}
	}
}

ISR (TIMER1_COMPA_vect)
{
#ifdef ISDEBUGGING
	//USARTSendStr("interrupt\n"); Not to be used normally because will disturb the established delay.
#endif
		if (running && (steps_togo>0))
		{
			PORTB &= 0xF0;
			switch(step)
			{
				case 0:
					PORTB |= (1 << PORTB0) | (1 << PORTB2);
					break;
				case 1:
					PORTB |= (1 << PORTB1) | (1 << PORTB2);
					break;
				case 2:
					PORTB |= (1 << PORTB1) | (1 << PORTB3);
					break;
				case 3:
					PORTB |= (1 << PORTB3) | (1 << PORTB0) ;
					break;
				default:
					break;
			}
			if (direction==FORWARD) step++;
			else step--;
			step = step % 4;
			steps_togo--;
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

