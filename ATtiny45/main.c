#define F_CPU 1000000ul
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>

#define ON_TIME 	1800 //30 minutes
#define START_CNT	12	//Interrupt frequency = (1MHz / 4096) / (256 - START_CNT)

void sleep(void)
{
   /* Do the actual sleep */
   set_sleep_mode(SLEEP_MODE_IDLE);
   sleep_enable();
   sleep_cpu();
   sleep_disable();
}

uint8_t button()
{
	uint8_t pin = PINB & (1 << 3);
	return !(pin);
}

void setRelay()
{
	PORTB |= (1 << 4);
}

void resetRelay()
{
	PORTB &= ~(1 << 4);
}

uint8_t getRelay()
{
	uint8_t pin =  PINB & (1 << 4);
	return pin;
}

volatile int32_t timeout = 10;

void startTimer()
{
	timeout = ON_TIME;

	TCNT1 = START_CNT;	//(1MHz / 4096) / (256 - START_CNT) = 1.000576Hz
	TCCR1 = 0x0D;		//enable Counter1 PCK/4096
	TIMSK = (1 << 2); 	//enable Counter1 interrupt
}

void stopTimer()
{
	TCCR1 = 0;
	TIMSK = 0;
}

volatile uint8_t inputFlags = 0;

int main(void)
{
	/* Setup the CPU */

	/* Setup the clock prescaler */
	CLKPR =_BV(CLKPCE);
	CLKPR = 0x03;

	PRR = 0x07;
	ACSR = 0x80;		//disable Analog Comparator

	DDRB |= (1 << 4);  	//PB4 is the relay output
	PORTB |= (1 << 3); 	//enable pull-up for PB3

	PCMSK = (1 << 3); 	//enable pin change interrupt for PB3
	GIMSK = (1 << 5); 	//enable pin change interrupt


	for (;;)
	{
		sei();
		sleep();

		if(inputFlags & 0x01)
		{
			if(getRelay())
				goto stop;

			startTimer();
			setRelay();
		}

		if(timeout == 0)
		{
stop:
			stopTimer();
			resetRelay();
		}

		inputFlags = 0;
	}
}


ISR(PCINT0_vect)
{
	cli();
	_delay_ms(30);
	if(button())
	{
		inputFlags = 1;
	}
	sei();
}

ISR(TIM1_OVF_vect)
{
	timeout--;
	TCNT1 = 11;
}
