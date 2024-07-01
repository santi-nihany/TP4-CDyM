#define F_CPU 16000000UL
#define BR9600 (0x67)
#include <avr/io.h>
#include <util/delay.h>
#include "serialPort.h"

volatile char RX_Buffer=0;

int main(void)
{
	uint8_t l_red = 0;
	DDRB = (1 << PORTB1)|(1 << PORTB2)|(1 << PORTB5); // salida puertos B 1,2,5
	/* configuracion pwm puertos 1 y 2*/
	OCR1A = 0;
	OCR1B = 0;
	TCCR1A = (1<< COM1A1) | (1<< COM1B1) | (1<< COM1A0) | (1<< COM1B0) | (1<< WGM10);	// PHASE CORRECT 8BIT - INVERTIDO
	TCCR1B= (1<< CS10);							        // PRESCALER 1
	PORTB|=((1 << PORTB2)|(1 << PORTB5));
	
	/* UART */
	SerialPort_Init(BR9600);
	SerialPort_TX_Enable();
	SerialPort_RX_Enable();
	SerialPort_Send_String("Ingrese R, G o B\n\r");
	SerialPort_RX_Interrupt_Enable();
	sei();
    while (1) 
    {
		if(l_red > TCNT1){
			PORTB &=~ (1<< PORTB5);
		}else{
			PORTB |= (1<< PORTB5);
		}
		// leer
		if(RX_Buffer){
			switch (RX_Buffer) {
				case 'R':
					l_red^=0xFF;
					break;
				case 'G':
					OCR1B^=0xFF;
					break;
				case 'B':
					OCR1A^=0xFF;
					break;
					
			}
			RX_Buffer=0;
		}
    }
}
ISR(USART_RX_vect){
	RX_Buffer = UDR0;
}
