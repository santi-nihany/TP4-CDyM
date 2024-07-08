#define F_CPU 16000000UL
#define BR9600 (0x67)
#include <avr/io.h>
#include <util/delay.h>
#include "serialPort.h"

volatile char RX_Buffer=0;
//volatile act = 0;
volatile uint8_t adc_value;


void ADC_Init() {
	DIDR0 = 0x01; // digital input disable
	ADCSRA |= (1 << ADPS0) | (1 << ADPS1) | (1 << ADPS2) | (1 << ADEN) ; // adc enable and ck/128 -> f = 125KHz
	ADMUX |= (1 << REFS0) | (1 << ADLAR) | (1 << MUX1) | (1 << MUX0); // Vref = AVCC, left justified, AC3 pin
}

uint8_t ADC_Read() {
	ADCSRA |= (1 << ADSC);  // Iniciar conversión
	while (!(ADCSRA & (1 << ADSC)));  // Esperar a que termine la conversión
	ADCSRA |= (1<< ADIF); // clear the ADIF flag
	return ADCH;  // Retornar el valor leído
}


int main(void)
{
	uint8_t l_red = 0;
	uint8_t ant = 0;
	DDRB |= (1 << PORTB1)|(1 << PORTB2)|(1 << PORTB5); // salida puertos B 1,2,5
	DDRC &= ~(1<< PINC3);
	/* configuracion pwm puertos 1 y 2*/
	OCR1A = 0;
	OCR1B = 0;
	TCCR1A |= (1<< COM1A1) | (1<< COM1B1) | (1<< COM1A0) | (1<< COM1B0) | (1<< WGM10);	// PHASE CORRECT 8BIT - INVERTIDO
	TCCR1B |= (1<< CS10);							        // PRESCALER 1
	PORTB|=((1 << PORTB2)|(1 << PORTB5));

	ADC_Init();
	
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
		adc_value = ADC_Read();
		switch (RX_Buffer) {
			case 'R':
				l_red = adc_value;
				if(ant != RX_Buffer){
					SerialPort_Send_String("CAMBIANDO INTENSIDAD DEL ROJO\n\r");
					ant=RX_Buffer;		
				}
				break;
			case 'G':
				OCR1B = adc_value;
				if(ant != RX_Buffer){
					SerialPort_Send_String("CAMBIANDO INTENSIDAD DEL VERDE\n\r");
					ant=RX_Buffer;
				}
				break;
			case 'B':
				OCR1A = adc_value;
				if(ant != RX_Buffer){
					SerialPort_Send_String("CAMBIANDO INTENSIDAD DEL AZUL\n\r");
					ant=RX_Buffer;
				}
				break;
		}
		_delay_us(16);
    }
}
ISR(USART_RX_vect){
	RX_Buffer = UDR0;
	/*act=RX_Buffer;
	RX_Buffer=0;*/
}
