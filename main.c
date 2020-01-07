#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#define F_CPU 8000000UL
#include <util/delay.h>


typedef struct USARTRX{
	char receiver_buffer;	
	unsigned char status;	//reserva 1 byte
	unsigned char receive: 1; //reserva 1 bit	
	unsigned char error:  1;
	
	}USARTRX_st;

volatile USARTRX_st rxUSART = {0, 0, 0, 0};
	
//volatile char transmit_buffer[1];	
int i=0, pwm=0;

char adc1 = 1;
char adc2 = 0;

unsigned int LDR1, LDR2;
uint16_t LDR1an = 0; 
uint16_t LDR2an = 0;

uint16_t diff;

void adc_start(){
	ADCSRA |= (1<<ADSC); //começa conversão
}

volatile uint8_t flag_adc=0;

ISR(ADC_vect)    // Função de Interrupção do ADC
{	
	if(adc1==1){
		adc1=0;
		adc2=1;
		LDR1an = ADCL + (ADCH<<8);
		flag_adc = 1;
		ADMUX = (1<<MUX1) | (0<<MUX0);
	}
	else if(adc2==1){
		adc1=1;
		adc2=0;
		LDR2an = ADCL + (ADCH<<8);
		flag_adc = 1;
		ADMUX = (0<<MUX1) | (1<<MUX0);
	}
	
	diff = LDR2an - LDR1an;
}


ISR(USART_RX_vect)
{
	rxUSART.status = UCSR0A; // guarda flags
	
	if( rxUSART.status & ((1<FE0) | (1<<DOR0) | (1>>UPE0)))
		rxUSART.error = 1;
		
		rxUSART.receiver_buffer = UDR0;
		rxUSART.receive = 1;
		}

void init(){
	
	//USART spi
	UBRR0H = 0;
	UBRR0L = 103; //BAUD = 9600
	
	UCSR0A = 0b00000010; //(1 << U2X0); //Double Speed
	UCSR0B = 0b00001000; //(1 << TXEN0)
	UCSR0C = 0b00000110; //(1 << UCSZ01) | (1 << UCSZ00);
	
	//Configuração dos Portos
	DDRB = 0b01000000;		// Configura PORTB6 como output
	PORTB = 0b01000000;
	DDRC = 0b00011000;     //Configura PORTC3 e PORTC4 como output
	DDRD = 0b00000011;    //PORT6 e PORT7 como input / PD0 e PD1 USART output
	PORTD=0b00000011;
		
	/*
	//Timer0 LED a piscar à frequência de 1Hz
	OCR0A=156;
	TCCR0A |= (1<<WGM01); //CTC Mode
	TCCR0B |= (1<<CS02) | (1<<CS00); //Prescaler 1024
	TIMSK0 |= (1<<OCIE0A);
	*/
	
	//Configuração do ADC
	ADMUX = (0<<REFS0) | (1<<REFS1) | (0<<ADLAR) | (1<<MUX0); //define o AVcc como referencia e define adc1 como pino de entrada do adc, com ADLAR ajustado a esquerda
	ADCSRA = (1<<ADEN) | (1<<ADIE) | (0<<ADIF) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0);//define o começo do adc em modo continuo ativa as flags e define o prescaler para o adc para 128
	
	sei();		//Enable global das interrupções
}

void send_message(char *buffer)
{
	unsigned char i=0;
	while(buffer[i]!='\0')
	{
		while((UCSR0A & 1<<UDRE0)==0);
		UDR0=buffer[i];
		i++;
	}
}

void val_adc(){
	
	volatile char transmit_buffer[10];
	volatile char transmit_buffer1[10];
	volatile char transmit_buffer2[10];
	volatile char x[10];
	
	
	adc_start();
	while(flag_adc==0);
	flag_adc=0;
	sprintf(transmit_buffer, "LDR1: %u\r\n", LDR1an);
	send_message(transmit_buffer);
	sprintf(transmit_buffer1, "LDR2: %u\r\n", LDR2an);
	send_message(transmit_buffer1);
	sprintf(transmit_buffer2, "---------------\r\n");
	send_message(transmit_buffer2);
	sprintf(x, "%d\r\n", diff);
	send_message(x);
}


int val_sw(int sw1,int sw2){
	sw1 = PIND7;
	sw2 = PIND6;
	return sw1,sw2;
	
}

/*void Controlo_motor(){
	//uint32_t Dif_lux = LDR2an-LDR1an;
	int sw1=0, sw2=0;
	volatile char x[10];
	volatile char y[10];
	
	//val_sw( sw1, sw2);
	
	while(sw1==0 & (LDR2an-LDR1an>150)){					//sw1=0 ->nao estiver precionado. sw1=1 -> maximo aberto.
		PORTB |= (1<<PORTB3);
		val_sw(sw1,sw2);
		sprintf(x, "%u\r\n", LDR2an-LDR1an);
		send_message(x);
		
	}
	PORTB |= (0<<PORTB3);
	
	while(sw2==0 & (LDR2an-LDR1an<-150)){				//sw2=0 ->nao estiver precionado. sw2=1 -> maximo fechado.
		PORTB |= (1<<PORTB4);
		val_sw(sw1,sw2);
		sprintf(y, "claro\r\n");
		send_message(y);
	}
	
	PORTB |= (0<<PORTB4);

}
*/

int main(void)
{
	init();

	while(1)
	{	
		val_adc();	
		//Controlo_motor();
		
		_delay_ms(5000);
	}
}



