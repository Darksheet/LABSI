#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#define F_CPU 8000000UL
#include <util/delay.h>
#define BAUDRATE 9600
#define BAUD_PRESCALLER (((F_CPU / (BAUDRATE * 16UL))) - 1)
//#define BAUD 9600


//#include <avr/pgmspace.h>

typedef struct USARTRX{
	char receiver_buffer;	
	unsigned char status;	//reserva 1 byte
	unsigned char receive: 1; //reserva 1 bit	
	unsigned char error:  1;
	}USARTRX_st;

volatile USARTRX_st rxUSART = {0, 0, 0, 0};
	
//volatile char transmit_buffer[1];	


int i=0, pwm=0;

unsigned char dado[4096];

unsigned int LDR1, LDR2;
uint16_t LDR1an = 0; 
uint16_t LDR2an = 0;

void adc_start(){
	ADCSRA |= (1<<ADSC); //começa conversão

}

volatile uint8_t flag_adc=0;

ISR(ADC_vect)    // Função de Interrupção do ADC
{
	/*
	pwm = (ADCH); //Guarda o valor obtido pela leitura
	adc_start();  //Inicializa Leitura pelo ADC*/
	static uint8_t first_conversion = 1;
	
	 LDR1an = ADCL + (ADCH<<8);
	 flag_adc = 1;
	
	/*
	if(ADMUX == 1) // channel 1
	{
		if(first_conversion == 2)
		{
			//pb1 = ADCH;
			//pb2 = ADCL;
			// LDR1an = ADCL>>6 | ADCH<<2;
			 LDR1an = ADCL + ADCH<<8;
			 
			ADMUX = (1<<MUX1) | (0<<MUX0); // switch ADC to channel 2
			first_conversion = 1;
			flag_adc = 1;
		}
		else first_conversion++;
	}
	else if(ADMUX == 2) // channel 2
	{
		if(first_conversion == 2)
		{
			//if(pot_flag == 1) 
			//pb1 = ADCH;
			//pb2 = ADCL;
			LDR2an = ADCL + ADCH<<8;// ADCL>>6 | ADCH<<2;

			ADMUX = (0<<MUX1) | (1<<MUX0); // switch back to channel 0
			
			first_conversion = 1;
			flag_adc = 1;
		}
		else first_conversion++;
	}
	 //LDR1 =  (LDR1an / 1024) * 5;
	 //LDR2 =  (LDR2an / 1024) * 5;
	 //LDR1 = LDR1an;
	 LDR1an = 256;*/
}

// Função receber dados

/*void rec_dad(){
	
	unsigned char i = 0;
	
	while(dado[i] != '\0'){
		while(UCSR0A & (1<<UDRE0)==0);
		UDR0 = dado[i];
		i++;
	}
	//sprintf("%d", LDR1an);
}*/

ISR(USART_RX_vect)
{
	rxUSART.status = UCSR0A; // guarda flags
	
	if( rxUSART.status & ((1<FE0) | (1<<DOR0) | (1>>UPE0)))
		rxUSART.error = 1;
		
		rxUSART.receiver_buffer = UDR0;
		rxUSART.receive = 1;
		
		
		
		
		}
		


/*
ISR(TIMER2_OVF_vect){
	OCR2A = pwm; // Valor lido pelo ADC
	
}
*/

void init(){
	
	//USART spi
	UBRR0H = 0;
	UBRR0L = 103; //BAUD = 9600
	
	//UBRR0H = (uint8_t)(BAUD_PRESCALLER>>8);
	//UBRR0L = (uint8_t)(BAUD_PRESCALLER);
	UCSR0A = 0b00000010; //(1 << U2X0); //Double Speed
	UCSR0B = 0b00011000; //(1 << RXEN0) | (1 << RXCIE0); 
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
	
	/*
	//Timer2 Regulação de Luminosidade do LED
	TCCR2A |= (1<<COM2A1) | (1<<WGM20) | (1<<WGM21); //Fast pwm mode, clear 0CA2 on compare match
	TCCR2B |= (1<<CS20);  //Sem prescaler
	TIMSK2 |= (1<<TOIE2);
	OCR2A = 0; //Começa a 0
	*/
	
	//Configuração do ADC1
	ADMUX = (0<<REFS0) | (1<<REFS1) | (0<<ADLAR) | (1<<MUX0); //define o AVcc como referencia e define adc1 como pino de entrada do adc, com ADLAR ajustado a esquerda
	ADCSRA = (1<<ADEN) | (1<<ADIE) | (0<<ADIF) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0);//define o começo do adc em modo continuo ativa as flags e define o prescaler para o adc para 128
	//adc_start();
	//Configuração do ADC2
	//ADMUX = (0<<REFS0) | (0<<REFS1) | (0<<ADLAR) | (1<<MUX1); //define o AVcc como referencia e define adc2 como pino de entrada do adc, com ADLAR ajustado a esquerda
	//ADCSRA = (1<<ADEN) | (1<<ADIE) | (1<<ADIF) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0);//define o começo do adc em modo continuo ativa as flags e define o prescaler para o adc para 128
	//adc_start();
	
	
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

int main()
{
	init();
	volatile char transmit_buffer[10];
	
	
	
	while(1)
	{
		adc_start();
		while(flag_adc==0);
		flag_adc=0;
		sprintf(transmit_buffer, "%u\r\n", LDR1an);
		send_message(transmit_buffer);
		
		/*if(rxUSART.receive == 1) // verifica se existe novos dados recebidos
		{
			if(rxUSART.error == 1) // verifica se existe erros
				{
					//procedimentos para tratar erros
					rxUSART.error = 0;
				}
				else {
					sprintf(transmit_buffer, "Tecla: %c\r\n", rxUSART.receiver_buffer);
				}
				rxUSART.receive = 0;
		}*/
		/*void rec_dad();
		printf("%d",LDR1an);
		//printf("%d",LDR2);*/
		
	}
}



