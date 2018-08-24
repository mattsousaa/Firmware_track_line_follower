/*
 * Pista_pancadao.c
 *
 * Created: 09/06/2018 21:58:48
 * Author : Mateus Sousa
 */ 

#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/delay.h>

void USART_init(void);
void USART_send(unsigned char data);
unsigned char USART_Receive(void);
void USART_putstring(char* StringPtr);

char entry_sensor1 = 0x00;
char entry_sensor2 = 0xFF;
char entry_sensor3 = 0xFF;

#define set_bit(y,bit) (y|=(1<<bit))
#define clr_bit(y,bit) (y&=~(1<<bit))
#define cpl_bit(y,bit) (y^=(1<<bit))
#define tst_bit(y,bit) (y&(1<<bit))
#define BAUDRATE 9600
#define BAUD_PRESCALLER (((F_CPU / (BAUDRATE * 16UL))) - 1)

ISR(PCINT0_vect){
  
  if (!(PINB & (1 << PINB0))){

  if(entry_sensor1 == 0x00){
    USART_send('1');
    //UCSR0B = (0<<TXEN0);
    //_delay_ms(10000);
    entry_sensor1 = 0xFF; 
    entry_sensor2 = 0x00;
  }
  }

  if (!(PINB & (1 << PINB1))){
    if(entry_sensor2 == 0x00){
      USART_send('2');
      //_delay_ms(10000);
      //cpl_bit(PORTB, 3);
      entry_sensor2 = 0xFF;
      entry_sensor3 = 0x00;
    }
  }

  if (!(PINB & (1 << PINB2))){
    if(entry_sensor3 == 0x00){
      USART_send('3');
      //_delay_ms(10000);
      //cpl_bit(PORTB, 3);
      entry_sensor1 = 0x00;
      entry_sensor2 = 0x00;
      entry_sensor3 = 0xFF;
    }
  }
}

void setup_uc(void){

  /************IO PORT CONFIG************/
  DDRB = 0b00001000;
  //SET PORTB initial value
  PORTB = 0b00000111; //Internal Pull-up
  
  PCICR |= (1 << PCIE0);
  
  //Config INT0 and INT1 active on falling edge
  EICRA = 0b00001010;
  PCMSK0 = 0b00000111;
  
}

int main(void){
  
  setup_uc();
  
  USART_init();
  
  sei();
  
    /* Replace with your application code */
    while (1){
    }
}

void USART_init(void){
  
  /************************************************************************/
  /*UART                                                              */
  /************************************************************************/
  
  /*Set baud rate */
  UBRR0H = (uint8_t)(BAUD_PRESCALLER>>8);
  UBRR0L = (uint8_t)(BAUD_PRESCALLER);

  UCSR0B = (1<<RXEN0)|(1<<TXEN0)|(1 << RXCIE0);
  /* Set frame format: 8data, 2stop bit */
  UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);  //8 bit data format
  
}
//------------------------------------------------------------------------------------
void USART_send(unsigned char data){
  /* Wait for empty transmit buffer */
  while(!(UCSR0A & (1<<UDRE0)));
  
  /* Put data into buffer, sends the data */
  UDR0 = data;
}

//------------------------------------------------------------------------------------
unsigned char USART_Receive(void){
  /* Wait for data to be received */
  while (!(UCSR0A & (1<<RXC0)));
  /* Get and return received data from buffer */
  return UDR0;
}
//------------------------------------------------------------------------------------
void USART_putstring(char* StringPtr){
  // sends the characters from the string one at a time to the USART
  while(*StringPtr != 0x00)
  {
    USART_send(*StringPtr);
    StringPtr++;
  }
}

