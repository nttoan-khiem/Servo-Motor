#define F_CPU 16000000

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

int controlVariable1;
int controlVariable2;
int position1;
int position2;
int interal;
int oldError;
int error;

void USART_Transmit(char data);

ISR(INT0_vect){
  if((PIND>>4)&0x01){
    position1 ++;
  }else{
    position1 --;
  }
  if(position1 >= 2000){
    position1 -= 2000;
    position2 -= 2000; 
  }else if(position1 <= -2000){
    position1 += 2000;
    position2 += 2000;
  }
  USART_Transmit(position1);
  USART_Transmit(position2);
}
ISR(INT1_vect){
  if((PIND>>5)&0x01){
    position2 ++;
  }else{
    position2 --;
  }
  USART_Transmit(position1);
  USART_Transmit(position2);
}
ISR(TIMER0_OVF_vect){
  int visai = 0;
  error = position1 - position2;
  visai = error;
  int controlVariable2Main = controlVariable2 + visai; 
  if(controlVariable2Main == 0 && controlVariable1 >= 0){
    controlVariable2Main = 1;
  }else if(controlVariable2Main == 0 && controlVariable1 <= 0) {
    controlVariable2Main = -1;
  }
  if(controlVariable1 >= 0){
    OCR0A = (unsigned char)controlVariable1; 
    PORTC |= (1<<3); 
  }else{
    OCR0A = (unsigned char)(-controlVariable1);
    PORTC |= (1<<2);
  }
  if(controlVariable2Main >= 0){
    OCR0B = (unsigned char)(controlVariable2Main);
    PORTC |= (1<<1);
  }else{
    OCR0B = (unsigned char)(-(controlVariable2Main));
    PORTC |= 1;
  }
}
ISR(TIMER0_COMPA_vect){
  PORTC &= (~0x0c);
}
ISR(TIMER0_COMPB_vect){
  PORTC &= (~0x03);
}
int main(){
  initSystem();
  controlVariable1 = 50;
  controlVariable2 = 50;
  while(1){
    while(1){
      controlVariable1 ++;
      controlVariable2 ++;
      if(controlVariable1 == -10){
        controlVariable1 = 10;
        controlVariable2 = 10;
      }
      if(controlVariable1 == 250) break;
      _delay_ms(10);
    }
    while(1){
      controlVariable1 --;
      controlVariable2 --;
      if(controlVariable1 == 10){
        controlVariable1 = -10;
        controlVariable2 = -10;
      }
      if(controlVariable1 == -250) break;
      _delay_ms(10);
    }
  }
}

void initSystem(){
  //=========
  //timer0 with PWM
  TCCR0A = 0x00;
  TCCR0B = 0x03;
  TIMSK0 = 0x07;
  //Interrupt External
  EICRA = 0x0f; // interrupt signal trigger RAISING
  EIMSK = 0x03; // interrupt mask register INT1 INT0
  //=========
  //Pinconfig
  DDRD = 0x00;  //All pin of port D is input pin
  PORTD = 0xff; // pull up 
  DDRC = 0xff;  //All pin of port C is output pin
  //Init USART
  UBRR0H = (unsigned char)((F_CPU/16/9600 - 1)>>8);
  UBRR0L = (unsigned char)(F_CPU/16/9600 -1);
  UCSR0B |= (1<<4); //enable RX bit 4
  UCSR0B |= (1<<3); //enable TX bit 3
  UCSR0A |= (1<<6);
  UCSR0C = 0b00000110;  //Asynchronous USART //disable parity //1 bit stop //8bit data // polo raide
  //==========
  SREG |= 0x80;
}
void USART_Transmit(char data)
{
  /* Wait for empty transmit buffer */
  while ( !( UCSR0A & (1<<5)) );
  /* Put data into buffer, sends the data */
  UDR0 = data;
}
