#define F_CPU 16000000

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
int controlVariable1 = 0;
int controlVariable2 = 0;
int position1 = 0;
int oldError = 0;
int interal = 0;
int position1Request = 0;
unsigned int counter = 0;
int currentSpeed = 0;
int currentAcceleration = 0;
void initSystem();
void USART_Transmit(char data);
int pid(unsigned char kp, unsigned char ki, unsigned char kd);
ISR(TIMER0_OVF_vect){
  if(controlVariable1 > 0){
    OCR0A = (unsigned char)controlVariable1; 
    PORTC |= (1<<3);
  }else{
    OCR0A = (unsigned char)(-controlVariable1);
    PORTC |= (1<<2);
  }
  if(controlVariable2 > 0){
    OCR0B = (unsigned char)controlVariable2;
    PORTC |= (1<<1);
  }else{
    OCR0B = (unsigned char)(-controlVariable2);
    PORTC |= 1;
  }
}
ISR(TIMER0_COMPA_vect){
  PORTC &= (~0x0c);
}
ISR(TIMER0_COMPB_vect){
  PORTC &= (~0x03);
}
ISR(INT0_vect){
  if((PIND>>4)&0x01){
    position1 ++;
  }else{
    position1 --;
  }
}
ISR(INT1_vect){
  if((PIND>>5)&0x01){
    position1 ++;
  }else{
    position1 --;
  }
}
ISR(TIMER1_OVF_vect){
  TCNT1H = 0xff;
  TCNT1L = 0x50;
  position1Request += currentSpeed;
  if(position1Request > 20000){
    position1Request -= 20000;
    position1 -= 20000;
  }else if(position1Request < -20000){
    position1Request += 20000;
    position1 += 20000;
  }
}
int main(){
  initSystem();
  currentSpeed = 10;
  while(1){
    controlVariable1 = pid(10,0,10);
    USART_Transmit('=');
    USART_Transmit('\n');
    if(position1 < 0){
      USART_Transmit('-');
      USART_Transmit(-position1/1000 + 0x30);
      USART_Transmit((-position1/100)%10 + 0x30);
      USART_Transmit((-position1/10)%10 + 0x30);
      USART_Transmit((-position1%10) + 0x30);
      USART_Transmit('\n');
      }
    USART_Transmit(position1/1000 + 0x30);
    USART_Transmit((position1/100)%10 + 0x30);
    USART_Transmit((position1/10)%10 + 0x30);
    USART_Transmit((position1%10) + 0x30);
    USART_Transmit('\n');
    if(position1Request < 0){
      USART_Transmit('-');
      USART_Transmit(- position1Request/1000 + 0x30);
      USART_Transmit((- position1Request/100)%10 + 0x30);
      USART_Transmit((- position1Request/10)%10 + 0x30);
      USART_Transmit((- position1Request%10) + 0x30);
      USART_Transmit('\n');
      }
    USART_Transmit(position1Request/1000 + 0x30);
    USART_Transmit((position1Request/100)%10 + 0x30);
    USART_Transmit((position1Request/10)%10 + 0x30);
    USART_Transmit((position1Request%10) + 0x30);
    USART_Transmit('\n');
    USART_Transmit('=');
    USART_Transmit('\n');
  }
}

void initSystem(){
  //pinconfig
  DDRB = 0xff;
  DDRC = 0xff;
  DDRD = 0xff;
  //=========
  //timer0 with PWM
  TCCR0A = 0x00;
  TCCR0B = 0x01;
  TIMSK0 = 0x07;
  //timer1 normal mode
  TCCR1A = 0x00;
  TCCR1B = 0x05;
  TCNT1H = 0xff;
  TCNT1L = 0x50;
  TIMSK1 = 0x01;   //interrupt overflow timer1 enable
  //=========
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
int pid(unsigned char kp, unsigned char ki, unsigned char kd){
  int result = 0;
  counter ++;
  int error = position1Request - position1;
  interal += error;
  int interalAvg = interal;
  int divi = error - oldError;
  result = kp*error + ki*interalAvg + kd*divi;
  oldError = error;
  if(result > 255){
    result = 255;
  }else if(result < -255){
    result = -255;
  }
  return result;
}
