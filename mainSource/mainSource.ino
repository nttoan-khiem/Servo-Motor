#define F_CPU 16000000

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
int controlVariable1 = 0;
int controlVariable2 = 0;
int position1 = 0;
int position2 = 0;
float oldError = 0;
double interal = 0;
double positionRequest = 0;
unsigned int counter = 0;
float currentSpeed = 0;
float currentAcceleration = 0;
void initSystem();
void USART_Transmit(char data);
float pid(float kp, float ki, float kd);
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
    position2 ++;
  }else{
    position2 --;
  }
}
int main(){
  initSystem();
  while(1){
    controlVariable1 = 100;
    controlVariable2 = pid(5,0,0);
    USART_Transmit('=');
    USART_Transmit('\n');
    if(position1 < 0){
      USART_Transmit('-');
      USART_Transmit((int)(-position1)/1000 + 0x30);
      USART_Transmit(((int)(-position1)/100)%10 + 0x30);
      USART_Transmit(((int)(-position1)/10)%10 + 0x30);
      USART_Transmit(((int)(-position1)%10) + 0x30);
      USART_Transmit('\n');
      }else{
    USART_Transmit((int) position1/1000 + 0x30);
    USART_Transmit(((int) position1/100)%10 + 0x30);
    USART_Transmit(((int) position1/10)%10 + 0x30);
    USART_Transmit(((int) position1%10) + 0x30);
    USART_Transmit('\n');
      }
    if(position2 < 0){
      USART_Transmit('-');
      USART_Transmit((int)(-position2)/1000 + 0x30);
      USART_Transmit(((int)(-position2)/100)%10 + 0x30);
      USART_Transmit(((int)(-position2)/10)%10 + 0x30);
      USART_Transmit(((int)(-position2)%10) + 0x30);
      USART_Transmit('\n');
      }else{
    USART_Transmit((int) position2/1000 + 0x30);
    USART_Transmit(((int) position2/100)%10 + 0x30);
    USART_Transmit(((int) position2/10)%10 + 0x30);
    USART_Transmit(((int) position2%10) + 0x30);
    USART_Transmit('\n');
      }
  /*  if(positionRequest < 0){
      USART_Transmit('-');
      USART_Transmit((int)(- positionRequest)/1000 + 0x30);
      USART_Transmit(((int)(- positionRequest)/100)%10 + 0x30);
      USART_Transmit(((int)(- positionRequest)/10)%10 + 0x30);
      USART_Transmit(((int)(- positionRequest)%10) + 0x30);
      USART_Transmit('\n');
      }
    USART_Transmit((int) positionRequest/1000 + 0x30);
    USART_Transmit(((int) positionRequest/100)%10 + 0x30);
    USART_Transmit(((int) positionRequest/10)%10 + 0x30);
    USART_Transmit(((int) positionRequest%10) + 0x30);
    USART_Transmit('\n');
    USART_Transmit('=');
    USART_Transmit('\n'); */
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
float pid(float kp, float ki, float kd){
  float result = 1.0;
  float error =1.0*(position1 - position2);
  interal += error;
  if(interal > 20){
    interal = 20;
  }else if(interal < -20){
    interal = -20;
  }
  float divi = error - oldError;
  result = kp*error + ki*interal + kd*divi;
  oldError = error;
  if(result > 255){
    result = 255;
  }else if(result < -255){
    result = -255;
  }
  return result;
}
