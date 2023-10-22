#define F_CPU 16000000

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

void initSystem();
void writeDataLCD(unsigned char data);
void writeCommandLCD(unsigned char command);

int main(){
  initSystem();
  while(1){
    writeDataLCD('f');
    _delay_ms(1000);
  }
}

void initSystem(){
  //Pinconfig
  DDRB = 0x0f;
  DDRD = 0x00;  //All pin of port D is input pin
  PORTD = 0xff; // pull up 
  DDRC = 0xff;  
  //==========
  SREG |= 0x80;
  writeCommandLCD(0x02);
  writeCommandLCD(0x28);
  writeCommandLCD(0x0c);
}
void writeDataLCD(unsigned char data){
  PORTC |= (1<<4);
  PORTB = (data>>4);
  PORTC |= (1<<5);
  _delay_ms(2);
  PORTC &= ~(1<<5);
  _delay_ms(2);
  PORTB = data;
  PORTC |= (1<<5); 
  _delay_ms(2);
  PORTC &= ~(1<<5);
}
void writeCommandLCD(unsigned char command){
  PORTC &= ~(1<<4);
  PORTB = (command >> 4);
  PORTC |= (1<<5); 
  _delay_ms(2);
  PORTC &= ~(1<<5);
  _delay_ms(2);
  PORTB = command;
  PORTC |= (1<<5); 
  _delay_ms(2);
  PORTC &= ~(1<<5);
}
