#define F_CPU 16000000

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define kp 1
#define kd 1/4
#define ki 1/10

int controlVariable1;
int controlVariable2;
int position1;
int position2;
int interal;
int oldError;
int error;
int currentSpeed;
unsigned char pause = 0;

void initSystem();
void writeDataLCD(unsigned char data);
void writeCommandLCD(unsigned char command);
void writeUpperLine(char data[17]);
void writeLowwerLine(char data[17]);

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
}
ISR(INT1_vect){
  if((PIND>>5)&0x01){
    position2 ++;
  }else{
    position2 --;
  }
}

//Main function control by PID=================================
ISR(TIMER0_OVF_vect){
  int visai = 0;
  error = position1 - position2;
  interal += error;
  visai = error*kp  + (error - oldError)*kd + interal*ki;
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
  oldError = error;
}
//===========================================================================
ISR(TIMER0_COMPA_vect){
  PORTC &= (~0x0c);
}
ISR(TIMER0_COMPB_vect){
  PORTC &= (~0x03);
}
int main(){
  initSystem();
  writeUpperLine(" INITIALIZATION ");
  writeLowerLine("[              ]");
  writeCommandLCD(0xc1);
  writeDataLCD('=');
  writeDataLCD('=');
  writeDataLCD('=');
  _delay_ms(150);
  writeDataLCD('=');
  writeDataLCD('=');
  _delay_ms(100);
  writeDataLCD('=');
  writeDataLCD('=');
  writeDataLCD('=');
  writeDataLCD('=');
  _delay_ms(500);
  writeDataLCD('=');
  writeDataLCD('=');
  writeDataLCD('=');
  _delay_ms(350);
  writeDataLCD('=');
  writeDataLCD('=');
  writeUpperLine("Welcome to______");
  writeLowerLine("SteamisCool_____");
  _delay_ms(5000);
  controlVariable1 = 123;
  controlVariable2 = 123;
  writeCommandLCD(0x01);
  while(1){
    writeCommandLCD(0x80);
    writeDataLCD(controlVariable1/100 + 0x30);
    writeDataLCD((controlVariable1/10)%10 + 0x30);
    writeDataLCD((controlVariable1%10) + 0x30);
    if((PIND&(1<<7)) == 0){
      _delay_ms(20);
      while((PIND&(1<<7)) == 0);
      if(controlVariable1 < 245 && controlVariable1 >= 0){
        controlVariable1 += 10;
        controlVariable2 += 10;
      }else if(controlVariable1 > -245 && controlVariable1 <= 0){
        controlVariable1 -= 10;
        controlVariable2 -= 10;
      }
    }
    if((PIND&(1<<6)) == 0){
      _delay_ms(20);
      while((PIND&(1<<6)) == 0);
      if(controlVariable1 <= 255 && controlVariable1 > 30){
        controlVariable1 -= 10;
        controlVariable2 -= 10;
      }else if(controlVariable1 >= -255 && controlVariable1 < -30){
        controlVariable1 += 10;
        controlVariable2 += 10;
      }
    }
    if((PINB&(1<<5)) == 0){
      _delay_ms(20);
      while((PINB&(1<<5)) == 0);
      if(pause == 0){
        pause = 1;
      }else{
        pause = 0;
      }
    }
    if((PINB&(1<<4)) == 0){
      _delay_ms(20);
      while((PINB&(1<<4)) == 0);
      currentSpeed = controlVariable1;
      if(controlVariable1 > 0){
        while(1){
          if(controlVariable1 == 20){
            controlVariable1 = -20;
            controlVariable2 = -20;
            break;
          }
          controlVariable1 --;
          controlVariable2 --;
          _delay_ms(20);
        }
        while(1){
          if(controlVariable1 == -currentSpeed){
            break;
          }
          controlVariable1 --;
          controlVariable2 --;
          _delay_ms(20);
        }
      }else{
        while(1){
          if(controlVariable1 == -20){
            controlVariable1 = 20;
            controlVariable2 = 20;
            break;
          }
          controlVariable1 ++;
          controlVariable2 ++;
          _delay_ms(20);
        }
        while(1){
          if(controlVariable1 == -currentSpeed){
            break;
          }
          controlVariable1 ++;
          controlVariable2 ++;
          _delay_ms(20);
        }
      }
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
  DDRB = 0x0f;
  PORTB = 0xf0;
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
  PORTB = (data>>4) | 0xf0;
  PORTC |= (1<<5);
  _delay_ms(2);
  PORTC &= ~(1<<5);
  _delay_ms(2);
  PORTB = data |0xf0;
  PORTC |= (1<<5); 
  _delay_ms(2);
  PORTC &= ~(1<<5);
}
void writeCommandLCD(unsigned char command){
  PORTC &= ~(1<<4);
  PORTB = (command >> 4)|0xf0;
  PORTC |= (1<<5); 
  _delay_ms(2);
  PORTC &= ~(1<<5);
  _delay_ms(2);
  PORTB = command | 0xf0;
  PORTC |= (1<<5); 
  _delay_ms(2);
  PORTC &= ~(1<<5);
}
void writeUpperLine(char data[17]){
  writeCommandLCD(0x80);
  unsigned char i =0;
  for(i=0;i<16;i++){
    writeDataLCD(data[i]);
  }
}
void writeLowerLine(char data[17]){
  writeCommandLCD(0xc0);
  unsigned char i = 0;
  for(i=0;i<16;i++){
    writeDataLCD(data[i]);
  }
}
