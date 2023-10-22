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
int currentSpeed;
unsigned char pause = 0;
int timerSys = 0;
int timerChange = 0;
int poiter = 0;
char upperLine[50];
char lowerLine[50];

void initSystem();
void writeDataLCD(unsigned char data);
void writeCommandLCD(unsigned char command);
void writeUpperLine(char data[17]);
void writeLowwerLine(char data[17]);
ISR(INT0_vect){   //PD2 pin 2
  if((PIND>>4)&0x01){  //PD4 pin4
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
ISR(INT1_vect){   //PD3 pin 3
  if((PIND>>5)&0x01){ //PD5 pin 5
    position2 ++;
  }else{
    position2 --;
  }
}
ISR(TIMER0_OVF_vect){
  int visai = 0;
  error = position1 - position2;
  interal += error;
  visai = error;   //+ (error - oldError)/2 + interal/4;
  int controlVariable2Main = controlVariable2 + visai; 
  if(controlVariable2Main == 0 && controlVariable1 >= 0){
    controlVariable2Main = 1;
  }else if(controlVariable2Main == 0 && controlVariable1 <= 0) {
    controlVariable2Main = -1;
  }
  if(controlVariable1 >= 0){
    OCR0A = (unsigned char)controlVariable1; 
    if(pause == 0){
      PORTC |= (1<<3);
    }
  }else{
    OCR0A = (unsigned char)(-controlVariable1);
    if(pause == 0){
    PORTC |= (1<<2);
    }
  }
  if(controlVariable2Main >= 0){
    OCR0B = (unsigned char)(controlVariable2Main);
    if(pause == 0){
    PORTC |= (1<<1);
    }
  }else{
    OCR0B = (unsigned char)(-(controlVariable2Main));
    if(pause == 0){
    PORTC |= 1;
    }
  }
  oldError = error;
}
ISR(TIMER0_COMPA_vect){
  PORTC &= (~0x0c);
}
ISR(TIMER0_COMPB_vect){
  PORTC &= (~0x03);
}
ISR(TIMER1_OVF_vect){
  TCNT1H = 0xff;
  TCNT1L = 0x63;
  timerSys ++;
  timerChange ++;
  if(timerSys > 3000){
    timerSys = 0;
    lowerLine[0]=' '; lowerLine[1]=' '; lowerLine[2]=' ';
    lowerLine[3]=' '; lowerLine[4]=' '; lowerLine[5]=' ';
    lowerLine[6]=' '; lowerLine[7]=' '; lowerLine[8]=' ';
    lowerLine[9]=' '; lowerLine[10]=' '; lowerLine[11]='H';
    lowerLine[12]='e'; lowerLine[13]='l'; lowerLine[14]='l';
    lowerLine[15]='o'; lowerLine[16]=' '; lowerLine[17]='W';
    lowerLine[18]='e'; lowerLine[19]='l'; lowerLine[20]='c';
    lowerLine[21]='o'; lowerLine[22]='m'; lowerLine[23]='e';
    lowerLine[24]=' '; lowerLine[25]='t'; lowerLine[26]='o';
    lowerLine[27]=' '; lowerLine[28]='S'; lowerLine[29]='t';
    lowerLine[30]='e'; lowerLine[31]='a'; lowerLine[32]='m';
    lowerLine[33]='I'; lowerLine[34]='s'; lowerLine[35]='C';
    lowerLine[36]='o'; lowerLine[37]='o'; lowerLine[38]='l';
    lowerLine[39]=' '; lowerLine[40]=' '; lowerLine[41]=' ';
    lowerLine[42]=' '; lowerLine[43]=' '; lowerLine[44]=' ';
    lowerLine[45]=' '; lowerLine[46]=' '; lowerLine[47]=' ';
    lowerLine[48]=' '; lowerLine[49]=' ';
  }
  if(timerChange > 40){
    poiter ++;
    if(poiter > 34){
      poiter = 0;
    }
    timerChange = 0;
  }
}
int main(){
  initSystem();
  writeCommandLCD(0x01);
  writeUpperLine(" INITIALIZATION ");
  writeLowerLine("[              ]");
  writeCommandLCD(0xc1);
  writeDataLCD('=');
  writeDataLCD('=');
  writeDataLCD('=');
  _delay_ms(15);
  writeDataLCD('=');
  writeDataLCD('=');
  _delay_ms(10);
  writeDataLCD('=');
  writeDataLCD('=');
  writeDataLCD('=');
  writeDataLCD('=');
  _delay_ms(50);
  writeDataLCD('=');
  writeDataLCD('=');
  writeDataLCD('=');
  _delay_ms(35);
  writeDataLCD('=');
  writeDataLCD('=');
  _delay_ms(20);
  upperLine[0]='W'; upperLine[1]='e'; upperLine[2]='l';
  upperLine[3]='c'; upperLine[4]='o'; upperLine[5]='m';
  upperLine[6]='e'; upperLine[7]=' '; upperLine[8]='t';
  upperLine[9]='o'; upperLine[10]='_'; upperLine[11]='_';
  upperLine[12]='_'; upperLine[13]='_'; upperLine[14]='_';
  upperLine[15]='_';
  lowerLine[0]='S'; lowerLine[1]='t'; lowerLine[2]='e';
  lowerLine[3]='a'; lowerLine[4]='m'; lowerLine[5]='I';
  lowerLine[6]='s'; lowerLine[7]='C'; lowerLine[8]='o';
  lowerLine[9]='o'; lowerLine[10]='l'; lowerLine[11]='_';
  lowerLine[12]='_'; lowerLine[13]='_'; lowerLine[14]='_';
  lowerLine[15]='_';
  //====================================================EFFECT=========
  int i = 0;
  int j = 0; 
  for(i=0;i<16;i++){
    writeCommandLCD(0x80+i);
    writeDataLCD(upperLine[i]);
    writeCommandLCD(0xc0+i);
    writeDataLCD(lowerLine[i]);
  }
  //================================================Effect==========
  writeCommandLCD(0x80);
  writeDataLCD(' ');
  writeDataLCD(' ');
  writeDataLCD(' ');
  _delay_ms(100);
  for(i=1;i<=16;i++){
    writeCommandLCD(0x80+i-1);
    writeDataLCD(upperLine[i-1]);
    writeDataLCD(' '); 
    writeDataLCD(' ');
    writeDataLCD(' ');
    _delay_ms(100);
  }
  writeCommandLCD(0xC0);
  writeDataLCD(' ');
  writeDataLCD(' ');
  writeDataLCD(' ');
  _delay_ms(100);
  for(i=1;i<=16;i++){
    writeCommandLCD(0xc0+i-1);
    writeDataLCD(lowerLine[i-1]);
    writeDataLCD(' '); 
    writeDataLCD(' ');
    _delay_ms(100);
  }
  for(i=0;i<3;i++){
    _delay_ms(280);
    writeCommandLCD(0x08);
    _delay_ms(200);
    writeCommandLCD(0x0c);
  }
  SREG |= 0x80;
  //=================================================================
  controlVariable1 = 120;
  controlVariable2 = 120;
  writeCommandLCD(0x01);
  upperLine[0]='S'; upperLine[1]='p'; upperLine[2]='e';
  upperLine[3]='e'; upperLine[4]='d'; upperLine[5]=':';
  upperLine[6]=' '; upperLine[7]=' '; upperLine[8]=' ';
  upperLine[9]=' '; upperLine[10]=' '; upperLine[11]=' ';
  upperLine[12]=' '; upperLine[13]=' '; upperLine[14]=' ';
  upperLine[15]=' ';
  writeCommandLCD(0x80);
  for(i=0;i<16;i++){
    writeDataLCD(upperLine[i]);
  }
  lowerLine[0]=' '; lowerLine[1]=' '; lowerLine[2]=' ';
  lowerLine[3]=' '; lowerLine[4]=' '; lowerLine[5]=' ';
  lowerLine[6]=' '; lowerLine[7]=' '; lowerLine[8]=' ';
  lowerLine[9]=' '; lowerLine[10]=' '; lowerLine[11]='H';
  lowerLine[12]='e'; lowerLine[13]='l'; lowerLine[14]='l';
  lowerLine[15]='o'; lowerLine[16]=' '; lowerLine[17]='W';
  lowerLine[18]='e'; lowerLine[19]='l'; lowerLine[20]='c';
  lowerLine[21]='o'; lowerLine[22]='m'; lowerLine[23]='e';
  lowerLine[24]=' '; lowerLine[25]='t'; lowerLine[26]='o';
  lowerLine[27]=' '; lowerLine[28]='S'; lowerLine[29]='t';
  lowerLine[30]='e'; lowerLine[31]='a'; lowerLine[32]='m';
  lowerLine[33]='I'; lowerLine[34]='s'; lowerLine[35]='C';
  lowerLine[36]='o'; lowerLine[37]='o'; lowerLine[38]='l';
  lowerLine[39]=' '; lowerLine[40]=' '; lowerLine[41]=' ';
  lowerLine[42]=' '; lowerLine[43]=' '; lowerLine[44]=' ';
  lowerLine[45]=' '; lowerLine[46]=' '; lowerLine[47]=' ';
  lowerLine[48]=' '; lowerLine[49]=' ';
  while(1){
    writeCommandLCD(0xc0);
    for(i=0;i<16;i++){
       writeDataLCD(lowerLine[poiter+i]);
    }
    upperLine[0]='S'; upperLine[1]='p'; upperLine[2]='e';
    upperLine[3]='e'; upperLine[4]='d'; upperLine[5]=':';
    upperLine[6]=' '; upperLine[7]=' '; upperLine[8]=' ';
    upperLine[9]=' '; upperLine[10]=' '; upperLine[11]=' ';
    upperLine[12]=' '; upperLine[13]=' '; upperLine[14]=' ';
    upperLine[15]=' ';
    writeCommandLCD(0x80);
    for(i=0;i<16;i++){
      writeDataLCD(upperLine[i]);
    }
    writeCommandLCD(0x80+6);
    if(controlVariable1 > 0){
      writeDataLCD(controlVariable1/100 + 0x30);
      writeDataLCD((controlVariable1/10)%10 + 0x30);
      writeDataLCD((controlVariable1%10) + 0x30);
      writeDataLCD(0x7e);
    }else{
      writeDataLCD(-controlVariable1/100 + 0x30);
      writeDataLCD((-controlVariable1/10)%10 + 0x30);
      writeDataLCD((-controlVariable1%10) + 0x30);
      writeDataLCD(0x7f);
    }
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
      while(1){
        if(controlVariable1 > 0){
          controlVariable1 --;
          _delay_ms(30);
          if(controlVariable1 == 30){
            break;
          }
        }else{
          controlVariable1 ++;
          _delay_ms(30);
          if(controlVariable1 == -30){
            break;
          }
        }
      }
      lowerLine[0]=' '; lowerLine[1]=' '; lowerLine[2]=' ';
      lowerLine[3]=' '; lowerLine[4]=' '; lowerLine[5]=' ';
      lowerLine[6]=' '; lowerLine[7]=' '; lowerLine[8]=' ';
      lowerLine[9]=' '; lowerLine[10]=' '; lowerLine[11]=' ';
      lowerLine[12]='S'; lowerLine[13]='y'; lowerLine[14]='s';
      lowerLine[15]='t'; lowerLine[16]='e'; lowerLine[17]='m';
      lowerLine[18]=' '; lowerLine[19]='i'; lowerLine[20]='s';
      lowerLine[21]=' '; lowerLine[22]='p'; lowerLine[23]='a';
      lowerLine[24]='u'; lowerLine[25]='s'; lowerLine[26]='e';
      lowerLine[27]=' '; lowerLine[28]=':'; lowerLine[29]=')';
      lowerLine[30]=')'; lowerLine[31]=' '; lowerLine[32]=' ';
      lowerLine[33]=' '; lowerLine[34]=' '; lowerLine[35]=' ';
      lowerLine[36]=' '; lowerLine[37]=' '; lowerLine[38]=' ';
      lowerLine[39]=' '; lowerLine[40]=' '; lowerLine[41]=' ';
      lowerLine[42]=' '; lowerLine[43]=' '; lowerLine[44]=' ';
      lowerLine[45]=' '; lowerLine[46]=' '; lowerLine[47]=' ';
      lowerLine[48]=' '; lowerLine[49]=' ';
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
      lowerLine[0]=' '; lowerLine[1]=' '; lowerLine[2]=' ';
      lowerLine[3]=' '; lowerLine[4]=' '; lowerLine[5]=' ';
      lowerLine[6]=' '; lowerLine[7]=' '; lowerLine[8]='I';
      lowerLine[9]='n'; lowerLine[10]='v'; lowerLine[11]='e';
      lowerLine[12]='r'; lowerLine[13]='t'; lowerLine[14]=' ';
      lowerLine[15]='i'; lowerLine[16]='s'; lowerLine[17]=' ';
      lowerLine[18]='d'; lowerLine[19]='o'; lowerLine[20]='n';
      lowerLine[21]='e'; lowerLine[22]=','; lowerLine[23]=' ';
      lowerLine[24]='a'; lowerLine[25]='m'; lowerLine[26]=' ';
      lowerLine[27]='I'; lowerLine[28]=' '; lowerLine[29]='w';
      lowerLine[30]='o'; lowerLine[31]='r'; lowerLine[32]='k';
      lowerLine[33]='i'; lowerLine[34]='n'; lowerLine[35]='g';
      lowerLine[36]=' '; lowerLine[37]='w'; lowerLine[38]='e';
      lowerLine[39]='l'; lowerLine[40]='l'; lowerLine[41]=' ';
      lowerLine[42]='?'; lowerLine[43]=' '; lowerLine[44]=' ';
      lowerLine[45]=' '; lowerLine[46]=' '; lowerLine[47]=' ';
      lowerLine[48]=' '; lowerLine[49]=' ';
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
  //timer1 normal mode
  TCCR1A = 0x00;
  TCCR1B = 0x05;
  TCNT1H = 0xff;
  TCNT1L = 0x63;
  TIMSK1 = 0x01;
  //=========
  //Pinconfig
  DDRB = 0x0f;
  PORTB = 0xf0;
  DDRD = 0x00;  //All pin of port D is input pin
  PORTD = 0xff; // pull up 
  DDRC = 0xff;  
  //==========
  writeCommandLCD(0x02);
  writeCommandLCD(0x28);
  writeCommandLCD(0x0c);
  //Init USART
  UBRR0H = (unsigned char)((F_CPU/16/9600 - 1)>>8);
  UBRR0L = (unsigned char)(F_CPU/16/9600 -1);
  UCSR0B |= (1<<4); //enable RX bit 4
  UCSR0B |= (1<<3); //enable TX bit 3
  UCSR0A |= (1<<6);
  UCSR0C = 0b00000110;  //Asynchronous USART //disable parity //1 bit stop //8bit data // polo raide
  //==========
}
void writeDataLCD(unsigned char data){
  PORTC |= (1<<4);
  PORTB = (data>>4) | 0xf0;
  PORTC |= (1<<5);
  _delay_us(200);
  PORTC &= ~(1<<5);
  _delay_us(200);
  PORTB = data |0xf0;
  PORTC |= (1<<5); 
  _delay_us(200);
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
