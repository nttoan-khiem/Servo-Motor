#define F_CPU 16000000

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

int controlVariable1;
int controlVariable2;
int giaToc = 10;
unsigned char errorSys = 0;
unsigned long timeToMove1 = 0;
unsigned char slowDone = 0;
unsigned char speedDone = 0;
unsigned long long position1 = 0;
unsigned long long position2 = 0;
unsigned int timerChange = 0;
unsigned int poiter = 0;
unsigned char speedUp = 0;
unsigned char slowDown = 0;
unsigned char invert = 0;
unsigned char doneInvert = 0;
unsigned char doneNear = 0;
unsigned char doneFar = 0;
int oldError;
int error;
int currentSpeed;
unsigned char pause = 0;
unsigned long timerSys = 0;
unsigned char coreControl = 1;
char uperLine[16];
char lowerLine[16];
void initSystem();
void writeDataLCD(unsigned char data);
void writeCommandLCD(unsigned char command);
void near();
void far();
void halfFar(unsigned long long time);
ISR(INT0_vect){   //PD2 pin 2
  if((PIND>>4)&0x01){  //PD4 pin4
    position1 ++;
    if(controlVariable1 < 0){
      errorSys = 1;
    }
  }else{
    position1 --;
    if(controlVariable1 > 0){
      errorSys = 1;
    }
  }
}
ISR(INT1_vect){   //PD3 pin 3
  if((PIND>>5)&0x01){ //PD5 pin 5
    position2 ++;
    if(controlVariable1 < 0){
      errorSys = 1;
    }
  }else{
    position2 --;
    if(controlVariable1 > 0){
      errorSys = 1;
    }
  }
}
ISR(TIMER0_OVF_vect){
  int visai = 0;
  error = position1 - position2;
  visai = error*4;   //+ (error - oldError)/2 + interal/4;
  if(visai > 50){
    visai = 50;
  }else if(visai < -50){
    visai = -50;
  }
  if(coreControl == 0){
    visai = 0;
  }
  int controlVariable2Main = controlVariable2 + visai; 
  if(controlVariable2Main >= 240){
    controlVariable2Main = 240;
  }else if(controlVariable2Main <= -240){
    controlVariable2Main = -240;
  }
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
  if(timerChange > 30){
    poiter ++;
    if(poiter > 84){
      poiter = 0;
    }
    timerChange = 0;
  }
  if(timerSys % 5 == 0 && controlVariable1 <= 230 && speedUp == 1){  //speed up function
    controlVariable1 += giaToc;
    controlVariable2 = controlVariable1;
    if(controlVariable1 >= 230){
      speedUp = 0;
      speedDone = 1;
    }
  }
  if(timerSys % 5 == 0 && controlVariable1 >= 30 && slowDown == 1){    //slowdown function
    controlVariable1 -= giaToc;
    controlVariable2 = controlVariable1;
    if(controlVariable1 <= 30){
      slowDown = 0;
      slowDone = 1;
    }
  }
  if(invert == 1){
    if(timerSys % 5 == 0){
      if(controlVariable1 > -230){
        controlVariable1 -= giaToc;
        controlVariable2 = controlVariable1;
      }else{
        invert = 2; 
      }
    }
  }
  if(invert == 2){
    if(timerSys % 5 == 0){
      if(controlVariable1 < 230){
        controlVariable1 += giaToc;
        controlVariable2 = controlVariable1;
      }else{
        invert = 0; 
      }
    }
  }
}
int main(){
  initSystem();
  int counter = 0;
  uperLine[0]='H'; uperLine[1]='o'; uperLine[2]='l';
  uperLine[3]='d'; uperLine[4]=' '; uperLine[5]='k';
  uperLine[6]='e'; uperLine[7]='y'; uperLine[8]=' ';
  uperLine[9]=0x7e; uperLine[10]=' '; uperLine[11]='t';
  uperLine[12]='o'; uperLine[13]=' '; uperLine[14]=' ';
  uperLine[15]=' ';
  writeCommandLCD(0x80);
  for(counter = 0; counter < 16; counter ++){
    writeDataLCD(uperLine[counter]);
  }
  lowerLine[0]='s'; lowerLine[1]='w'; lowerLine[2]='i';
  lowerLine[3]='t'; lowerLine[4]='c'; lowerLine[5]='h';
  lowerLine[6]=' '; lowerLine[7]='t'; lowerLine[8]='o';
  lowerLine[9]=' '; lowerLine[10]='m'; lowerLine[11]='a';
  lowerLine[12]='n'; lowerLine[13]='u'; lowerLine[14]='a';
  lowerLine[15]='l';
  writeCommandLCD(0xc0);
  for(counter = 0; counter < 16; counter ++){
    writeDataLCD(lowerLine[counter]);
  }
  controlVariable1 = 30;
  controlVariable2 = 30;
  SREG |= 0x80;
  near();
  timerSys = 0;
  far();
  timeToMove1 = timerSys*1/2;
  while(error >= 4 || error <= -4){
    writeCommandLCD(0x80+15);
    writeDataLCD(0x30+(error%10));
  }
  near();
  speedUp = 1;
  int codeEffect = 0;
  unsigned autoMode = 1;
  if((PIND&(1<<7)) == 0){
    autoMode = 0;
  }
  if(autoMode == 1){
    writeCommandLCD(0x80);
    uperLine[0]='M'; uperLine[1]='o'; uperLine[2]='d';
    uperLine[3]='e'; uperLine[4]=' '; uperLine[5]='A';
    uperLine[6]='u'; uperLine[7]='t'; uperLine[8]='o';
    uperLine[9]='m'; uperLine[10]='a'; uperLine[11]='t';
    uperLine[12]='i'; uperLine[13]='c'; uperLine[14]=' ';
    uperLine[15]=' ';
    for(counter = 0; counter < 16; counter ++){
      writeDataLCD(uperLine[counter]);
    }
    writeCommandLCD(0xc0);
    lowerLine[0]='w'; lowerLine[1]='a'; lowerLine[2]='s';
    lowerLine[3]=' '; lowerLine[4]='e'; lowerLine[5]='n';
    lowerLine[6]='t'; lowerLine[7]='a'; lowerLine[8]='b';
    lowerLine[9]='l'; lowerLine[10]='i'; lowerLine[11]='s';
    lowerLine[12]='h'; lowerLine[13]='e'; lowerLine[14]='d';
    lowerLine[15]=' ';
    for(counter = 0; counter < 16; counter ++){
      writeDataLCD(lowerLine[counter]);
    }
  }else{
    writeCommandLCD(0x80);
    uperLine[0]='M'; uperLine[1]='o'; uperLine[2]='d';
    uperLine[3]='e'; uperLine[4]=' '; uperLine[5]='M';
    uperLine[6]='a'; uperLine[7]='n'; uperLine[8]='u';
    uperLine[9]='a'; uperLine[10]='l'; uperLine[11]=' ';
    uperLine[12]='c'; uperLine[13]='t'; uperLine[14]='r';
    uperLine[15]=' ';
    for(counter = 0; counter < 16; counter ++){
      writeDataLCD(uperLine[counter]);
    }
    writeCommandLCD(0xc0);
    lowerLine[0]='w'; lowerLine[1]='a'; lowerLine[2]='s';
    lowerLine[3]=' '; lowerLine[4]='e'; lowerLine[5]='n';
    lowerLine[6]='t'; lowerLine[7]='a'; lowerLine[8]='b';
    lowerLine[9]='l'; lowerLine[10]='i'; lowerLine[11]='s';
    lowerLine[12]='h'; lowerLine[13]='e'; lowerLine[14]='d';
    lowerLine[15]=' ';
    for(counter = 0; counter < 16; counter ++){
      writeDataLCD(lowerLine[counter]);
    }
  }
  _delay_ms(2000);
  while(1){
    if(autoMode){
      codeEffect += 1;
      if(codeEffect >= 4){
        codeEffect = 0;
      }
      if(errorSys == 1){
        writeCommandLCD(0x80);
        while(1){
          writeDataLCD("E");
          writeDataLCD("!");
        }
      }
      uperLine[0]='A'; uperLine[1]='U'; uperLine[2]='T';
      uperLine[3]='O'; uperLine[4]=' '; uperLine[5]='E';
      uperLine[6]='f'; uperLine[7]='f'; uperLine[8]='e';
      uperLine[9]='c'; uperLine[10]='t'; uperLine[11]=':';
      uperLine[12]=' '; uperLine[13]=' '; uperLine[14]=' ';
      uperLine[15]=' ';
      uperLine[13] = 0x30 + codeEffect;
      writeCommandLCD(0x80);
      for(counter = 0; counter < 16; counter ++){
        writeDataLCD(uperLine[counter]);
      }
      if((codeEffect%2)==0){
        lowerLine[0]='B'; lowerLine[1]='a'; lowerLine[2]='c';
        lowerLine[3]='h'; lowerLine[4]=' '; lowerLine[5]='K';
        lowerLine[6]='h'; lowerLine[7]='o'; lowerLine[8]='a';
        lowerLine[9]=' '; lowerLine[10]='H'; lowerLine[11]='C';
        lowerLine[12]='M'; lowerLine[13]=' '; lowerLine[14]=' ';
        lowerLine[15]=' ';
      }else{
        lowerLine[0]='K'; lowerLine[1]='h'; lowerLine[2]='.';
        lowerLine[3]=' '; lowerLine[4]='D'; lowerLine[5]='i';
        lowerLine[6]='e'; lowerLine[7]='n'; lowerLine[8]='-';
        lowerLine[9]='D'; lowerLine[10]='i'; lowerLine[11]='e';
        lowerLine[12]='n'; lowerLine[13]='T'; lowerLine[14]='u';
        lowerLine[15]=' ';
      }
      writeCommandLCD(0xc0);
      for(counter = 0; counter < 16; counter ++){
        writeDataLCD(lowerLine[counter]);
      }
    }else{
      int effectManual = 0;
      writeCommandLCD(0x01);
      while(1){
        effectManual = 0;
        while((PIND&(1<<7)) == 0){
          writeCommandLCD(0x80);
          writeDataLCD(0x30+effectManual);
          _delay_ms(500);
          effectManual ++;
        }
        if(effectManual > 0){
          codeEffect = effectManual - 1;
          break;
        }
      }
    }
    if(codeEffect == 0){
      if(doneNear == 1){
        far();
      }else{
        near();
      }
    }else if(codeEffect == 1){
      if(doneNear == 0){
        near();
      }
      for(counter = 0; counter < 4; counter ++){
        halfFar(timeToMove1);
      }
    }else if(codeEffect == 2){
      if(doneNear == 0){
        near();
      }
      invert = 1;
      for(counter = 0; counter < 4; counter ++){
        if(invert == 0){
          invert = 1;
        }
        halfFar(timeToMove1);
        _delay_ms(1500);
      }
    }else if(codeEffect == 3){
      if(doneFar == 0){
        far();
      }
      slowDown = 1;
      _delay_ms(1000);
      coreControl = 0;
      controlVariable1 = 40;
      controlVariable2 = -40;
      _delay_ms(2000);
      controlVariable1 = -40;
      controlVariable2 = 40;
      _delay_ms(1500);
      controlVariable1 = 30;
      controlVariable2 = 30;
      speedUp = 1;
      speedDone = 0;
      coreControl = 1;
      while(error >= 4 || error <= -4){
        writeCommandLCD(0x80+15);
        if(error < 0){
          writeDataLCD(0x30+(-error%10));
        }else{
          writeDataLCD(0x30+(error%10));
        }
      }
      writeCommandLCD(0x80+15);
      writeDataLCD(0x30);
      near();
      for(counter = 0; counter < 4; counter ++){
        halfFar(timeToMove1);
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
  DDRD = 0x43;  //all pin is input, 2 pin lowest output, PD6 output
  PORTD = 0xff; // pull up 
  DDRC = 0xff;  
  //==========
  writeCommandLCD(0x02);
  writeCommandLCD(0x28);
  writeCommandLCD(0x0c);
}

void near(){
  doneFar = 0;
  doneNear = 0;
  PORTD &= 0b10111100;
  PORTD |= 0b00000011;
  _delay_ms(500);
  while(1){
    if(((PINB>>4)&0x01)==0){
      PORTD &= 0b11111110;
      //break;
    }
    if(((PINB>>5)&0x01)==0){
      PORTD &= 0b11111101;
      //break;
    }
    if((PORTD & 0x03) == 0x00) break;
  }
  doneNear = 1;
  doneFar = 0;
}

void far(){
  doneFar = 0;
  doneNear = 0;
  PORTD |= 0b01000011;
  PORTD &= 0b11111100;
  _delay_ms(500);
  while(1){
    if(((PINB>>4)&0x01)==0){
      PORTD |= 0b00000001;
      //break;
    }
    if(((PINB>>5)&0x01)==0){
      PORTD |= 0b00000010;
      //break;
    }
    if((PORTD & 0x03) == 0x03) break;
  }
  doneFar = 1;
}

void halfFar(unsigned long time){
  int delay = 0;
  PORTD |= 0b01000011;
  PORTD &= 0b11111100;
  for(delay=0; delay < time; delay++){
    _delay_ms(10);
  }
  PORTD &= 0b10111100;
  PORTD |= 0b00000011;
  while(1){
    if(((PINB>>4)&0x01)==0){
      PORTD &= 0b11111110;
      //break;
    }
    if(((PINB>>5)&0x01)==0){
      PORTD &= 0b11111101;
      //break;
    }
    if((PORTD & 0x03) == 0x00) break;
  }
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
