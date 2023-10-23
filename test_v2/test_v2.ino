#define F_CPU 16000000

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

int controlVariable1;
int controlVariable2;
unsigned int position1 = 0;
unsigned int position2 = 0;
unsigned char doneFar = 1;
unsigned char doneNear = 0;
unsigned char clearToNear = 1;
unsigned char speedUp = 0;
unsigned char slowDown = 0;
int interal;
int oldError;
int error;
int currentSpeed;
unsigned char pause = 0;
unsigned long timerSys = 0;
unsigned char coreControl = 1;
unsigned int timerChange = 0;
int poiter = 0;
char upperLine[16];
char lowerLine[100];
void initSystem();
void writeDataLCD(unsigned char data);
void writeCommandLCD(unsigned char command);
void writeUpperLine(char data[17]);
void writeLowwerLine(char data[17]);
void near();
void far();
ISR(INT0_vect){   //PD2 pin 2
  if((PIND>>4)&0x01){  //PD4 pin4
    position1 ++;
  }else{
    position1 --;
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
  visai = error*3;   //+ (error - oldError)/2 + interal/4;
  if(visai > 20){
    visai = 20;
  }else if(visai < -20){
    visai = -20;
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
  if(timerSys % 2 == 0 && controlVariable1 <= 240 && speedUp == 1){  //speed up function
      controlVariable1 += 2;
      controlVariable2 = controlVariable1;
      if(controlVariable1 >= 240){
        speedUp = 0;
        writeCommandLCD(0x80+15);
        writeDataLCD(' ');
      }
  }
  if(timerSys % 2 == 0 && controlVariable1 >= 5 && slowDown == 1){    //slowdown function
    controlVariable1 -= 2;
    controlVariable2 = controlVariable1;
    if(controlVariable1 <= 5){
      slowDown = 0;
      pause = 1;
      writeCommandLCD(0x80+15);
      writeDataLCD(' ');
    }
  }
}
int main(){
  initSystem();
  pause = 1;
  controlVariable1 = 210;
  controlVariable2 = 210;
  SREG |= 0x80;
  int saisotemp;
  int i = 0;
  int codeInput = 0;
  writeCommandLCD(0x01);
  lowerLine[0]=' '; lowerLine[1]=' '; lowerLine[2]=' ';
  lowerLine[3]=' '; lowerLine[4]='H'; lowerLine[5]='O';
  lowerLine[6]=' '; lowerLine[7]='C'; lowerLine[8]='H';
  lowerLine[9]='I'; lowerLine[10]=' '; lowerLine[11]='M';
  lowerLine[12]='I'; lowerLine[13]='N'; lowerLine[14]='H';
  lowerLine[15]=' '; lowerLine[16]='C'; lowerLine[17]='I';
  lowerLine[18]='T'; lowerLine[19]='Y'; lowerLine[20]=' ';
  lowerLine[21]='U'; lowerLine[22]='N'; lowerLine[23]='I';
  lowerLine[24]='V'; lowerLine[25]='E'; lowerLine[26]='R';
  lowerLine[27]='S'; lowerLine[28]='I'; lowerLine[29]='T';
  lowerLine[30]='Y'; lowerLine[31]=' '; lowerLine[32]='O';
  lowerLine[33]='F'; lowerLine[34]=' '; lowerLine[35]='T';
  lowerLine[36]='E'; lowerLine[37]='C'; lowerLine[38]='H';
  lowerLine[39]='N'; lowerLine[40]='O'; lowerLine[41]='L';
  lowerLine[42]='O'; lowerLine[43]='G'; lowerLine[44]='Y';
  lowerLine[45]='-'; lowerLine[46]='F'; lowerLine[47]='a';
  lowerLine[48]='c'; lowerLine[49]='u'; lowerLine[50]='l';
  lowerLine[51]='t'; lowerLine[52]='y'; lowerLine[53]=' ';
  lowerLine[54]='o'; lowerLine[55]='f'; lowerLine[56]=' ';
  lowerLine[57]='E'; lowerLine[58]='l'; lowerLine[59]='e';
  lowerLine[60]='c'; lowerLine[61]='t'; lowerLine[62]='r';
  lowerLine[63]='i'; lowerLine[64]='c'; lowerLine[65]='a';
  lowerLine[66]='l'; lowerLine[67]=' '; lowerLine[68]='a';
  lowerLine[69]='n'; lowerLine[70]='d'; lowerLine[71]=' ';
  lowerLine[72]='E'; lowerLine[73]='l'; lowerLine[74]='e';
  lowerLine[75]='c'; lowerLine[76]='t'; lowerLine[77]='r';
  lowerLine[78]='o'; lowerLine[79]='n'; lowerLine[80]='i';
  lowerLine[81]='c'; lowerLine[82]='s'; lowerLine[83]=' ';
  lowerLine[84]='E'; lowerLine[85]='n'; lowerLine[86]='g';
  lowerLine[87]='i'; lowerLine[88]='n'; lowerLine[89]='e';
  lowerLine[90]='e'; lowerLine[91]='r'; lowerLine[92]='i';
  lowerLine[93]='n'; lowerLine[94]='g'; lowerLine[95]=' ';
  lowerLine[96]=' '; lowerLine[97]=' '; lowerLine[98]=' ';
  lowerLine[99]=' ';
  while(1){
    writeCommandLCD(0x80);
    upperLine[0]='E'; upperLine[1]='r'; upperLine[2]='r';
    upperLine[3]='o'; upperLine[4]='r'; upperLine[5]=':';
    upperLine[6]=' '; upperLine[7]=' '; upperLine[8]=' ';
    upperLine[9]=' '; upperLine[10]=' '; upperLine[11]=' ';
    upperLine[12]=' '; upperLine[13]=' '; upperLine[14]=' ';
    upperLine[15]=' ';
    for(i=0; i<7; i++){
      writeDataLCD(upperLine[i]);
    }
    writeCommandLCD(0x80+7);
    if(saisotemp >= 0){
      writeDataLCD(' ');
    }else{
      writeDataLCD('-');
      saisotemp = - saisotemp;
    }
    if(saisotemp <= 2){
      saisotemp = 0;
    }
    writeDataLCD((saisotemp/100)%10 + 0x30);
    writeDataLCD((saisotemp/10)%10 + 0x30);
    writeDataLCD((saisotemp/1)%10 + 0x30);
    writeCommandLCD(0xc0);
    for(i=0; i<16; i++){
      writeDataLCD(lowerLine[poiter+i]);
    }
    if((PIND&(1<<7)) == 0){
      coreControl = 1;
      codeInput = 0;
      while((PIND&(1<<7)) == 0){
        writeCommandLCD(0x80 + 14);
        writeDataLCD(0x30+codeInput);
        _delay_ms(500);
        codeInput ++;
      }
      if(codeInput == 1){    //speed up 
        controlVariable1 = 10;
        controlVariable2 = 10;
        _delay_ms(20);
        pause = 0;
        speedUp = 1;
        writeCommandLCD(0x80+15);
        writeDataLCD('!');
      }else if(codeInput == 2){   //slow 
        far();/////
        doneFar = 1;
        doneNear = 0;
        slowDown = 1;
        writeCommandLCD(0x80+15);
        writeDataLCD('!');
      }else if(codeInput == 3){
        if(doneFar == 0){
          far();
          timerSys = 0;
          doneFar = 1;
          doneNear = 0;
        }
        coreControl = 0;
        if(coreControl == 0){
          writeCommandLCD(0x80+15);
          writeDataLCD('!');
          while(controlVariable1 > 5){
            controlVariable1 -= 5;
            controlVariable2 -= 5;
            _delay_ms(100);
          }
          while(1){
            saisotemp = position1 - position2;
            writeCommandLCD(0x80+7);
            if(saisotemp >= 0){
              writeDataLCD(' ');
            }else{
              writeDataLCD('-');
              saisotemp = - saisotemp;
            }
            if(saisotemp <= 2){
              saisotemp = 0;
            }
            writeDataLCD((saisotemp/100)%10 + 0x30);
            writeDataLCD((saisotemp/10)%10 + 0x30);
            writeDataLCD((saisotemp/1)%10 + 0x30);
            if((PIND&(1<<7)) == 0) break;
            controlVariable1 = 0;
            controlVariable2 = 30;
          }
          coreControl = 1;
          controlVariable1 = 10;
          controlVariable2 = 10;
          speedUp = 1;
        }
      }
    }
    saisotemp = position1 - position2;
    if(saisotemp >= -4 && saisotemp <= 4){
      clearToNear = 1;
    }else{
      clearToNear = 0;
    }
    if(timerSys > 50 && doneFar == 1 && clearToNear == 1 && pause == 0){
      near();
      doneNear = 1;
      doneFar = 0;
      timerSys = 0;
    }
    if(timerSys > 50 && doneNear == 1){
      far();
      doneNear = 0;
      doneFar = 1;
      timerSys = 0;
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
  /*
  PORTD |= 0b00000010;
  _delay_ms(500);
  while(1){
    if(((PINB>>5)&0x01)==0){
      PORTD &= 0b11111101;
      break;
    }
  }*/
}

void far(){
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
  /*PORTD &= 0b11111101;
  _delay_ms(500);
  while(1){
    if(((PINB>>5)&0x01)==0){
      PORTD |= 0b00000010;
      break;
    }
  }*/
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
