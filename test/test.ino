int position = 0;
void updatePosition(){
  if(digitalRead(4)==1){
    position ++;
  }else{
    position --;
  }
}

void setup(){
  attachInterrupt(0,updatePosition,RISING);
  Serial.begin(9600);
}

void loop(){
   Serial.println(position);
}
