#define enca 2 //yellow
#define encb 3 //white

int pos = 0;

void setup() {
  Serial.begin(9600);
  pinMode(enca, INPUT);
  pinMode(encb, INPUT);
  attachInterrupt(digitalPinToInterrupt(enca), readEncoder,RISING);
}

void loop() {
  Serial.println(pos);

}

void readEncoder(){
  int b = digitalRead(encb);

  if (b>0){
    pos++;
  }
  else {
    pos--;
  }
}