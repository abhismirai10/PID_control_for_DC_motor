//to test the position encoder for both motors
#define enca_1 2 //yellow
#define encb_1 4 //white

#define enca_2 3 //yellow
#define encb_2 5 //white

int pos1 = 0;
int pos2 = 0;

void setup() {
  Serial.begin(9600);
  pinMode(enca_1, INPUT);
  pinMode(encb_1, INPUT);
  attachInterrupt(digitalPinToInterrupt(enca_1), readEncoder_1,RISING);

  pinMode(enca_2, INPUT);
  pinMode(encb_2, INPUT);
  attachInterrupt(digitalPinToInterrupt(enca_2), readEncoder_2,RISING);
}

void loop() {
  Serial.print(pos1);
  Serial.print(" ");
  Serial.println(pos2);

}

void readEncoder_1(){
  int b1 = digitalRead(encb_1);

  if (b1>0){
    pos1++;
  }
  else {
    pos1--;
  }
}

void readEncoder_2(){
  int b2 = digitalRead(encb_2);

  if (b2>0){
    pos2++;
  }
  else {
    pos2--;
  }
}