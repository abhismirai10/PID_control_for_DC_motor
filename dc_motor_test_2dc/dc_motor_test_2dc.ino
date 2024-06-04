//to control dc motor using driver for both motors
#define enca_1 2 //yellow
#define encb_1 4 //white

#define enca_2 3 //yellow
#define encb_2 5 //white

#define pwm_pin_1 11
#define in2 6
#define in1 7

#define pwm_pin_2 10
#define in4 8
#define in3 9

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

  pinMode(in2, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(pwm_pin_1, OUTPUT);

  pinMode(in4, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(pwm_pin_2, OUTPUT);
}

void loop() {
  setMotor(1, 90, pwm_pin_1, in1, in2);
  setMotor(1, 90, pwm_pin_2, in3, in4);
  delay(1000);
  Serial.println(pos1);
  Serial.println(pos2);

  setMotor(-1, 90, pwm_pin_1, in1, in2);
  setMotor(-1, 90, pwm_pin_2, in3, in4);
  delay(1000);
  Serial.println(pos1);
  Serial.println(pos2);

}
void setMotor(int dir, int pwmVal, int pwm, int in1_pin, int in2_pin){
  analogWrite(pwm, pwmVal);
  if(dir == 1){
    digitalWrite(in1_pin, HIGH);
    digitalWrite(in2_pin, LOW);
  }
  else if(dir == -1){
    digitalWrite(in1_pin, LOW);
    digitalWrite(in2_pin, HIGH);
  }
  else{
    digitalWrite(in1_pin, LOW);
    digitalWrite(in2_pin, LOW);
  }
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

