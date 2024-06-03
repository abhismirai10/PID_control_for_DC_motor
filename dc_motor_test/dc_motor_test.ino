//to control dc motor using driver
#define enca 2 //yellow
#define encb 3 //white

#define pwm_pin 11
#define in2 6
#define in1 7

int pos = 0;

void setup() {
  Serial.begin(9600);
  pinMode(enca, INPUT);
  pinMode(encb, INPUT);
  attachInterrupt(digitalPinToInterrupt(enca), readEncoder,RISING);

  pinMode(in2, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(pwm_pin, OUTPUT);
}

void loop() {
  setMotor(1, 90, pwm_pin, in1, in2);
  delay(1000);
  Serial.println(pos);

  setMotor(-1, 90, pwm_pin, in1, in2);
  delay(1000);
  Serial.println(pos);

  setMotor(0, 0, pwm_pin, in1, in2);
  delay(1000);
  Serial.println(pos);

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

void readEncoder(){
  int b = digitalRead(encb);

  if (b>0){
    pos++;
  }
  else {
    pos--;
  }
}