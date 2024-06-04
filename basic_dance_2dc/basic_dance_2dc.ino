//use pid to control both the motors
#define enca_1 2 //yellow
#define encb_1 4 //white

#define enca_2 3 //yellow
#define encb_2 5 //white

#define pwm_pin_1 10
#define in2 8   //here in2 is in 9 and in1 is in 8 for same direction
#define in1 9

#define pwm_pin_2 11
#define in4 6
#define in3 7

int pos1 = 0;
int pos2 = 0;

unsigned long prevT1 = 0; 
float eprev1 = 0;
float eintegral1 = 0;

unsigned long prevT2 = 0; 
float eprev2 = 0;
float eintegral2 = 0;

float d2p_factor1 = 1.124;
float d2p_factor2 = 1.124;

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
  // define the target positions
  int target_degrees[] = {90,  180, 270, 360, 720, 810, 900, 990, 1080, 1440};
  int steps[] =          {100, 100, 100, 100, 100, 100, 100, 100, 100 , 100 };

  // PID constants
  float kp1 = 2.05;
  float kd1 = 0.055;
  float ki1 = 0.0001;

  float kp2 = 1.47;
  float kd2 = 0.065;
  float ki2 = 0.0001;

  // iterate through target positions
  for (int i = 0; i < sizeof(target_degrees) / sizeof(target_degrees[0]); i++) {
    // set the target position
    int target_degree = target_degrees[i];

    int target1 = int(d2p_factor1 * target_degree);
    int target2 = int(d2p_factor2 * target_degree);

    // move towards the target position in steps
    for (int step = 0; step < steps[i]; step++) {
      // time difference
      unsigned long currT2 = micros();
      unsigned long et2 = currT2 - prevT2;
      float deltaT2 = ((float)(currT2 - prevT2)) / 1.0e6;
      prevT2 = currT2;

      unsigned long currT1 = micros();
      unsigned long et1 = currT1 - prevT1;
      float deltaT1 = ((float)(currT1 - prevT1)) / 1.0e6;
      prevT1 = currT1;

      // error
      int e2 = pos2 - target2;
      int e1 = pos1 - target1;

      // derivative
      float dedt2 = (e2 - eprev2) / deltaT2;
      float dedt1 = (e1 - eprev1) / deltaT1;

      // integral
      eintegral2 = eintegral2 + e2 * deltaT2;
      eintegral1 = eintegral1 + e1 * deltaT1;

      // control signal
      float u2 = kp2 * e2 + kd2 * dedt2 + ki2 * eintegral2;  
      float u1 = kp1 * e1 + kd1 * dedt1 + ki1 * eintegral1; 

      // motor power
      float pwr2 = fabs(u2);
      if (pwr2 > 230) {
        pwr2 = 230;
      }
      float pwr1 = fabs(u1);
      if (pwr1 > 230) {
        pwr1 = 230;
      }
          
      // if (pwr1 < 50){
      //   pwr1 = 50;
      // }
      // if (pwr2 < 50){
      //   pwr2 = 50;
      // }
      // motor direction
      int dir2 = 1;
      if (u2 < 0) {
        dir2 = -1;
      } 
      int dir1 = 1;
      if (u1 < 0) {
        dir1 = -1;
      } 

      // signal the motor
      setMotor(dir2, (uint8_t)pwr2, pwm_pin_2, in3, in4);
      setMotor(dir1, (uint8_t)pwr1, pwm_pin_1, in1, in2);

      // store previous error
      eprev2 = e2;
      eprev1 = e1;

      Serial.print(target2);
      Serial.print(" ");
      Serial.print(pos2);
      Serial.println();
    }
  }
}


void setMotor(int dir, uint8_t pwmVal, int pwm, int in1_pin, int in2_pin) {
  analogWrite(pwm, pwmVal);
  if (dir == 1) {
    digitalWrite(in1_pin, HIGH);
    digitalWrite(in2_pin, LOW);
  } else if (dir == -1) {
    digitalWrite(in1_pin, LOW);
    digitalWrite(in2_pin, HIGH);
  } else {
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
    pos2--;
  }
  else {
    pos2++;
  }
}

