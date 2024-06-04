//to control dc motor using driver
#define enca 2 //yellow
#define encb 3 //white

#define pwm_pin 11
#define in2 6
#define in1 7

int pos = 0;

unsigned long prevT = 0; // Change to unsigned long to handle micros() overflow
float eprev = 0;
float eintegral = 0;

void setup() {
  Serial.begin(9600);
  pinMode(enca, INPUT);
  pinMode(encb, INPUT);
  attachInterrupt(digitalPinToInterrupt(enca), readEncoder, RISING);

  pinMode(in2, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(pwm_pin, OUTPUT);
}

void loop() {
  // define the target positions
  int target_degrees[] = {90,  0,  -90,   0, -270, 0,   270,   0, 90, -90};
  int steps[] =          {100, 100, 100, 100, 100, 100, 100, 100, 100, 100 };

  // PID constants
  float kp = 1;
  float kd = 0.055;
  float ki = 0.0001;

  // iterate through target positions
  for (int i = 0; i < sizeof(target_degrees) / sizeof(target_degrees[0]); i++) {
    // set the target position
    int target_degree = target_degrees[i];
    int target = int(1.124 * target_degree);

    // move towards the target position in steps
    for (int step = 0; step < steps[i]; step++) {
      // time difference
      unsigned long currT = micros(); // Change to unsigned long
      float deltaT = ((float)(currT - prevT)) / 1.0e6;
      prevT = currT;

      // error
      int e = target - pos;

      // derivative
      float dedt = (e - eprev) / deltaT;

      // integral
      eintegral = eintegral + e * deltaT;

      // control signal
      float u = kp * e + kd * dedt + ki * eintegral;

      // motor power
      float pwr = fabs(u);
      if (pwr > 230) {
        pwr = 230;
      }

      if (pwr < 50){
        pwr = 50;
      }

      // motor direction
      int dir = 1;
      if (u < 0) {
        dir = -1;
      }

      // signal the motor
      setMotor(dir, (uint8_t)pwr, pwm_pin, in1, in2); // Cast pwmVal to uint8_t

      // store previous error
      eprev = e;

      Serial.print(target);
      Serial.print(" ");
      Serial.print(pos);
      Serial.println();
    }
  }
}



void setMotor(int dir, uint8_t pwmVal, int pwm, int in1_pin, int in2_pin) { // Change pwmVal data type to uint8_t
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

void readEncoder() {
  int b = digitalRead(encb);

  if (b > 0) {
    pos++;
  } else {
    pos--;
  }
}