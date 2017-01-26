#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define BASEMIN 200
#define BASEMAX 857
#define BASESPEED 0
#define SHOULDERMIN 300
#define SHOULDERMAX 700
#define SHOULDERSPEED 0
#define ELBOWMIN 300
#define ELBOWMAX 700
#define ELBOWSPEED 0
#define CLAWMIN 40
#define CLAWMAX 900
#define CLAWSPEED 0

#define MOUSE_DATA 5
#define MOUSE_CLK 6

String cmd;
int goal;
int diff;
int potVal = 0;
int baseCurr = (BASEMAX - BASEMIN) / 2 + BASEMIN;
int shoulderCurr = (SHOULDERMAX - SHOULDERMIN) / 2 + SHOULDERMIN;
int elbowCurr = (ELBOWMAX - ELBOWMIN) / 2 + ELBOWMIN;
int clawCurr = (CLAWMAX - CLAWMIN) / 2 + CLAWMIN;
int baseLastRead = 0;
int shoulderLastRead = 0;
int elbowLastRead = 0;
int clawLastRead = 0;
int baseGoal = baseCurr;
int shoulderGoal = shoulderCurr;
int elbowGoal = elbowCurr;
int clawGoal = clawCurr;

void setup() {
  Serial.begin(115200);
  
  pwm.begin();
  pwm.setPWMFreq(90);
  pwm.setPWM(0, 0, baseCurr);
  pwm.setPWM(1, 0, shoulderCurr);
  pwm.setPWM(2, 0, elbowCurr);
  pwm.setPWM(3, 0, clawCurr);
  mouse_init(); 
  Serial.println("Ready");
}

void loop() {
  //fromPots();
  fromMouse();
}

void fromMouse() {
  char mstat;
  char mx;
  char my;
  char mz;

  mouse_write(0xeb);  
  mouse_read();
  mstat = mouse_read();
  mx = mouse_read();
  my = mouse_read();
  mz = mouse_read();

  Serial.print(moveClaw(processClick(mstat)), BIN);
  Serial.print("\tX=");
  Serial.print(moveBase(mx), DEC);
  Serial.print("\tY=");
  Serial.print(moveShoulder(my), DEC);
  Serial.print("\tZ=");
  Serial.print(moveElbow(mz*30), DEC);
  Serial.println();
  delay(20); 
}

void fromPots() {
  //fromBasePot();
  //fromShoulderPot();
  //fromElbowPot();
  //fromClawPot();
  //Serial.println("");
}

int processClick(int mstat) {
  int leftClick = mstat % 2 == 1;
  return leftClick;
}

int moveBase(int amt) {
  //potVal = analogRead(0);
  //Serial.print(map(baseCurr, BASEMIN, BASEMAX, 0, 180));
  //Serial.print("\t");
  baseGoal += amt;
  baseGoal = constrain(baseGoal, 0, 1024);
  goal = map(baseGoal, 0, 1024, BASEMIN, BASEMAX);  
  if (baseGoal < baseLastRead + 3 && baseGoal > baseLastRead - 3) return;
  baseLastRead = baseGoal;   
  if (baseCurr > goal) {
    diff = baseCurr - goal;
    for (uint16_t pulselen = baseCurr; pulselen > goal; pulselen--) {
      pwm.setPWM(0, 0, pulselen);
      delay(BASESPEED);
    }
  } else {      
    diff = goal - baseCurr;      
    for (uint16_t pulselen = baseCurr; pulselen < goal; pulselen++) {
      pwm.setPWM(0, 0, pulselen);
      delay(BASESPEED);
    }          
  }
  baseCurr = goal;
  //delay(constrain(abs(amt),0,100));
  return goal;
}

int moveShoulder(int amt) {
  //potVal = analogRead(1);
  //Serial.print(map(shoulderCurr, SHOULDERMIN, SHOULDERMAX, 0, 180));
  //Serial.print("\t");
  shoulderGoal += amt;
  shoulderGoal = constrain(shoulderGoal, 0, 1024);
  if (shoulderGoal < shoulderLastRead + 3 && shoulderGoal > shoulderLastRead - 3) return;
  goal = map(shoulderGoal, 0, 1024, SHOULDERMIN, SHOULDERMAX);        
  shoulderLastRead = shoulderGoal;
  if (shoulderCurr > goal) {
    diff = shoulderCurr - goal;
    for (uint16_t pulselen = shoulderCurr; pulselen > goal; pulselen--) {
      pwm.setPWM(1, 0, pulselen);
      delay(SHOULDERSPEED);
    }
  } else {
    diff = goal - shoulderCurr;
    for (uint16_t pulselen = shoulderCurr; pulselen < goal; pulselen++) {
      pwm.setPWM(1, 0, pulselen);
      delay(SHOULDERSPEED);
    }          
  }
  shoulderCurr = goal;
  //delay(constrain(abs(amt),0,100));
  return goal;
}

int moveElbow(int amt) {
  //potVal = analogRead(2);
  //Serial.print(map(elbowCurr, ELBOWMIN, ELBOWMAX, 180, 0));
  //Serial.print("\t");
  elbowGoal += amt;
  elbowGoal = constrain(elbowGoal, 0, 1024);
  if (elbowGoal < elbowLastRead + 3 && elbowGoal > elbowLastRead - 3) return;
  goal = map(elbowGoal, 1024, 0, ELBOWMIN, ELBOWMAX);        
  elbowLastRead = elbowGoal;
  if (elbowCurr > goal) {
    diff = elbowCurr - goal;
    for (uint16_t pulselen = elbowCurr; pulselen > goal; pulselen--) {
      pwm.setPWM(2, 0, pulselen);
      delay(ELBOWSPEED);
    }
  } else {
    diff = goal - elbowCurr;
    for (uint16_t pulselen = elbowCurr; pulselen < goal; pulselen++) {
      pwm.setPWM(2, 0, pulselen);
      delay(ELBOWSPEED);
    }          
  }
  elbowCurr = goal;
  //delay(constrain(abs(amt),0,100));
  return goal;
}

int moveClaw(int amt) {
  //potVal = analogRead(3);
  //Serial.print(map(clawCurr, CLAWMIN, CLAWMAX, 0, 180));
  //Serial.print("\t");
  //clawGoal += amt;
  //clawGoal = constrain(clawGoal, 0, 1024);
  //if (clawGoal < clawLastRead + 3 && clawGoal > clawLastRead - 3) return;
  goal = map(amt, 1, 0, CLAWMIN, CLAWMAX);        
  clawLastRead = amt;
  if (clawCurr > goal) {
    diff = clawCurr - goal;
    for (uint16_t pulselen = clawCurr; pulselen > goal; pulselen--) {
      pwm.setPWM(3, 0, pulselen);
      delay(CLAWSPEED);
    }
  } else {
    diff = goal - clawCurr;
    for (uint16_t pulselen = clawCurr; pulselen < goal; pulselen++) {
      pwm.setPWM(3, 0, pulselen);
      delay(CLAWSPEED);
    }          
  }
  clawCurr = goal;
  //delay(50);
  return amt;
}

void fromSerial() {
  if (Serial.available()){
    delay(10);
    while(Serial.available()>0){      
      cmd = Serial.readStringUntil(' ');
      if (cmd == "b") {
        goal = map(Serial.readStringUntil(' ').toInt(), 0, 180, BASEMIN, BASEMAX);   
        Serial.print("BASE: ");
        Serial.print(baseCurr);
        if (baseCurr > goal) {
          diff = baseCurr - goal;
          Serial.print(" - ");
          Serial.print(diff);
          for (uint16_t pulselen = baseCurr; pulselen > goal; pulselen--) {
            pwm.setPWM(0, 0, pulselen);
            delay(BASESPEED);
          }
        } else {      
          diff = goal - baseCurr;      
          Serial.print(" + ");
          Serial.print(diff);
          for (uint16_t pulselen = baseCurr; pulselen < goal; pulselen++) {
            pwm.setPWM(0, 0, pulselen);
            delay(BASESPEED);
          }          
        }
        Serial.print(" = ");
        Serial.println(goal);
        baseCurr = goal;
        delay(diff / 2);
      } else if (cmd == "s") {
        Serial.print("SHOULDER: ");
        Serial.print(shoulderCurr);
        goal = map(Serial.readStringUntil(' ').toInt(), 0, 180, SHOULDERMIN, SHOULDERMAX);        
        if (shoulderCurr > goal) {
          diff = shoulderCurr - goal;
          Serial.print(" - ");
          Serial.print(diff);
          for (uint16_t pulselen = shoulderCurr; pulselen > goal; pulselen--) {
            pwm.setPWM(1, 0, pulselen);
            delay(SHOULDERSPEED);
          }
        } else {
          diff = goal - shoulderCurr;
          Serial.print(" + ");
          Serial.print(diff);
          for (uint16_t pulselen = shoulderCurr; pulselen < goal; pulselen++) {
            pwm.setPWM(1, 0, pulselen);
            delay(SHOULDERSPEED);
          }          
        }
        Serial.print(" = ");
        Serial.println(goal);
        shoulderCurr = goal;
        delay(diff / 2);
      } else if (cmd == "e") {
        Serial.print("ELBOW: ");
        Serial.print(elbowCurr);
        goal = map(Serial.readStringUntil(' ').toInt(), 180, 0, ELBOWMIN, ELBOWMAX);        
        if (elbowCurr > goal) {
          diff = elbowCurr - goal;
          Serial.print(" - ");
          Serial.print(diff);
          for (uint16_t pulselen = elbowCurr; pulselen > goal; pulselen--) {
            pwm.setPWM(2, 0, pulselen);
            delay(ELBOWSPEED);
          }
        } else {
          diff = goal - elbowCurr;
          Serial.print(" + ");
          Serial.print(diff);
          for (uint16_t pulselen = elbowCurr; pulselen < goal; pulselen++) {
            pwm.setPWM(2, 0, pulselen);
            delay(ELBOWSPEED);
          }          
        }
        Serial.print(" = ");
        Serial.println(goal);
        elbowCurr = goal;
        delay(diff / 2);
      } else if (cmd == "c") {
        Serial.print("CLAW: ");
        Serial.print(clawCurr);
        goal = map(Serial.readStringUntil(' ').toInt(), 0, 180, CLAWMIN, CLAWMAX);        
        if (clawCurr > goal) {
          diff = clawCurr - goal;
          Serial.print(" - ");
          Serial.print(diff);
          for (uint16_t pulselen = clawCurr; pulselen > goal; pulselen--) {
            pwm.setPWM(3, 0, pulselen);
            delay(CLAWSPEED);
          }
        } else {
          diff = goal - clawCurr;
          Serial.print(" + ");
          Serial.print(diff);
          for (uint16_t pulselen = clawCurr; pulselen < goal; pulselen++) {
            pwm.setPWM(3, 0, pulselen);
            delay(CLAWSPEED);
          }          
        }
        Serial.print(" = ");
        Serial.println(goal);
        clawCurr = goal;
        delay(diff / 2);
      }
    }
    Serial.println("Ready");
  }
}

void gohi(int pin) {
  pinMode(pin, INPUT);
  digitalWrite(pin, HIGH);
}

void golo(int pin) {
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
}

void mouse_write(char data) {
  char i;
  char parity = 1;

  gohi(MOUSE_DATA);
  gohi(MOUSE_CLK);
  delayMicroseconds(300);
  golo(MOUSE_CLK);
  delayMicroseconds(300);
  golo(MOUSE_DATA);
  delayMicroseconds(10);
  gohi(MOUSE_CLK);
  while (digitalRead(MOUSE_CLK) == HIGH)
    ;
  for (i=0; i < 8; i++) {
    if (data & 0x01) {
      gohi(MOUSE_DATA);
    } 
    else {
      golo(MOUSE_DATA);
    }
    while (digitalRead(MOUSE_CLK) == LOW)
      ;
    while (digitalRead(MOUSE_CLK) == HIGH)
      ;
    parity = parity ^ (data & 0x01);
    data = data >> 1;
  }  
  if (parity) {
    gohi(MOUSE_DATA);
  } 
  else {
    golo(MOUSE_DATA);
  }
  while (digitalRead(MOUSE_CLK) == LOW)
    ;
  while (digitalRead(MOUSE_CLK) == HIGH)
    ;
  gohi(MOUSE_DATA);
  delayMicroseconds(50);
  while (digitalRead(MOUSE_CLK) == HIGH)
    ;
  while ((digitalRead(MOUSE_CLK) == LOW) || (digitalRead(MOUSE_DATA) == LOW))
    ;
  golo(MOUSE_CLK);
}

char mouse_read(void) {
  char data = 0x00;
  int i;
  char bit = 0x01;

  gohi(MOUSE_CLK);
  gohi(MOUSE_DATA);
  delayMicroseconds(50);
  while (digitalRead(MOUSE_CLK) == HIGH)
    ;
  delayMicroseconds(5); 
  while (digitalRead(MOUSE_CLK) == LOW)
    ;
  for (i=0; i < 8; i++) {
    while (digitalRead(MOUSE_CLK) == HIGH)
      ;
    if (digitalRead(MOUSE_DATA) == HIGH) {
      data = data | bit;
    }
    while (digitalRead(MOUSE_CLK) == LOW)
      ;
    bit = bit << 1;
  }
  while (digitalRead(MOUSE_CLK) == HIGH)
    ;
  while (digitalRead(MOUSE_CLK) == LOW)
    ;
  while (digitalRead(MOUSE_CLK) == HIGH)
    ;
  while (digitalRead(MOUSE_CLK) == LOW)
    ;
  golo(MOUSE_CLK);
  return data;
}

void mouse_init() { 
  char mouseId;
   
  gohi(MOUSE_CLK);
  gohi(MOUSE_DATA);
  mouse_write(0xff);
  mouse_read();
  mouse_read();
  mouse_read();
  mouse_write(0xf3);
  mouse_read();
  mouse_write(0xC8);
  mouse_read();
  mouse_write(0xf3);
  mouse_read();
  mouse_write(0x64);
  mouse_read();
  mouse_write(0xf3);
  mouse_read();
  mouse_write(0x50);
  mouse_read();
  mouse_write(0xf2);
  mouse_read();
  mouse_read();
  mouse_write(0xe8);
  mouse_read();
  mouse_write(0x03); 
  mouse_read();
  mouse_write(0xe6); 
  mouse_read();
  mouse_write(0xf3); 
  mouse_read();
  mouse_write(0x28); 
  mouse_read();
  mouse_write(0xf4);
  mouse_read();
  mouse_write(0xf0); 
  mouse_read();
  delayMicroseconds(100);
}
