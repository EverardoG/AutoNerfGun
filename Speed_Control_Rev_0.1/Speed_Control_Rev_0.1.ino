// motor set up
#include <Wire.h>
#include <Adafruit_MotorShield.h>
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *m3 = AFMS.getMotor(3);
Adafruit_DCMotor *m4 = AFMS.getMotor(4);

// encoder set up
#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>
Encoder e3(2, 4); // set encoder pins
Encoder e4(3, 5);
const int conv3 = 745; // pulses per revolution - avg can reach up to 58.94
const int conv4 = conv3;
long pos3;
long pos4;

// PID set up
#include <PID_v1.h>
double Setvel, Input3, Output3;
double Kp3 = 10, Ki3 = 30, Kd3 = 3; //40,20,2 work well, also 10,30, 3
PID PID3(&Input3, &Output3, &Setvel, Kp3, Ki3, Kd3, DIRECT);

double Input4, Output4;
double Kp4 = 10, Ki4 = 30, Kd4 = 3; //40,20,2 work well, also 10,30, 3
PID PID4(&Input4, &Output4, &Setvel, Kp4, Ki4, Kd4, DIRECT);

// LCD screen set up
#include <LiquidCrystal.h>
LiquidCrystal lcd(12, 11, 8, 7, 9, 10);

// velocity measurement set up
long newPos3; // position of motor 3
long oldPos3; // last position of motor 3
long newPos4; // position of motor 4
long oldPos4; // last position of motor 3
long oldTime = 0; // last time check
long newTime = 0; // current time
long deltaTime = 0; // time passed
double velocity3; // motor speed mtr3
double velocity4; // motor speed mtr4
double vel_rs3; // revolutions per second mtr3
double vel_rs4; // revolutions per second mtr4

// real time control & estop
int newLoopTime = 0;
int oldLoopTime = 0;
int controlLoopInterval = 100; // run at 10 Hz
boolean realTimeRun = true;
int eStopPin = 13;
int inputPin = A0;
int inputVal = 0;

void setup() {
  // put your setup code here, to run once:

  // comms set up
  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("PID Control Test");
  AFMS.begin();  // create with the default frequency 1.6KHz

  // PID set up
  PID3.SetMode(AUTOMATIC);
  PID3.SetOutputLimits(-255, 255);
  PID4.SetMode(AUTOMATIC);
  PID4.SetOutputLimits(-255, 255);

  // estop set up
  pinMode(eStopPin, INPUT_PULLUP);

  //lcd set up
  lcd.begin(16, 2);
  lcd.print("Motor Velocity:");
  lcd.setCursor(0, 1);
}

void loop() {
  //  Serial.print(realTimeRun);

  if (!realTimeRun) {
    while (!realTimeRun) {
      if (!digitalRead(eStopPin)) {
      lcd.setCursor(0, 1);
      lcd.print("RELEASE BUTTON TO RESTART");
        while (!digitalRead(eStopPin)) {
          delay(1);
        }
        realTimeRun = true;
        Serial.println("RESTARTING");
        break;
      }
    }
  }

  while (realTimeRun) {
    // check e stop and stop if hit
    if (!digitalRead(eStopPin)) {
      setVelocity(m3, 0);
      setVelocity(m4, 0);
      lcd.clear();
      lcd.print("ESTOP TRIGGERED");
      lcd.setCursor(0, 1);
      lcd.print("HIT AGAIN TO RESTART");
      while (!digitalRead(eStopPin)) {
        delay(1);
      }
      realTimeRun = false;
      Serial.println("ESTOP TRIGGERED - HIT AGAIN TO RESTART");
      break;
    }

    // Real-Time clock control. Check to see if one clock cycle has elapsed before running this control code
    newLoopTime = millis();                                             // get current Arduino time (50 days till wrap)
    if (newLoopTime - oldLoopTime >= controlLoopInterval) {             // if true run flight code
      oldLoopTime = newLoopTime;                                        // reset time stamp

      // SENSE
      // encoder reading
      pos3 = e3.read();
      pos4 = e4.read();

      // speed input reading
      inputVal = analogRead(inputPin);

      // velocity calculations
      oldTime = newTime;
      newTime = millis();

      oldPos3 = newPos3;
      newPos3 = pos3;
      
      oldPos4 = newPos4;
      newPos4 = pos4;

      velocity3 = ( (double) (newPos3 - oldPos3)) / ( (double) (newTime - oldTime)); // ticks per millisecond
      vel_rs3 = (velocity3 / conv3) * 1000;

      velocity4 = ( (double) (newPos4 - oldPos4)) / ( (double) (newTime - oldTime)); // ticks per millisecond
      vel_rs4 = (velocity4 / conv4) * 1000;
      
//      Serial.print(" | Delta pos: "); Serial.print(newPos - oldPos);
//      Serial.print(" | Delta time: "); Serial.print(newTime - oldTime);
//      Serial.print(" | Velocity Rev/s: "); Serial.println(vel_rs);

      // THINK
      Setvel = remapVal(inputVal); // revs per sec
      Input3 = vel_rs3;
      Input4 = vel_rs4;
      //    int Output2 = Kp * (Setvel - vel_rs);
      PID3.Compute();
      PID4.Compute();

      // ACT
      setVelocity(m3, Output3);
      setVelocity(m4, Output4);
      lcd.clear();
      lcd.print("Motor Velocity:");
      lcd.setCursor(0, 1);
      lcd.print(Setvel);
      lcd.setCursor(5, 1);
      lcd.print("rev/s");
    }
  }
}

double remapVal(int sensorVal) {
  // sensorVal - input sensor value from potentiometer
  // remaps sensor value from potentiometer to a velocity
  double newVal = 8.00 * ((double) sensorVal - 70.0) / 930.0;
  return newVal;

}

void setVelocity(Adafruit_DCMotor *motor, int vel) {
  // motor - which motor object to set velocity for
  // vel - -255 to +255

  if (vel > 255) vel = 255;
  else if (vel < -255) vel = -255;

  if (vel >= 0) {
    motor->run(FORWARD);
    motor->setSpeed(vel);
  }

  else {
    motor->run(BACKWARD);
    motor->setSpeed(-vel);
  }
}

