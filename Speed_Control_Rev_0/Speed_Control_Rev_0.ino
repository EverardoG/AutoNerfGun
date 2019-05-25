// motor set up
#include <Wire.h>
#include <Adafruit_MotorShield.h>
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *m3 = AFMS.getMotor(3);

// encoder set up
#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>
Encoder e3(2, 4); // set encoder pins
const int conv = 420 * 2; // pulses per revolution - avg can reach up to 58.94

// PID set up
#include <PID_v1.h>
double Setvel, Input, Output;
double Kp = 40, Ki = 20, Kd = 2; //40,20,2 work well
PID PID3(&Input, &Output, &Setvel, Kp, Ki, Kd, DIRECT);

// LCD screen set up
#include <LiquidCrystal.h>
LiquidCrystal lcd(12,11,8,7,9,10);

// velocity measurement set up
long newPos; // position of motor
long oldPos; // last position of motor
long deltaPos; // motor control
long oldTime = 0; // last time check
long newTime = 0; // current time
long deltaTime = 0; // time passed
double velocity; // motor speed
double vel_rs; // revolutions per second

// real time control & estop
int newLoopTime = 0;
int oldLoopTime = 0;
int controlLoopInterval = 100; // run at 10 Hz
boolean realTimeRun = true;
int eStopPin = 13;

void setup() {
  // put your setup code here, to run once:

  // comms set up
  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("PID Control Test");
  AFMS.begin();  // create with the default frequency 1.6KHz

  // PID set up
  PID3.SetMode(AUTOMATIC);
  PID3.SetOutputLimits(-255, 255);

  // estop set up
  pinMode(eStopPin, INPUT_PULLUP);

  //lcd set up
  lcd.begin(16,2);
  lcd.print("Motor Velocity:");
  lcd.setCursor(0,1);
}

void loop() {
  //  Serial.print(realTimeRun);

  if (!realTimeRun) {
    while (!realTimeRun) {
      if (!digitalRead(eStopPin)) {
        while (!digitalRead(eStopPin)){delay(1);}
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
      while (!digitalRead(eStopPin)){delay(1);}
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
      long pos = e3.read();

      // velocity calculations
      oldTime = newTime;
      newTime = millis();

      oldPos = newPos;
      newPos = pos;

      velocity = ( (double) (newPos - oldPos)) / ( (double) (newTime - oldTime)); // ticks per millisecond

      vel_rs = (velocity / conv) * 1000;
      Serial.print(" | Delta pos: "); Serial.print(newPos - oldPos);
      Serial.print(" | Delta time: "); Serial.print(newTime - oldTime);
      Serial.print(" | Velocity Rev/s: "); Serial.println(vel_rs);

      // THINK
      Setvel = 6.5; // revs per sec
      Input = vel_rs;
      //    int Output2 = Kp * (Setvel - vel_rs);
      PID3.Compute();

      // ACT
      setVelocity(m3, Output);
      lcd.clear();
      lcd.print("Motor Velocity:");
      lcd.setCursor(0,1);
      lcd.print(vel_rs);
      lcd.setCursor(5,1);
      lcd.print("rev/s");
      //  setVelocity(m3, 255);
    }
  }
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

