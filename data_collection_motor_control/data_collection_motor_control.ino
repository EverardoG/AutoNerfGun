// motor set up
#include <Wire.h>
#include <Adafruit_MotorShield.h>
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *m3 = AFMS.getMotor(3);

// encoder set up
#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>
Encoder e3(2, 4); // set encoder pins
const int conv = 745; // pulses per revolution - avg can reach up to 58.94

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

// counter for velocity iterations
int velCount = 0;

// settings
boolean dataCollection = true;

void setup() {
  // comms set up
  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("Main Flight Code Running Now");

  //shield set up
  AFMS.begin();  // create with the default frequency 1.6KHz

  // estop set up
  pinMode(eStopPin, INPUT_PULLUP);
}

void loop() {
  // put your main code here, to run repeatedly:

  if (!realTimeRun) {
    while (!realTimeRun) {
      if (!digitalRead(eStopPin)) {
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

      // increment vel counter
      if (velCount > 255) velCount = 0;
      else velCount++;

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
      if (dataCollection) {
        Serial.print(velCount); Serial.print(" , "); Serial.println(vel_rs);
      }
      else {
        Serial.print(" | Delta pos: "); Serial.print(newPos - oldPos);
        Serial.print(" | Delta time: "); Serial.print(newTime - oldTime);
        Serial.print(" | Velocity Rev/s: "); Serial.print(vel_rs);
        Serial.print(" | PWM Signal: "); Serial.println(velCount);
      }

      setVelocity(m3, velCount);

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

