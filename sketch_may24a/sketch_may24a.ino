/* Encoder Library - TwoKnobs Example
 * http://www.pjrc.com/teensy/td_libs_Encoder.html
 *
 * This example code is in the public domain.
 */
#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>

Encoder topEncoder(2, 3); // set encoder pins
int m1Out = 11; // output to motor
long newPosTop; // position of motor
long oldPosTop; // last position of motor
long deltaPosTop; // motor control
long oldTime = 0; // last time check
long newTime = 0; // current time
long deltaTime = 0; // time passed
double velocity; // motor speed
double Input;
double Output;
double setPoint;
double Kp = 0.2;

void setup() {
  Serial.begin(9600);
  Serial.println("Position Test");
  pinMode(m1Out, OUTPUT);

  Input = newPosTop;
  setPoint = 1000;
}

void loop() {

  // SENSE
  // time check
  oldTime = newTime;
  newTime = millis();
  deltaTime = newTime-oldTime;
  
  // Encoder sensing
  oldPosTop = newPosTop;
  newPosTop = topEncoder.read(); //check encoder value
  deltaPosTop = newPosTop - oldPosTop; // get position change

  // velocity
  velocity = deltaPosTop / deltaTime;
  // THINK
  // PID control
  Input = newPosTop;
  Serial.print(" In: "); Serial.print(Input);
  Serial.print(" | Diff: "); Serial.print(Input-setPoint);
  Output = (setPoint - Input) * Kp;
  Serial.print(" | Motor Out: "); Serial.println(Output);

  // edge cases
  if (Output > 255) Output = 255;
  else if (Output < 0) Output = 0;

  Serial.println((int) Output);

  // ACT
  analogWrite(m1Out, Output); // set motor speed
  
   
//  long newLeft, newRight;
//  newLeft = knobLeft.read();
//  newRight = knobRight.read();
//  if (newLeft != positionLeft || newRight != positionRight) {
//    Serial.print("Left = ");
//    Serial.print(newLeft);
//    Serial.print(", Right = ");
//    Serial.print(newRight);
//    Serial.println();
//    positionLeft = newLeft;
//    positionRight = newRight;
//  }
  // if a character is sent from the serial monitor,
  // reset both back to zero.
//  if (Serial.available()) {
//    Serial.read();
//    Serial.println("Reset both knobs to zero");
//    knobLeft.write(0);
//    knobRight.write(0);
//  }
}
