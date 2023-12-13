#include <Servo.h>
#define LIN_ACTUATOR_PIN 3
#define EXTEND 1100 // Pulse width in microseconds corresponding to full extension
#define RETRACT 1400 // Pulse width in microseconds corresponding to full retraction

Servo linActuator;
int position;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  linActuator.attach(LIN_ACTUATOR_PIN);
  linActuator.writeMicroseconds(RETRACT); // Write a value between 1000 and 2000 with this function to adjust position
}

void loop() {
  for(int i = RETRACT; i > EXTEND; i--) {
    linActuator.writeMicroseconds(i);
    delay(20);
  }
  delay(2000);
  for(int i = EXTEND; i < RETRACT; i++) {
    linActuator.writeMicroseconds(i);
    delay(20);
  }
  delay(2000);
}
