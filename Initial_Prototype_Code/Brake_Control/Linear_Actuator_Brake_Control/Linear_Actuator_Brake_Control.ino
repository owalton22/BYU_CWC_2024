#include <Servo.h>
#define LIN_ACTUATOR_PIN 3
#define EXTEND 1950 // Pulse width in microseconds corresponding to full extension
#define RETRACT 1000 // Pulse width in microseconds corresponding to full retraction

Servo linActuator;
int position;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  linActuator.attach(LIN_ACTUATOR_PIN);
  linActuator.writeMicroseconds(RETRACT); // Write a value between 1000 and 2000 with this function to adjust position
}

void loop() {
  linActuator.writeMicroseconds(RETRACT);
  Serial.println("Linear actuator retracted: ");
  float position = linActuator.read();
  Serial.println(position);
  delay(10000); // delay for 5 seconds
  linActuator.writeMicroseconds(EXTEND);
  Serial.println("Linear actuator fully extended: ");
  position = linActuator.read();
  Serial.println(position);
  delay(10000); // delay for 10 seconds

}
