#include <Servo.h>
#define LIN_ACTUATOR_PIN 3
#define EXTEND 1000 // Pulse width in microseconds corresponding to full extension
#define RETRACT 2000 // Pulse width in microseconds corresponding to full retraction
#define HALF 1500 // Pulse width in microseconds corresponding half extension

int half = 1000; // Increment this to test granularity
Servo linActuator;
int position;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  linActuator.attach(LIN_ACTUATOR_PIN);
  linActuator.writeMicroseconds(RETRACT); // Write a value between 1000 and 2000 with this function to adjust position
}

void loop() {
  // linActuator.writeMicroseconds(EXTEND_L12);
  // Serial.println("Linear actuator set to full extension: ");
  // position = linActuator.read();
  // Serial.println(position);
  // delay(3000); // delay for 3 seconds
  // linActuator.writeMicroseconds(RETRACT_L12);
  // Serial.println("Linear actuator set to full retraction: ");
  // position = linActuator.read();
  // Serial.println(position);
  // delay(3000); // delay for 3 seconds
  linActuator.writeMicroseconds(half);
  Serial.println("Linear actuator set to half extension: ");
  float position = linActuator.read();
  Serial.println(position);
  //delay(500); // delay for 3 seconds
  // linActuator.writeMicroseconds(RETRACT_L12);
  // Serial.println("Linear actuator set to full retraction: ");
  // position = linActuator.read();
  // Serial.println(position);
  // delay(3000); // delay for 3 seconds
  half = half + 1; // Comment out when just doing range test.
  if (half > 1500){
    while(1); 
  }
}
