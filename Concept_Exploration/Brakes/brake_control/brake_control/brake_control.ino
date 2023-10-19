#include <Servo.h>

#define BRAKE_PIN 3
#define BRAKE_DISENGAGED 125
#define BRAKE_ENGAGED 90

Servo brakeControl;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  pinMode(BRAKE_PIN, OUTPUT);

  brakeControl.attach(BRAKE_PIN);

  brakeControl.write(BRAKE_DISENGAGED);

}

void loop() {
  // put your main code here, to run repeatedly:
  delay(3000);

  brakeControl.write(BRAKE_ENGAGED);
  Serial.println("Engaged");

  delay(3000);

  brakeControl.write(BRAKE_DISENGAGED);
  Serial.println("Disengaged");
}
