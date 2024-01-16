//Include necessary libraries
#include "ms4525do.h"
#include <Servo.h>
#include <Encoder.h>
#include <LiquidCrystal_I2C.h>
#include

//Variables for pins to be used
#define BRAKE 0
#define PITCH_CONTROL 1
#define SW_PARALLEL 2
#define SW0 3
#define SW1 4
#define SW2 5
#define SW3 6
#define SW4 7
#define SW5 8
#define SW6 9
#define SW7 10

typedef enum {
  startup,
  restart,
  power_curve,
  steady_power,
  survival,
  emergency_stop,
  test
} operatingState;

typedef enum {
  brake,
  pitch,
  emergency_stop_test,
  pitot_tube_calibration,
  pitot_tube_measurement
} testState


void setup() {
  // put your setup code here, to run once:
  SetupPins();
  SetupServos();
  SetupLCD();
}

void loop() {
  // put your main code here, to run repeatedly:

}

void SetupPins() {
  
}

void SetupServos() {
  
}

void SetupLCD() {
  
}
