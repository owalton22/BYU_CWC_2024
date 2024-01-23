//Include necessary libraries
#include "ms4525do.h"
#include <Servo.h>
#include <Encoder.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <string.h>
#include <VariableLoad.h>
#include <BasicLinearAlgebra.h>

//Global constants for pins to be used
//These pins need interrupt capability
#define ENCODER_PIN_1 2
#define ENCODER_PIN_2 3
//These pins need PWM capability
#define BRAKE_CONTROL 4
#define PITCH_CONTROL 5
//These pins need no special capabilities
#define SW_PARALLEL 22
#define SW0 23
#define SW1 24
#define SW2 25
#define SW3 26
#define SW4 27
#define SW5 28
#define SW6 29
#define SW7 30

//Linear actuator global constants. For linear actuators, 1000 is fully extended and 2000 is fully retracted
#define INITIAL_PITCH 1500
#define BRAKE_DISENGAGED 1700
#define BRAKE_ENGAGED 1300

//Resistor bank global constants
#define INITIAL_RESISTANCE 28.0

//Enum for the overall operating state machine
enum operatingStateMachine {
  restart,
  power_curve,
  steady_power,
  survival,
  emergency_stop,
  test
} operatingState;

//Enum for the substates within the "test" state of the overall state machine
enum testStateMachine {
  test_select,
  power_select,
  brake,
  pitch,
  emergency_button,
  load_disconnect,
  pitot_tube_calibration,
  pitot_tube_measurement,
  survival_test,
  steady_power_test,
  power_curve_test,
  exit_test
} testState;

//Servos (linear actuators)
Servo pitchActuator;
Servo brakeActuator;

//Encoder
Encoder encoder(ENCODER_PIN_1, ENCODER_PIN_2);

//Pressure sensor
bfs::Ms4525do pressureSensor;

//Variable load object
VariableLoad resistorBank(SW_PARALLEL, SW0, SW1, SW2, SW3, SW4, SW5, SW6, SW7, INITIAL_RESISTANCE);

//Liquid Crystal Display
LiquidCrystal_I2C lcd(0x3f, 20, 4);

//Coefficients for wind speed calibration
float A;
float B;
float C;

void setup() {
  // put your setup code here, to run once:
  SetupPins();
  SetupLCD();
  SetupServos();
  SetupPressureSensor();

  //Set the states for the state machine
  operatingState = restart;
  testState = test_select;

  //Calibrate the pitot tube
  CalibratePitotTube();
}

void loop() {
  // put your main code here, to run repeatedly:
  //Read in the input if available to determine whether to switch to the test state
  static bool testing = false;
  
  String input = ReadInputString();
  if(input.equals("test")) {
    operatingState = test;
  }

  //Operating state transitions
  switch(operatingState) {
    
    case restart:
      break;
      
    case power_curve:
      break;
    
    case steady_power:
      break;
      
    case survival:
      break;
      
    case emergency_stop:
      break;
      
    case test:
      testing = true;
      break;
  }

  //Operating state actions
  switch(operatingState) {
    
    case restart:
      break;
      
    case power_curve:
      break;
    
    case steady_power:
      break;
      
    case survival:
      break;
      
    case emergency_stop:
      break;
      
    case test:
      break;
  }

  //This is the testing state machine. It should only execute if we are actively testing
  if(testing) {
    //Testing state transitions
    switch(testState) {
      case brake:
      case pitch:
      case emergency_stop_test:
      case pitot_tube_calibration:
      case pitot_tube_measurement:
      case exit_test:
          operatingState = restart; 
          testing = false;
          break;     
    }

    //Testing state actions
    switch(testState) {
      
    }
  }
  
  
}

void CalibratePitotTube() {
  // Arrays to store the measured pressure values and wind speed calibration values
  float windSpeedArray[3];
  float pressureArray[3];

  // Read 3 wind speeds and pressures to calibrate
  for(int i = 0; i < 3; i++) {
    // Get the wind speed and store in the array
    windSpeedArray[i] = (float) ReadWindSpeed();

    // Average 1000 pressure readings to get a pressure at that wind speed
    float pressureSum = 0.0;
    int j = 0;
    int k = 0;
    for (j = 0; j < 1000; j++) {
      if(pressureSensor.Read()) {
        k++;
        float currReading = pressureSensor.pres_pa();
        
        pressureSum += currReading;
      }
    }
    float pressureAverage = pressureSum / k;

    // Store the pressure in the array
    pressureArray[i] = pressureAverage;
  }

  Serial.println("All wind speeds entered. Calibrating...");

  // Matrices to hold the pressures and quadratic fit data. These arrays will be used to solve the matrix 
  // problem Ax = B, where A is the quadratic fit matrix based on the wind speeds, x is the values a, b, and c 
  // in the quadratic equation y = ax^2 + bx + c, and B is the output pressure values. We will be solving for 
  // a, b, and c
  BLA::Matrix<3, 1> pressures;
  pressures = {pressureArray[0], pressureArray[1], pressureArray[2]};
  
  BLA::Matrix<3, 3> quadFit;
  quadFit = {pow(windSpeedArray[0], 2), windSpeedArray[0], 1, 
             pow(windSpeedArray[1], 2), windSpeedArray[1], 1, 
             pow(windSpeedArray[2], 2), windSpeedArray[2], 1};

  // Invert the quadratic fit matrix, solve for a, b, and c, and store the values
  BLA::Invert(quadFit);

  BLA::Matrix<3, 1> abc = quadFit * pressures;
  
  A = abc(0, 0);
  B = abc(1, 0);
  C = abc(2, 0);
}

int ReadInputInt() {
  bool inputDetected = false;

  String input = ReadInputString();

  //Keep trying to read input until input is detected. This function is only used in the test state 
  //and setup so code blocking is not a problem.
  while(!inputDetected) {
    input = ReadInputString();
    if(input.length() > 0) {
      inputDetected = true;
    }
  }

  //Cast the input to type float and return it
  float inputInt = input.toInt();
  return inputInt;
}

String ReadInputString() {
  //Initialize an input string
  String inputString = "";

  //Read in all entered values and store them as a string
  while(Serial.available()) {
    //Read values until a newline is read, which signifies the end of the input.
    while(true) {
      char c = Serial.read();
      
      if(c == '\n') {
        break;
      }

      Serial.println(c);
      inputString += c;
    }
  }

  return inputString;
}

float ReadRPM() {
  static long oldPosition = 0;
  static long oldTime = 0;
  static float rpm = 0;

  long newPosition = encoder.read();

  if(newPosition != oldPosition) {
    long newTime = micros();
    float dx = newPosition - oldPosition;
    float dt = newTime - oldTime;
    oldPosition = newPosition;
    oldTime = newTime;
    //TODO: Check this conversion to see if it needs to be updated for the new encoder
    //1000000*60 converts time from microseconds to minutes, 2048 converts dx to rotations
    rpm = (dx * 1000000 * 60) / (dt * 2048);
  }

  return rpm;
}

float ReadWindSpeed() {
  Serial.println("Please enter a new wind speed");

  //Read the wind speed from the serial monitor, output it, and return it
  int windSpeed = ReadInputInt();

  Serial.print("Wind speed of ");
  Serial.print(windSpeed, 1);
  Serial.println("m/s entered.");
  
  return windSpeed;
}

void SetBrake(int brakePosition) {
  //We might need to flip a switch to turn on the actuator
  
  //Set the servo to the desired angle
  brakeActuator.writeMicroseconds(brakePosition);
}

void SetPitch(int pitchAngle){
  //We might need to flip a switch to turn on the actuator

  //Set the servo to the desired angle
  pitchActuator.writeMicroseconds(pitchAngle);
}

//Function to set up the LCD screen
void SetupLCD() {
  //Initialize the LCD screen and backlight it
  lcd.init();
  lcd.backlight();
}

//Function to set up all Arduino pins
void SetupPins() {
  //Set all output pins
  pinMode(BRAKE_CONTROL, OUTPUT);
  pinMode(PITCH_CONTROL, OUTPUT);
  pinMode(SW_PARALLEL, OUTPUT);
  pinMode(SW0, OUTPUT);
  pinMode(SW1, OUTPUT);
  pinMode(SW2, OUTPUT);
  pinMode(SW3, OUTPUT);
  pinMode(SW4, OUTPUT);
  pinMode(SW5, OUTPUT);
  pinMode(SW6, OUTPUT);
  pinMode(SW7, OUTPUT);
}

//Function to set up the pressure sensor
void SetupPressureSensor() {
  pressureSensor.Config(&Wire, 0x28, 1.0f, -1.0f);
  /* Starting communication with the pressure transducer */
  if (!pressureSensor.Begin()) {
    Serial.println("Error communicating with the pressure sensor");
  }
}

//Function to set up all servo motors
void SetupServos() {
  //Initialize pitch control linear actuator
  pitchActuator.attach(PITCH_CONTROL);
  SetPitch(INITIAL_PITCH);

  //Initialize brake control linear actuator
  brakeActuator.attach(BRAKE_CONTROL);
  SetBrake(BRAKE_DISENGAGED);
}
