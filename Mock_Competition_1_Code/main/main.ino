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
#define E_BUTTON 31
#define LOAD_VOLTAGE A0
//TODO: Find out why there are two relays
#define NACELLE_RELAY 32
#define LOAD_BOX_RELAY 33

//Linear actuator global constants. For linear actuators, 1000 is fully extended and 2000 is fully retracted
#define INITIAL_PITCH 1500
#define BRAKE_DISENGAGED 1700
#define BRAKE_ENGAGED 1300

//Resistor bank global constants
#define INITIAL_RESISTANCE 28.0
#define BASE_RESISTANCE 10.0 //TODO: REPLACE THIS VALUE WITH AN ACTUAL VALUE

//Arduino Mega global constants
#define OPERATING_VOLTAGE 5.0
#define ANALOG_RANGE 1023.0

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
  pitot_tube_calibration,
  pitot_tube_measurement,
  read_base_resistor_voltage,
  read_power,
  emergency_button,
  load_disconnect,
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
float voltageDividerFactor = (BASE_RESISTANCE + INITIAL_RESISTANCE) / BASE_RESISTANCE;

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
      
      case test_select:
        SelectTest();
        break;
        
      case power_select:
        testState = test_select;
        break;
        
      case brake:
        testState = test_select;
        break;
     
      case pitch:
        testState = test_select;
        break;

      case pitot_tube_calibration:
        testState = test_select;
        break;
     
      case pitot_tube_measurement:
        testState = test_select;
        break;

      case read_base_resistor_voltage:
        testState = test_select;
        break;

      case read_power:
        testState = test_select;
        break;
     
      case emergency_button:
        testState = test_select;
        break;
     
      case load_disconnect:
        testState = test_select;
        break;
     
      case survival_test:
        testState = test_select;
        break;
     
      case steady_power_test:
        testState = test_select;
        break;
     
      case power_curve_test:
        testState = test_select;
        break;
     
      case exit_test:
        operatingState = restart; 
        testing = false;
        break;     
    }

    //Testing state actions
    switch(testState) {
      
      case test_select:
        break;
        
      case power_select:
        Serial.print("Select the desired power source. Type e to power externally (from the wall), ");
        Serial.println("i to power internally (from the turbine).");

        String input = ReadInputString();

        if input.equals("i") {
          external = false;
          SetPowerMode(external);
        }
        else if input.equals("e") {
          external = true;
          SetPowerMode(external);
        }
        else {
          Serial.println("Invalid entry. Returning to test selection");
        }
        break;
        
      case brake:
        Serial.println("Select brake setting. Type e for engaged, d for disengaged");
        
        String input = ReadInputString();

        if input.equals("e") {
          SetBrake(BRAKE_ENGAGED);
        }
        else if input.equals("d") {
          SetBrake(BRAKE_DISENGAGED);
        }
        else {
          Serial.println("Invalid entry. Returning to test selection");
        }
        break;
     
      case pitch:
        Serial.println("Select pitch value");
        
        int input = ReadInputInt();

        if input.equals("e") {
          SetBrake(BRAKE_ENGAGED);
        }
        else if input.equals("d") {
          SetBrake(BRAKE_DISENGAGED);
        }
        else {
          Serial.println("Invalid entry. Returning to test selection");
        }
        break;

      case pitot_tube_calibration:
        break;
     
      case pitot_tube_measurement:
        break;
        
      case read_base_resistor_voltage:
        break;

      case read_power:
        break;
     
      case emergency_button:
        break;
     
      case load_disconnect:
        break;
     
      case survival_test:
        break;
     
      case steady_power_test:
        break;
     
      case power_curve_test:
        break;
     
      case exit_test:
        break;    
    }
  }
  
  
}

//Calculates an estimate of the current power output from the turbine
void CalculatePower() {
  //Read the voltage from the load pin and convert it to volts. We are reading in a scaled down voltage that is
  //only measured across the base resistor as opposed to the whole load. As such, we calculated a voltage
  //divider factor that converts the voltage we measure to the total voltage across the load, which is the value
  //we actually want. That voltage divider factor is defined globally and updated each time the resistor bank
  //changes resistance values.
  int voltageInt = analogRead(LOAD_VOLTAGE);
  float baseResistorVoltage = (voltageInt / ANALOG_RANGE) * OPERATING_VOLTAGE;
  float loadVoltage = baseResistorVoltage * voltageDividerFactor;

  //Read the resistance from the resistor bank
  float resistance = resistorBank.getResistance();

  //Calculate and return the power
  float currentPower = (loadVoltage * loadVoltage) / resistance;
  return currentPower;
}

//Calibrates the pitot tube based on 3 known wind speeds
void CalibratePitotTube() {
  // Arrays to store the measured pressure values and wind speed calibration values
  float windSpeedArray[3];
  float pressureArray[3];

  // Read 3 wind speeds and pressures to calibrate
  for(int i = 0; i < 3; i++) {
    // Get the wind speed and store in the array
    windSpeedArray[i] = (float) InputWindSpeed();

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

//Reads a manually entered wind speed from the serial monitor during calibration
float InputWindSpeed() {
  Serial.println("Please enter a new wind speed");

  //Read the wind speed from the serial monitor, output it, and return it
  int windSpeed = ReadInputInt();

  Serial.print("Wind speed of ");
  Serial.print(windSpeed, 1);
  Serial.println("m/s entered.");
  
  return windSpeed;
}

//Reads whether the emergency stop button is pressed. Returns true if pressed, false if not
bool IsButtonPressed() {
  return digitalRead(E_BUTTON);
}

//Reads whether the load is connected. Returns true if connected and false if not
bool IsLoadConnected() {
  //TODO: Might change to add some sort of threshold or something
  int voltage = analogRead(LOAD_VOLTAGE);
  return voltage != 0;
}

//Assigns the optimal load based on current conditions
void OptimizeLoad() {
  
}

//Reads an integer input from the serial monitor
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
  int inputInt = input.toInt();

  if(inputInt == 0) {
    Serial.println("WARNING: Non-integer input detected. If you put in a zero, badly done.");
  }
  
  return inputInt;
}

//Reads a string from the serial monitor
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

//Reads the current RPMs from the encoder
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
  
    // Average 100 measurements of the pressure
    float pressureSum = 0.0;
    int i = 0;
    int j = 0;
    for (i = 0; i < 100; i++) {
      if(pressureSensor.Read()) {
        j++;
        float currReading = pressureSensor.pres_pa();
        
        pressureSum += currReading;
      }
    }
    float pressureAverage = pressureSum / j;
    
  float windSpeed = (-B + sqrt(pow(B, 2) - 4*A*(C - pressureAverage))) / (2*A);
  
  return windSpeed;
}

//Selects a test state based on manual user input
void SelectTest() {
  Serial.print("Enter a desired test. Valid options: pwrsrc\n brake\n pitch\n ebutt\n loaddis\n ptcalib\n ptread\n");
  Serial.print("survival\n steadypwr\n pwrcurve\n readpwr\n readvolt\n exit\n");
  
  String input = ReadInputString();

  if(input.equals("pwrsrc")) {
    testState = power_select;
  }
  
  if(input.equals("brake")) {
    testState = brake;
  }
  
  if(input.equals("pitch")) {
    testState = pitch;
  }
  
  if(input.equals("ebutt")) {
    testState = emergency_button;
  }
  
  if(input.equals("loaddis")) {
    testState = load_disconnect;
  }
  
  if(input.equals("ptcalib")) {
    testState = pitot_tube_calibration;
  }
  
  if(input.equals("ptread")) {
    testState = pitot_tube_measurement;
  }
  
  if(input.equals("survival")) {
    testState = survival_test;
  }
  
  if(input.equals("steadypwr")) {
    testState = steady_power_test;
  }
  
  if(input.equals("pwrcurve")) {
    testState = power_curve_test;
  }

  if(input.equals("readpwr")) {
    testState = read_power;
  }
  
  if(input.equals("readvolt")) {
    testState = read_base_resistor_voltage;
  }
  
  if(input.equals("exit")) {
    testState = exit_test;
  }
}

//Sets the brake linear actuator to a given position
void SetBrake(int brakePosition) {
  //We might need to flip a switch to turn on the actuator
  
  //Set the servo to the desired angle
  brakeActuator.writeMicroseconds(brakePosition);
}

//Sets the pitch linear actuator to a given position
void SetPitch(int pitchAngle){
  //We might need to flip a switch to turn on the actuator

  //Set the servo to the desired angle
  pitchActuator.writeMicroseconds(pitchAngle);
}

//Sets the load to a given resistance value
void SetLoad(float resistance) {
  //TODO: Check with power to make sure this is the best way to do this
  resistorBank.setBestResistanceMatch(resistance);
}

//Switches the power supply between nacelle power coming from the wind turbine and nacelle power
//coming from the wall supply
void SetPowerMode(bool setExternal) {
  //TODO: ARE THESE THE RIGHT VALUES?
  if(setExternal) {
    digitalWrite(NACELLE_RELAY, LOW);
    digitalWrite(LOAD_BOX_RELAY, HIGH);
  }
  else {
    digitalWrite(NACELLE_RELAY, HIGH);
    digitalWrite(LOAD_BOX_RELAY, LOW);
  }
}

//Sets up the LCD screen
void SetupLCD() {
  //Initialize the LCD screen and backlight it
  lcd.init();
  lcd.backlight();
}

//Sets up all Arduino pins
void SetupPins() {
  //Set all output pins
  pinMode(BRAKE_CONTROL, OUTPUT);
  pinMode(PITCH_CONTROL, OUTPUT);
  pinMode(E_BUTTON, INPUT);
  pinMode(LOAD_VOLTAGE, INPUT);
  pinMode(NACELLE_RELAY, OUTPUT);
  digitalWrite(NACELLE_RELAY, LOW); //TODO: Find out whether this is the right value
  pinMode(LOAD_BOX_RELAY, OUTPUT);
  digitalWrite(LOAD_BOX_RELAY, HIGH); //TODO: Find out whether this is the right value
}

//Sets up the pressure sensor
void SetupPressureSensor() {
  pressureSensor.Config(&Wire, 0x28, 1.0f, -1.0f);
  /* Starting communication with the pressure transducer */
  if (!pressureSensor.Begin()) {
    Serial.println("Error communicating with the pressure sensor");
  }
}

//Sets up all servo motors
void SetupServos() {
  //Initialize pitch control linear actuator
  pitchActuator.attach(PITCH_CONTROL);
  SetPitch(INITIAL_PITCH);

  //Initialize brake control linear actuator
  brakeActuator.attach(BRAKE_CONTROL);
  SetBrake(BRAKE_DISENGAGED);
}
