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
#define MINIMUM_PITCH 1200
#define MAXIMUM_PITCH 1800
#define BRAKE_DISENGAGED 1300
#define BRAKE_ENGAGED 1700

//Resistor bank global constants
#define INITIAL_RESISTANCE 28.0
#define BASE_RESISTANCE 10.0 //TODO: REPLACE THIS VALUE WITH AN ACTUAL VALUE

//Arduino Mega global constants
#define OPERATING_VOLTAGE 5.0
#define ANALOG_RANGE 1023.0

//LCD writing global constants. This says to update the LCD every ___ milliseconds
#define LCD_WRITE_INTERVAL 1000

//Enum for the overall operating state machine
typedef enum {
  restart,
  power_curve,
  steady_power,
  survival,
  emergency_stop,
  test
} operatingStateMachine;

operatingStateMachine operatingState = restart;

//Enum for the substates within the "test" state of the overall state machine
typedef enum {
  test_select,
  power_select,
  brake,
  pitch,
  pitot_tube_calibration,
  pitot_tube_measurement,
  read_base_resistor_voltage,
  read_power,
  read_rpm,
  emergency_button,
  load_disconnect,
  survival_test,
  steady_power_test,
  power_curve_test,
  exit_test
} testStateMachine;

testStateMachine testState = test_select;

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

//Global variables for keeping track of time
unsigned long previousMillis = 0;

void setup() {
  //Start up the Serial Monitor
  //For the entry into the Serial Monitor, make sure the box next to the baud rate reads "Newline"
  //and not "Both NL & CR" or anything else. Otherwise, text entry will not work, and no testing
  //state transitions can occur
  Serial.begin(9600);
  
  // put your setup code here, to run once:
  SetupPins();
  Serial.println("Pins set up");
  SetupLCD();
  Serial.println("LCD set up");
  SetupServos();
  Serial.println("Servo set up");
  SetupPressureSensor();
  Serial.println("Pressure sensor set up");

  //Set the states for the state machine
  operatingState = restart;
  testState = test_select;

  //Calibrate the pitot tube
  CalibratePitotTube();
}

void loop() {
  //For the entry into the Serial Monitor, make sure the box next to the baud rate reads "Newline"
  //and not "Both NL & CR" or anything else. Otherwise, text entry will not work, and no testing
  //state transitions can occur
  
  // put your main code here, to run repeatedly:
  //Read in the input if available to determine whether to switch to the test state
  static bool testing = false;

  //Update the time to check reading the LCD
  unsigned long currentMillis = millis();
  if(currentMillis - previousMillis >= LCD_WRITE_INTERVAL) {
    //Write to the LCD and reset the value of the previous writing time
    WriteToLCD();  
    previousMillis = currentMillis;
  }



  if(Serial.available()){
    String input = Serial.readStringUntil('\n');
    Serial.println(input);
    if(input.equals("test")) {
      Serial.println("Entering test state");
      operatingState = test;
    }
  }

  //Operating state transitions
  switch(operatingState) {
    
    case restart:
    {
      break;
    }
      
    case power_curve:
    {
      break;
    }
    
    case steady_power:
    {
      break;
    }
      
    case survival:
    {
      break;
    }
      
    case emergency_stop:
    {
      break;
    }
      
    case test:
    {
      testing = true;
      break;
    }
  }

  //Operating state actions
  switch(operatingState) {
    
    case restart:
    {
      break;
    }
      
    case power_curve:
    {
      break;
    }
    
    case steady_power:
    {
      break;
    }
      
    case survival:
    {
      break;
    }
      
    case emergency_stop:
    {
      break;
    }
    
    case test:
    {
      break;
    }
  }

  //This is the testing state machine. It should only execute if we are actively testing
  if(testing) {
    WriteToLCD();
    //Testing state transitions
    switch(testState) {
      //Select a test to run
      case test_select:
      {
        Serial.println("Entering test selection state");
        SelectTest();
        break;
      }

      //Return to test selection after test is run
      case power_select:
      {
        testState = test_select;
        break;
      }
        
      //Return to test selection after test is run
      case brake:
      {
        testState = test_select;
        break;
      }
     
      //Return to test selection after test is run
      case pitch:
      {
        testState = test_select;
        break;
      }

      //Return to test selection after test is run
      case pitot_tube_calibration:
      {
        testState = test_select;
        break;
      }
     
      //Return to test selection after test is run
      case pitot_tube_measurement:
      {
        testState = test_select;
        break;
      }

      //Return to test selection after test is run
      case read_base_resistor_voltage:
      {
        testState = test_select;
        break;
      }

      //Return to test selection after test is run
      case read_power:
      {
        testState = test_select;
        break;
      }

      //Return to test selection after test is run
      case read_rpm:
      {
        testState = test_select;
        break;
      }
     
      //Return to test selection after test is run
      case emergency_button:
      {
        testState = test_select;
        break;
      }

      //Return to test selection after test is run
      case load_disconnect:
      {
        testState = test_select;
        break;
      }
      
      //Return to test selection after test is run
      case survival_test:
      {
        testState = test_select;
        break;
      }
     
      //Return to test selection after test is run
      case steady_power_test:
      {
        testState = test_select;
        break;
      }
     
      //Return to test selection after test is run
      case power_curve_test:
      {
        testState = test_select;
        break;
      }
     
      //Return to the operating state machine in the restart state
      case exit_test:
      {
        operatingState = restart; 
        testing = false;
        testState = test_select;
        break; 
      }
        
    }

    WriteToLCD();
    
    //Testing state actions
    switch(testState) {

      //Don't do anything in the test selection state, just return to state transitions
      case test_select:
      {
        break;
      }

      //Select the power source. Usually best to power nacelle components externally (from the wall) 
      //if the turbine is not running and internally if the turbine is running
      case power_select:
      {
        Serial.print("Select the desired power source. Type e to power externally (from the wall), ");
        Serial.println("i to power internally (from the turbine).");
        while(!Serial.available()) {
        }
        String powerInput = Serial.readStringUntil('\n');
        bool setExternal;

        if(powerInput.equals("i")) {
          setExternal = false;
          SetPowerMode(setExternal);
        }
        else if(powerInput.equals("e")) {
          setExternal = true;
          SetPowerMode(setExternal);
        }
        else {
          Serial.println("Invalid entry. Returning to test selection...");
        }
        break;
      }

      //Activate and deactivate the brake using the linear actuator
      case brake:
      {
        Serial.println("Select brake setting. Type e for engaged, d for disengaged");
        while(!Serial.available()) {
        }
        String brakeInput = Serial.readStringUntil('\n');

        if(brakeInput.equals("e")) {
          SetBrake(BRAKE_ENGAGED);
        }
        else if(brakeInput.equals("d")) {
          SetBrake(BRAKE_DISENGAGED);
        }
        else {
          Serial.println("Invalid entry. Returning to test selection...");
        }
        break;
      }

      //Change the pitch angle of the blades using the linear actuator
      case pitch:
      {
        char message[100];
        sprintf(message, "Select pitch value. Enter an integer value between %d and %d (inclusive)", 
                MINIMUM_PITCH, MAXIMUM_PITCH);
        Serial.println(message);
        
        int pitchInput = ReadInputInt();

        if(pitchInput >= MINIMUM_PITCH && pitchInput <= MAXIMUM_PITCH) {
          SetPitch(pitchInput);
        }
        else {
          Serial.println("Invalid entry. Returning to test selection...");
        }
        break;
      }

      //Calibrate the pitot tube using wind speeds
      case pitot_tube_calibration:
      {
        CalibratePitotTube();
        break;
      }

      //Read a wind speed measurement from the pitot tube
      case pitot_tube_measurement:
      {
        float windSpeed = ReadWindSpeed();
        
        Serial.print("Current wind speed: ");
        Serial.print(windSpeed);
        Serial.println(" m/s");
        break;
      }

      //Read the voltage across the resistor
      case read_base_resistor_voltage:
      {
        int voltageInt = analogRead(LOAD_VOLTAGE);
        float baseResistorVoltage = (voltageInt / ANALOG_RANGE) * OPERATING_VOLTAGE;

        Serial.print("Current base resistor voltage: ");
        Serial.print(voltageInt);
        Serial.println(" V");
        break;
      }

      //Read the current power output from the turbine
      case read_power:
      {
        float power = CalculatePower();

        Serial.print("Current power output: ");
        Serial.print(power);
        Serial.println(" W");
        break;
      }

      //Read the current rpm from the encoder
      case read_rpm:
      {
        float rpm = ReadRPM();

        Serial.print("Current RPM: ");
        Serial.print(rpm);
        Serial.println(" RPM");
        break;
      }

      //Test emergency stop when the emergency button is pressed
      case emergency_button:
      {
        break;
      }

      //Test emergency stop when the load is disconnected
      case load_disconnect:
      {
        break;
      }

      //Test the survival state
      case survival_test:
      {
        break;
      }

      //Test the steady power state
      case steady_power_test:
      {
        break;
      }

      //Test the power curve state
      case power_curve_test:
      {
        break;
      }

      //No state actions if the test state is being exited
      case exit_test:
      {
        break;
      }

      //Trigger this message to the serial monitor if the state machine gets
      //changed to an invalid angle for some reason
      default:
      {
        Serial.println("Default state triggered. Something went wrong.");
      }
    }
  }

  
  
}

//Calculates an estimate of the current power output from the turbine
float CalculatePower() {
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

  Serial.println("Calibration complete.");
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
  while(!Serial.available()){
  }
  String input = Serial.readStringUntil('\n');

  //Cast the input to type float and return it
  int inputInt = input.toInt();

  if(inputInt == 0) {
    Serial.println("WARNING: Non-integer input detected. If you put in a zero, badly done.");
  }
  
  return inputInt;
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
  Serial.print("Enter a desired test. Valid options: pwrsrc, brake, pitch, ptcalib, ptread, readpwr, readvolt, readrpm, ebutt, loaddis,\n");
  Serial.print("survival, steadypwr, pwrcurve, exit\n");

  while(!Serial.available()){
  }
  String input = Serial.readStringUntil('\n');

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

  if(input.equals("readpwr")) {
    testState = read_power;
  }
  
  if(input.equals("readvolt")) {
    testState = read_base_resistor_voltage;
  }

  if(input.equals("readrpm")) {
    testState = read_rpm;
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

//Write desired data to the LCD
void WriteToLCD() {
  //Clear the LCD screen
  lcd.clear();
  //Write out the current state. All of the below code does this. 
  lcd.setCursor(0, 0);
  lcd.print("State: ");
  if(operatingState == restart) {
    lcd.print("restart");
  }
  else if(operatingState == power_curve) {
    lcd.print("power curve");
  }
  else if(operatingState == steady_power) {
    lcd.print("steady power");
  }
  else if(operatingState == survival) {
    lcd.print("survival");
  } 
  else if(operatingState == emergency_stop) {
    lcd.print("e stop");
  }
  else if(operatingState == test) {
    if(testState == test_select) {
      lcd.print("test select");
    }
    if(testState == power_select) {
      lcd.print("power select");
    }
    if(testState == brake) {
      lcd.print("brake test");
    }
    if(testState == pitch) {
      lcd.print("pitch test");
    }
    if(testState == pitot_tube_calibration) {
      lcd.print("pt calib");
    }
    if(testState == pitot_tube_measurement) {
      lcd.print("pt measure");
    }
    if(testState == read_base_resistor_voltage) {
      lcd.print("read voltage");
    }
    if(testState == read_power) {
      lcd.print("read power");
    }
    if(testState == read_rpm) {
      lcd.print("read rpm");
    }
    if(testState == emergency_button) {
      lcd.print("e button");
    }
    if(testState == load_disconnect) {
      lcd.print("load discon");
    }
    if(testState == survival_test) {
      lcd.print("survival test");
    }
    if(testState == steady_power_test) {
      lcd.print("stdy pwr test");
    }
    if(testState == power_curve_test) {
      lcd.print("pwr crve test");
    }
    if(testState == exit_test) {
      lcd.print("exiting test ");
    }
  }
  
}
