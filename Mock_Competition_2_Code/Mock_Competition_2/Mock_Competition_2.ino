//Include necessary libraries
#include "ms4525do.h"
#include <Servo.h>
#include <Encoder.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <string.h>
#include "VariableLoad.h"
#include <BasicLinearAlgebra.h>

//Global constants for pins to be used
//These pins need interrupt capability
#define ENCODER_PIN_1 2
#define ENCODER_PIN_2 3
//These pins need PWM capability
#define BRAKE_CONTROL 4
#define PITCH_CONTROL 5
//These pins need no special capabilities
#define SW_SHORT 45
#define SW_PARALLEL 39
#define SW0 37
#define SW1 35
#define SW2 33
#define SW3 31
#define SW4 29
#define SW5 27
#define SW6 25
#define SW7 23
#define E_BUTTON 44
#define LOAD_VOLTAGE A0
//TODO: Find out why there are two relays
#define NACELLE_RELAY 32
#define LOAD_BOX_RELAY 33

//Linear actuator global constants. For the PQ12-R, 1000 is fully extended and 2000 is fully retracted.
//May be different depending on the model of linear actuator used.
#define INITIAL_PITCH 1300
#define MINIMUM_PITCH 1800
#define MAXIMUM_PITCH 1300
#define BRAKE_DISENGAGED 1600
#define BRAKE_ENGAGED 1250

//Resistor bank global constants
#define LOAD_RESISTOR_RESISTANCE 220.0
#define MINIMUM_RESISTANCE 0.0
#define MAXIMUM_RESISTANCE 440.0
#define BASE_RESISTANCE 10.0 //TODO: REPLACE THIS VALUE WITH AN ACTUAL VALUE
#define LOAD_SHORTED 1
#define LOAD_UNSHORTED 0

//Arduino Mega global constants
#define OPERATING_VOLTAGE 5.0
#define ANALOG_RANGE 1023.0

//LCD writing global constants. This says to update the LCD every ___ milliseconds
#define UPDATE_INTERVAL 1000

//Transition wind speed values
#define CUT_IN 4
#define RATED_WIND_SPEED 10.5
#define CUT_OUT 15

//Transition RPM values
#define RATED_RPM 2000 //CHANGE THIS VALUE DURING CALIBRATION
#define SURVIVAL_PITCH 1600 //CHANGE THIS VALUE DURING CALIBRATION

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
  set_load,
  tune_load,
  short_load,
  read_load,
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
VariableLoad resistorBank(SW0, SW1, SW2, SW3, SW4, SW5, SW6, SW7, SW_PARALLEL, SW_SHORT, LOAD_RESISTOR_RESISTANCE);
float voltageDividerFactor = (BASE_RESISTANCE + LOAD_RESISTOR_RESISTANCE) / BASE_RESISTANCE;

//Liquid Crystal Display
LiquidCrystal_I2C lcd(0x3f, 20, 4);

//Coefficients for wind speed calibration
float A;
float B;
float C;

//Other global variables
unsigned long previousMillis = 0;
float currentRPM = 0.0;
int currentPitch = 0.0;
float currentWindSpeed = 0.0;
float currentPower = 0.0;
String brakeState = "Dis";
String powerSource = "Int";
float kp = ;
float kd = ;
float previousTime = 0;

void setup() {
  //Start up the Serial Monitor
  //For the entry into the Serial Monitor, make sure the box next to the baud rate reads "Newline"
  //and not "Both NL & CR" or anything else. Otherwise, text entry will not work, and no testing
  //state transitions can occur
  Serial.begin(9600);
  
  //Set up the pins, LCD, servos, and pressure sensor (functions below)
  SetupPins();
  Serial.println("Pins set up");
  SetupLCD();
  Serial.println("LCD set up");
  lcd.setCursor(0, 0);
  lcd.print("Starting up...");
  SetupServos();
  Serial.println("Servo set up");
  SetupPressureSensor();
  Serial.println("Pressure sensor set up");
  SetupLoad();
  Serial.println("Load set up");

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
  if(currentMillis - previousMillis >= UPDATE_INTERVAL) {
    //Update the wind speed, RPM, and power
    Serial.println("Starting wind speed reading...");
    currentWindSpeed = ReadWindSpeed();
    Serial.println("Finished wind speed reading...");
    currentRPM = ReadRPM();
//    currentPower = CalculatePower();

    //Write to the LCD and reset the value of the previous writing time
    WriteToLCD();  
    previousMillis = currentMillis;
  }

  
  //Check to see if there is input. Print it out if input has been entered
  if(Serial.available()){
    String input = Serial.readStringUntil('\n');
    Serial.println(input);

    //If the input was "test", enter the test state
    if(input.equals("test")) {
      Serial.println("Entering test state");
      operatingState = test;
    }
  }

  //Operating state transitions
  switch(operatingState) {

    case restart:
    {
      //If the load is disconnected or the e-stop button is pressed, emergency stop
      if(!IsLoadConnected() || IsButtonPressed()) {
        operatingState = emergency_stop;
      }

      else if(ReadRPM() > 0) {
        operatingState = power_curve;
      }

//      //If the wind speed gets above cut-in speed, change to power curve state
//      else if(currentWindSpeed >= CUT_IN) {
//        operatingState = power_curve;
//      }
      
      break;
    }

    case power_curve:
    {
      //If the load is disconnected or the e-stop button is pressed, emergency stop
      if(!IsLoadConnected() || IsButtonPressed()) {
        operatingState = emergency_stop;
      }

      else if(ReadRPM() > RATED_RPM) {
        operatingState = steady_power;
      }

//      //If the wind speed gets above the rated speed, change to steady power state
//      else if(currentWindSpeed >= RATED_WIND_SPEED) {
//        operatingState = steady_power;
          previousTime = millis();
//      }
      
      break;
    }

    case steady_power:
    {
      //If the load is disconnected or the e-stop button is pressed, emergency stop
      if(!IsLoadConnected() || IsButtonPressed()) {
        operatingState = emergency_stop;
      }

      else if(currentPitch >= SURVIVAL_PITCH) {
        operatingState = survival;
      }

//      //If the wind speed gets above the cut-out speed, change to survival state
//      else if(currentWindSpeed >= CUT_OUT) {
//        operatingState = survival;
//      }
      
      break;
    }

    case survival:
    {
      //If the load is disconnected or the e-stop button is pressed, emergency stop
      if(!IsLoadConnected() || IsButtonPressed()) {
        operatingState = emergency_stop;
      }
      
      break;
    }

    case emergency_stop:
    {
      //If the load is disconnected or the e-stop button is pressed, emergency stop
      if(IsLoadConnected() && !IsButtonPressed()) {
        operatingState = restart;
      }
      
      break;
    }

    case test:
    {
      //Set the "testing" boolean to true to enter the second state machine
      testing = true;
      break;
    }
  }

  //Operating state actions
  switch(operatingState) {
    
    case restart:
    {
      //Disengage the brake, return to initial pitch if low wind speed
      SetBrake(BRAKE_DISENGAGED);
      if(currentWindSpeed < CUT_IN) {
        SetPitch(INITIAL_PITCH);
      }
      break;
    }
      
    case power_curve:
    {
      //Adjust the load to match best value based on the wind speed
      break;
    }
    
    case steady_power:
    {
      //Adjust the pitch to keep the RPMs of the motor consistent
      float sigma = 0.05;

      float currentTime = millis();

      float Ts = (currentTime - previousTime) / 1000;

      previousTime = currentTime;'

      float beta = (
      
      float previousPitch = currentPitch;

      float error = ReadRPM() - RATED_RPM;
      
      float newPitch = kp * error - kd * derivative;
      SetPitch(newPitch);
      break;
    }
      
    case survival:
    {
      //Pitch the blades out of the wind completely
      SetPitch(MINIMUM_PITCH);
      break;
    }
      
    case emergency_stop:
    {
      //Brake and pitch out of the wind
      SetBrake(BRAKE_ENGAGED);
      SetPitch(MINIMUM_PITCH);
      break;
    }
    
    case test:
    {
      //State actions occur in testing state machine
      break;
    }
  }

  //This is the testing state machine. It should only execute if we are actively testing
  if(testing) {
    //Write current values to the LCD
    WriteToLCD();
    //Testing state transitions
    switch(testState) {
      
      case test_select:
      {
        //Output formatting for readability
        Serial.println("----------------------------------------------------------------------------");
        //Select a test to run
        SelectTest();
        break;
      }

      case power_select:
      {
        //Return to test selection after test is run
        testState = test_select;
        break;
      }
        
      case brake:
      {
        //Return to test selection after test is run
        testState = test_select;
        break;
      }

      case pitch:
      {     
        //Return to test selection after test is run
        testState = test_select;
        break;
      }

      case pitot_tube_calibration:
      {
        //Return to test selection after test is run
        testState = test_select;
        break;
      }
     
      case pitot_tube_measurement:
      {
        //Return to test selection after test is run
        testState = test_select;
        break;
      }

      case read_base_resistor_voltage:
      {
        //Return to test selection after test is run
        testState = test_select;
        break;
      }

      case read_power:
      {
        //Return to test selection after test is run
        testState = test_select;
        break;
      }

      case read_rpm:
      {      
        //Return to test selection after test is run
        testState = test_select;
        break;
      }

      case set_load:
      {
        //Return to test selection after test is run
        testState = test_select;
        break;
      }

      case tune_load:
      {
        //Return to test selection after test is run
        testState = test_select;
        break;
      }

      case short_load:
      {
        //Return to test selection after test is run
        testState = test_select;
        break;
      }

      case read_load:
      {
        //Return to test selection after test is run
        testState = test_select;
        break;
      }
     
      case emergency_button:
      {
        //Return to test selection after test is run
        testState = test_select;
        break;
      }

      case load_disconnect:
      {
        //Return to test selection after test is run
        testState = test_select;
        break;
      }
      
      case survival_test:
      {
        //Return to test selection after test is run
        testState = test_select;
        break;
      }
     
      case steady_power_test:
      {
        //Return to test selection after test is run
        testState = test_select;
        break;
      }
     
      case power_curve_test:
      {
        //Return to test selection after test is run
        testState = test_select;
        break;
      }
     
      case exit_test:
      {
        //Return to the operating state machine in the restart state
        Serial.println("Leaving test state");
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
        //Get input from the serial monitor based on the prompt
        Serial.print("Select the desired power source. Type e to power externally (from the wall), ");
        Serial.println("i to power internally (from the turbine).");
        while(!Serial.available()) {
        }
        String powerInput = Serial.readStringUntil('\n');
        bool setExternal;

        //If the input is "i", set the power mode to internal
        if(powerInput.equals("i")) {
          setExternal = false;
          SetPowerMode(setExternal);
        }

        //If the input is "e", set the power mode to external
        else if(powerInput.equals("e")) {
          setExternal = true;
          SetPowerMode(setExternal);
        }

        //If no entry is detected, return to test selection
        else {
          Serial.println("Invalid entry. Returning to test selection...");
        }
        break;
      }

      //Activate and deactivate the brake using the linear actuator
      case brake:
      {
        //Get input from the serial monitor based on the prompt
        Serial.println("Select brake setting. Type e for engaged, d for disengaged");
        while(!Serial.available()) {
        }
        String brakeInput = Serial.readStringUntil('\n');

        //If the input is "e", engage the brake
        if(brakeInput.equals("e")) {
          SetBrake(BRAKE_ENGAGED);
        }

        //If the input "d", disengage the brake
        else if(brakeInput.equals("d")) {
          SetBrake(BRAKE_DISENGAGED);
        }

        //If the input is anything else, return to the testing state
        else {
          Serial.println("Invalid entry. Returning to test selection...");
        }
        break;
      }

      //Change the pitch angle of the blades using the linear actuator
      case pitch:
      {
        //Run this test until the user manually exits by trapping in the while loop
        while(true) {
          //Write out info to the LCD display
          WriteToLCD();
          //Output instructions for entering values for this test
          char message[100];
          sprintf(message, "Select pitch value. Enter an integer value between %d and %d (inclusive). ", 
                  MAXIMUM_PITCH, MINIMUM_PITCH);
          Serial.print(message);
          Serial.println("Enter 0 to exit");

          //Read in the input based on the prompt
          int pitchInput = ReadInputInt();
    
          //This is counterintuitive, but MINIMUM_PITCH is a greater value than MAXIMUM_PITCH
          if(pitchInput <= MINIMUM_PITCH && pitchInput >= MAXIMUM_PITCH) {
            SetPitch(pitchInput);
          }

          //If the input is 0, exit from this test
          else if(pitchInput == 0) {
            break;
          }

          //If the input is invalid, read in a new input
          else {
            Serial.println("Invalid entry.");
          }
        }

        Serial.println("Returning to test selection...");
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
        //Read in the wind speed
        float windSpeed = ReadWindSpeed();

        //Output the wind speed read
        Serial.print("Current wind speed: ");
        Serial.print(windSpeed);
        Serial.println(" m/s");
        break;
      }

      //Read the voltage across the resistor
      case read_base_resistor_voltage:
      {
        //Read in the voltage
        int voltageInt = analogRead(LOAD_VOLTAGE);
        float baseResistorVoltage = (voltageInt / ANALOG_RANGE) * OPERATING_VOLTAGE;

        //Output the voltage read
        Serial.print("Current base resistor voltage: ");
        Serial.print(voltageInt);
        Serial.println(" V");
        break;
      }

      //Read the current power output from the turbine
      case read_power:
      {
        //Calculate the current power output
        float power = CalculatePower();

        //Output the power calculated
        Serial.print("Current power output: ");
        Serial.print(power);
        Serial.println(" W");
        break;
      }

      //Read the current rpm from the encoder
      case read_rpm:
      {
        //Read in the current RPM
        float rpm = ReadRPM();

        //Output the current RPM
        Serial.print("Current RPM: ");
        Serial.print(rpm);
        Serial.println(" RPM");
        break;
      }

      case set_load:
      {
        //Get input from the serial monitor based on the prompt
        Serial.println("Enter a resistance value between 0 and 440 Ohms");
        while(!Serial.available()) {
        }
        String input = Serial.readStringUntil('\n');
        float resistance = input.toFloat();

        //Set the load to the input value
        if(resistance >= MINIMUM_RESISTANCE && resistance <= MAXIMUM_RESISTANCE) {
          SetLoad(resistance);
        }

        //If the output is invalid, return to the test state
        else {
          Serial.println("Invalid entry.");
          break;
        }

        //Print out the resistance value
        char message[100];
        sprintf(message, "Load resistance set to closest match of %.2f Ohms", resistance);
        Serial.println(message);
        break;
      }

      case tune_load:
      {
        //Run this test until the user exits
        while(true) {
          //Write data to the LCD
          WriteToLCD();
          //Get input from the serial monitor based on the prompt
          Serial.print("Type + to increment the load and - to decrement the load. ");
          Serial.println("Type exit to leave load tuning");
          //Wait to get input
          while(!Serial.available()) {
          }

          String input = Serial.readStringUntil('\n');
          
          //If the input is "+"
          if(input.equals("+")) {
            //The value 1 inrements the load
            resistorBank.changeResistance(1);
            Serial.println("Load incremented");
  
            //Print out the current load resistance
            float resistance = resistorBank.getResistance();
            Serial.print("Current load resistance is ");
            Serial.println(resistance);
          }

          //If the input is "-"
          else if(input.equals("-")) {
            //The value 0 decrements the load
            resistorBank.changeResistance(0);
            Serial.println("Load decremented");
  
            //Print out the current load resistance
            float resistance = resistorBank.getResistance();
            Serial.print("Current load resistance is ");
            Serial.println(resistance);
          }

          //If the input is exit, leave the test
          else if(input.equals("exit")) {
            break;
          }

          //If any other input is detected, read new input
          else {
            Serial.println("Invalid entry.");
          }
        }

        Serial.println("Returning to test selection...");
        break;
      }

      case short_load:
      {
        Serial.println("Type s to short the load and u to unshort the load");
        
        //Wait to get input
        while(!Serial.available());

        String shortInput = Serial.readStringUntil('\n');

        if(shortInput.equals("s")){
          resistorBank.shortLoad(LOAD_SHORTED);
          Serial.println("Load shorted.");
        }
        else if(shortInput.equals("u")) {
          resistorBank.shortLoad(LOAD_UNSHORTED);
          Serial.println("Load unshorted.");
        }
        else {
          Serial.println("Invalid entry.");
        }
      }

      case read_load:
      {
        //Get the resistance from the load
        float resistance = resistorBank.getResistance();

        //Output the resistance to the serial monitor
        char message[100];
        sprintf(message, "Current load resistance is %.2f", resistance);
        Serial.println(message);
        break;
      }

      //Test emergency stop when the emergency button is pressed
      case emergency_button:
      {
        //Check if the button is pressed
        if(IsButtonPressed()) {
          Serial.println("Emergency stop active. Performing stop procedures.");
          //Engage the brake and pitch the blades out of the wind
          SetBrake(BRAKE_ENGAGED);
          SetPitch(MINIMUM_PITCH);
        }
        else {
          Serial.println("Emergency stop not active.");
        }
        break;
      }

      //Test emergency stop when the load is disconnected
      case load_disconnect:
      {
        //Check if the button is pressed
        if(!IsLoadConnected()) {
          Serial.println("Load disconnected. Performing stop procedures.");
          //Engage the brake and pitch the blades out of the wind
          SetBrake(BRAKE_ENGAGED);
          SetPitch(MINIMUM_PITCH);
        }
        else {
          Serial.println("Load connected.");
        }
        break;
      }

      //Test the survival state
      case survival_test:
      {
         //Pitch blades out of the wind
        SetPitch(MINIMUM_PITCH);
        //Short the Motor
        resistorBank.shortLoad(LOAD_SHORTED);
        //TODO: Ask Power Team about using this function, how do we short the motor
        //SetLoad(SHORTING_RESISTANCE);
        
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
    windSpeedArray[i] = InputWindSpeed();

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
    Serial.print("Pressure Sum: ");
    Serial.println(pressureSum);
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
  
  return (float) windSpeed;
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
    Serial.println("WARNING: Non-integer input detected. If you put in a zero intentionally, disregard.");
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

  return abs(rpm);
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

  if(pow(B, 2) - 4*A*(C - pressureAverage)<0){
    Serial.println("error reading wind speed, negative number in sqrt");
  }
  else if (A == 0){
    Serial.println("error reading wind speed, divide by zero, A = 0");
  }
  else{
    Serial.println("We good as gravy ;)");
  }
  float windSpeed = (-B + sqrt(pow(B, 2) - 4*A*(C - pressureAverage))) / (2*A);
  
  return windSpeed;
}

//Selects a test state based on manual user input
void SelectTest() {
  Serial.print("Enter a desired test. Valid options: pwrsrc, brake, pitch, ptcalib, ptread, readpwr, readvolt, readrpm, setload, tuneload, shortload,\n");
  Serial.print("readload, ebutt, loaddis, survival, steadypwr, pwrcurve, exit\n");

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

  if(input.equals("setload")) {
    testState = set_load;
  }

  if(input.equals("tuneload")) {
    testState = tune_load;
  }

  if(input.equals("shortload")) {
    testState = short_load;
  }

  if(input.equals("readload")) {
    testState = read_load;
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
  if(brakePosition == BRAKE_ENGAGED) {
    brakeState = "Eng";
  }
  else {
    brakeState = "Dis";
  }
}

//Sets the pitch linear actuator to a given position
void SetPitch(int pitchAngle){
  //We might need to flip a switch to turn on the actuator

  //Set the servo to the desired angle
  pitchActuator.writeMicroseconds(pitchAngle);
  currentPitch = pitchAngle;
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
    powerSource = "Ext";
  }
  else {
    digitalWrite(NACELLE_RELAY, HIGH);
    digitalWrite(LOAD_BOX_RELAY, LOW);
    powerSource = "Int";
  }
}

//Sets up the LCD screen
void SetupLCD() {
  //Initialize the LCD screen and backlight it
  lcd.init();
  lcd.backlight();
}

//Sets up the relay load box
void SetupLoad() {
  //Make sure the load isn't shorted
  resistorBank.shortLoad(LOAD_UNSHORTED);
  SetLoad(MAXIMUM_RESISTANCE);
}

//Sets up all Arduino pins
void SetupPins() {
  //Set all output pins
  pinMode(BRAKE_CONTROL, OUTPUT);
  pinMode(PITCH_CONTROL, OUTPUT);
  pinMode(E_BUTTON, INPUT_PULLUP);
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
    else if(testState == power_select) {
      lcd.print("power select");
    }
    else if(testState == brake) {
      lcd.print("brake test");
    }
    else if(testState == pitch) {
      lcd.print("pitch test");
    }
    else if(testState == pitot_tube_calibration) {
      lcd.print("pt calib");
    }
    else if(testState == pitot_tube_measurement) {
      lcd.print("pt measure");
    }
    else if(testState == read_base_resistor_voltage) {
      lcd.print("read voltage");
    }
    else if(testState == read_power) {
      lcd.print("read power");
    }
    else if(testState == read_rpm) {
      lcd.print("read rpm");
    }
    else if(testState == set_load) {
      lcd.print("set load");
    }
    else if(testState == tune_load) {
      lcd.print("tune load");
    }
    else if(testState == short_load) {
      lcd.print("short load");
    }
    else if(testState == read_load) {
      lcd.print("read load");
    }
    else if(testState == emergency_button) {
      lcd.print("e button");
    }
    else if(testState == load_disconnect) {
      lcd.print("load discon");
    }
    else if(testState == survival_test) {
      lcd.print("survival test");
    }
    else if(testState == steady_power_test) {
      lcd.print("stdy pwr test");
    }
    else if(testState == power_curve_test) {
      lcd.print("pwr crve test");
    }
    else if(testState == exit_test) {
      lcd.print("exiting test ");
    }
  }
  //Write out the wind speed (U)
  lcd.setCursor(0, 1);
  lcd.print("U: ");
  lcd.print(currentWindSpeed);

  //Write out the RPM
  lcd.setCursor(10, 1);
  lcd.print("RPM: ");
  int RPM = (int) currentRPM;
  lcd.print(RPM);

  //Write out the pitch "angle"
  lcd.setCursor(0, 2);
  lcd.print((char)224);
  lcd.print(": ");
  lcd.print(currentPitch);

  //Write out the brake state
  lcd.setCursor(10, 2);
  lcd.print("Brake: ");
  lcd.print(brakeState);

  //Write out the power
  lcd.setCursor(0, 3);
  lcd.print("Pwr: ");
  lcd.print(currentPower);

  //Write out the current power source
  lcd.setCursor(10, 3);
  lcd.print("Src: ");
  lcd.print(powerSource);
}
