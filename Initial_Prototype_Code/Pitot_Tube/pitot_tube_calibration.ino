#include "ms4525do.h"
#include <string.h>
#include <BasicLinearAlgebra.h>

/* 
* An MS4525DO object
*/
bfs::Ms4525do pitotTube;

void setup() {
  /* Serial to display data */
  Serial.begin(9600);
  while(!Serial){}

  Serial.println("Starting new trial...");
  Wire.begin();
  Wire.setClock(400000);
  /* 
  * I2C address of 0x28, on bus 0, with a -1 to +1 PSI range
  */
  pitotTube.Config(&Wire, 0x28, 1.0f, -1.0f);
  /* Starting communication with the pressure transducer */
  if (!pitotTube.Begin()) {
    Serial.println("Error communicating with sensor");
    while(1){}
  }
}

void loop() {
  // Arrays to store the measured pressure values and wind speed calibration values
  float windSpeedArray[3];
  float pressureArray[3];

  // Read 3 wind speeds and pressures to calibrate
  for(int i = 0; i < 3; i++) {
    // Get the wind speed and store in the array
    windSpeedArray[i] = ReadWindSpeed();

    // Average 1000 pressure readings to get a pressure at that wind speed
    float pressureSum = 0.0;
    int j = 0;
    int k = 0;
    for (j = 0; j < 1000; j++) {
      if(pitotTube.Read()) {
        k++;
        float currReading = pitotTube.pres_pa();
        
        pressureSum += currReading;
      }
    }
    float pressureAverage = pressureSum / k;

    // Store the pressure in the array
    pressureArray[i] = pressureAverage;
  }

//  Serial.println("Wind speeds:");
//  for(int i = 0; i < 3; i++) {
//    Serial.println(windSpeedArray[i]);
//  }
//  
//  Serial.println("Pressures:");
//  for(int i = 0; i < 3; i++) {
//    Serial.println(pressureArray[i]);
//  }

  Serial.println("All wind speeds entered. Calibrating...");

  // Matrices to hold the pressures and quadratic fit data. These arrays will be used to solve the matrix problem
  // Ax = B, where A is the quadratic fit matrix based on the wind speeds, x is the values a, b, and c in the quadratic
  // equation y = ax^2 + bx + c, and B is the output pressure values. We will be solving for a, b, and c
  BLA::Matrix<3, 1> pressures;
  pressures = {pressureArray[0], pressureArray[1], pressureArray[2]};
  
  BLA::Matrix<3, 3> quadFit;
  quadFit = {pow(windSpeedArray[0], 2), windSpeedArray[0], 1, pow(windSpeedArray[1], 2), windSpeedArray[1], 1, pow(windSpeedArray[2], 2), windSpeedArray[2], 1};

//  Serial.println("Initial array:");
//  for(int i = 0; i < 3; i++) {
//    for(int j = 0; j < 3; j++) {
//      Serial.println(quadFit(i, j));
//    }
//  }

  // Invert the quadratic fit matrix, solve for a, b, and c, and store the values
  BLA::Invert(quadFit);

//  Serial.println("Inverse array:");
//  for(int i = 0; i < 3; i++) {
//    for(int j = 0; j < 3; j++) {
//      Serial.println(quadFit(i, j));
//    }
//  }

  BLA::Matrix<3, 1> abc = quadFit * pressures;
  
  float a = ABC(0, 0);
  float b = ABC(1, 0);
  float c = ABC(2, 0);

//  Serial.println(A);
//  Serial.println(B);
//  Serial.println(C);

  while(true) {

    // Average 1000 measurements of the wind speed
    float pressureSum = 0.0;
    int i = 0;
    int j = 0;
    for (i = 0; i < 1000; i++) {
      if(pitotTube.Read()) {
        j++;
        float currReading = pitotTube.pres_pa();
        
        pressureSum += currReading;
      }
    }
    float pressureAverage = pressureSum / j;

    // The values a, b, and c are in the equation y = ax^2 + bx + c, where y is the pressure and x is
    // the wind speed. We can measure the pressure, but we need to solve for the wind speed. This uses
    // the quadratic equation to solve for the wind speed.
    float windSpeed = (-b + sqrt(pow(b, 2) - 4*a*(c - pressureAverage))) / (2*a);

    Serial.print("Current wind speed: ");
    Serial.println(windSpeed);
    
    delay(1000);
  }
}

float ReadWindSpeed() {
  Serial.println("Please enter a new wind speed");

  // Wait to receive user input
  while(!Serial.available()) {
  }

  String inputString = "";

  // Read in all entered values and store them as a string
  while(Serial.available()) {
    // Read values until a newline is read, which signifies the end of the input.
    while(true) {
      char c = Serial.read();
      if(c == '\n') {
        break;
      }
//      Serial.print("Read character! ");
      Serial.println(c);
      if(isDigit(c) || c == '.') {
      inputString += c;
      }
    }
    
  }

  // Convert the entered wind speed to a float and return it
  float windSpeed = 0.0;

  if(inputString.length() > 0) {
    windSpeed = inputString.toFloat();
  }

  Serial.print("Wind speed of ");
  Serial.print(windSpeed, 1);
  Serial.println(" entered.");
  
  return windSpeed;
}
