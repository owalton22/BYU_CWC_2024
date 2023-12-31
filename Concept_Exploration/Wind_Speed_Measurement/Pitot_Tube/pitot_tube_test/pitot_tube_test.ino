/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2022 Bolder Flight Systems
* 
* Permission is hereby granted, free of charge, to any person obtaining a copy of this software 
* and associated documentation files (the "Software"), to deal in the Software without restriction, 
* including without limitation the rights to use, copy, modify, merge, publish, distribute, 
* sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is 
* furnished to do so, subject to the following conditions:
* 
* The above copyright notice and this permission notice shall be included in all copies or 
* substantial portions of the Software.
* 
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING 
* BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
* NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, 
* DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

float ReadWindSpeed();

#include "ms4525do.h"

/* 
* An MS4525DO object
*/
bfs::Ms4525do pres;

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
  pres.Config(&Wire, 0x28, 1.0f, -1.0f);
  /* Starting communication with the pressure transducer */
  if (!pres.Begin()) {
    Serial.println("Error communicating with sensor");
    while(1){}
  }
}

void loop() {
  int count = 0;
  /* Read the sensor */
  float a = 0;
  float b = 0;
  float c = 0;

  float windSpeedArray[3];
  
  for(int i = 0; i < 3; i++) {
    windSpeedArray[i] = ReadWindSpeed();
  }

  

  
//  float sum = 0;
//  while (count < 50) {
//    if (pres.Read()) {
//      /* Display the data */
//      if (count > 0) {
//        sum += pres.pres_pa();
//      }
//      Serial.print(pres.pres_pa(), 6);
//      Serial.print("\n");
//    }
//    delay(100);
//    count+=1;
//  }
//  float mean = sum / (count - 1);
//  Serial.print("Mean: ");
//  Serial.println(mean);
//  delay(100000000);
}

float ReadWindSpeed() {
  Serial.println("Please set a new wind speed");
  Serial.println("Enter new wind speed");

  float windSpeed = 4.0;
  return windSpeed;
}
