/*
* Variable Load System (main.ino)
*
* This product was developed by Davi Cavinatto (Power Group) for use 
* in the main turbine control system. It contains 2 public functions
* that can be used to control and monitor the resistance values of
* the Variable Load System.
*
* Version 5.0 - November 2, 2023
* Recent changes
*   This version can select the best match among the 20 possible
*   resistance combinations.
*/

#include <variableLoad.h>
#include <Arduino.h>

// Declare the myResistorBank object as a global varible
// variableLoad objectName(arduino_pins[0-8], single_pwr_resistor_value)
variableLoad myResistorBank(22, 48, 46, 44, 42, 40, 38, 36, 34, 28.0);

void setup(){
  Serial.begin(9600);
}

void loop(){
  // Read current resistance value
  float resistance = myResistorBank.getResistance();
  Serial.println(resistance);
  // Use changeResistance function to increase or decrease resistance value (0 decreases, 1 increases)
  myResistorBank.changeResistance(0);
  delay(5000);
  // Use setBestResistanceMatch function to set resistance value to the closest match
  myResistorBank.setBestResistanceMatch(4.3);
  delay(5000);

  // Read current resistance value
  resistance = myResistorBank.getResistance();
  Serial.println(resistance);

}