#include <Encoder.h>

#define ENCODER_PIN_1 2
#define ENCODER_PIN_2 3

Encoder encoder(ENCODER_PIN_1, ENCODER_PIN_2);

int numSamples = 100;

float averageRPM;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  float RPMSum = 0.0;
  
  // put your main code here, to run repeatedly:
  for(int i = 0; i < numSamples; i++) {
    float RPM = ReadRPM();


    RPMSum += RPM;

  }

  averageRPM = RPMSum / numSamples;

  Serial.println(averageRPM);
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
