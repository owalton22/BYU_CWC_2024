#define PWM_PIN 3

void setup() {
  // put your setup code here, to run once:
  pinMode(PWM_PIN, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  for(int i = 25; i < 50; i++) {
    analogWrite(PWM_PIN, i);
    delay(100);
  }
}
