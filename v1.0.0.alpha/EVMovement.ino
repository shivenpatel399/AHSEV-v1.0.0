/*
const int E1 = 7;
const int E2 = 8;

void M1_advance(float Speed) { ///<Motor1 Advance 
  analogWrite(E1,Speed);
}



void stopWait() {
  analogWrite(E1,0);
}

void setup() {
  // put your setup code here, to run once:
  for(int i=3;i<9;i++)
    pinMode(i,OUTPUT);
  for(int i=11;i<13;i++)
    pinMode(i,OUTPUT);
  M1_advance(240);  
  delay(1000);
  M1_advance(150); 
  delay(1000); 
  M1_advance(90); 
  delay(1000);
}

void loop() {
  // put your main code here, to run repeatedly:

}
*/

#include <pwm.h>

const int E1 = 7;

PwmOut pwm(7);

void setup() {
  analogWriteResolution(10);
  pinMode(, OUTPUT);
  analogWrite(E1, 512);
  delay(10);
  pwm.begin(20000, 0);
  pwm.pulse_perc(50.0);
  pinMode(LED_BUILTIN, OUTPUT);

  delay(10);


}

uint8_t loop_count = 0;
void loop() {
  digitalWrite(LED_BUILTIN, (loop_count++ & 1) ? HIGH : LOW);
  delay(500);
}
