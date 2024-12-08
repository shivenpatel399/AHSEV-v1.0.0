#include "pwm.h"

PwmOut pwm(D4);

void setup() {
  //period 50us = 20000hz; pulse 0 us = 0%
  pwm.begin(20000.0f, 0.0f);

  // set 100%
  pwm.pulse_perc(100.0f);

}

void loop() {
  
}
