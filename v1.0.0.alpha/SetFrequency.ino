#include <Arduino.h>
#include <Wire.h>

const int E1 = 7;
const int E2 = 8;


// Set up the Arduino Due's digital pins D5 and D10 for 15kHz PWM
void setup() {
  PMC->PMC_PCER1 |= PMC_PCER1_PID34 | PMC_PCER1_PID33;    // Enable peripheral TC6 (TC2 Channel 0) and TC7 (TC2 Channel 1)
  PIOC->PIO_ABSR |= PIO_ABSR_P29 | PIO_ABSR_P25;          // Switch the multiplexer to peripheral B for TIOA6 (D5) and TIOB7 (D10)
  PIOC->PIO_PDR |= PIO_PDR_P29 | PIO_PDR_P25;             // Disable the GPIO on the corresponding pins

  TC2->TC_CHANNEL[0].TC_CMR = TC_CMR_ACPC_SET |           // Set TIOA on counter match with RC0                           
                              TC_CMR_ACPA_CLEAR |         // Clear TIOA on counter match with RA0
                              TC_CMR_WAVE |               // Enable wave mode
                              TC_CMR_WAVSEL_UP_RC |       // Count up with automatic trigger on RC compare
                              TC_CMR_EEVT_XC0 |           // Set event selection to XC0 to make TIOB an output
                              TC_CMR_TCCLKS_TIMER_CLOCK1; // Set the timer clock to TCLK1 (MCK/2 = 84MHz/2 = 42MHz)

  TC2->TC_CHANNEL[1].TC_CMR = TC_CMR_BCPC_SET |           // Set TIOB on counter match with RC0                              
                              TC_CMR_BCPB_CLEAR |         // Clear TIOB on counter match with RB0                             
                              TC_CMR_WAVE |               // Enable wave mode
                              TC_CMR_WAVSEL_UP_RC |       // Count up with automatic trigger on RC compare
                              TC_CMR_EEVT_XC0 |           // Set event selection to XC0 to make TIOB an output
                              TC_CMR_TCCLKS_TIMER_CLOCK1; // Set the timer clock to TCLK1 (MCK/2 = 84MHz/2 = 42MHz)

  TC2->TC_CHANNEL[0].TC_RC = 2100;                        // Set the PWM frequency to 15kHz: 42MHz / 20kHz = 2100 
  TC2->TC_CHANNEL[1].TC_RC = 2100;                        // Set the PWM frequency to 15kHz: 42MHz / 20kHz = 2100

  TC2->TC_CHANNEL[0].TC_CCR = TC_CCR_SWTRG | TC_CCR_CLKEN; // Enable the timer TC6 (TC2 Channel 0)
  TC2->TC_CHANNEL[1].TC_CCR = TC_CCR_SWTRG | TC_CCR_CLKEN; // Enable the timer TC7 (TC2 Channel 1)
  
}

void loop() {
  TC2->TC_CHANNEL[0].TC_RA = 525;                         // Set the duty-cycle to 25% on D5
  TC2->TC_CHANNEL[1].TC_RB = 525;                         // Set the duty-cycle to 25% on D10
  delay(2000);                                            // Wait for 2 seconds                                     // Wait for 2 seconds
}
