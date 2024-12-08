#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_BNO08x.h>
#include <cmath>

// This is just to remind myself that I have to connect the speed pins to these 2

const int E1 = 5; /// Motor1 Speed
const int E2 = 10; /// Motor2 Speed

/* For Directional Control, its always moving forward. Therefore, we already put one of our white wires into Ground to Reset it to LOW setting */

// Encoder Pins
const byte enA = 2; /// Motor 1 Encoder (D5 Motor) and Right
const byte enB = 3; /// Motor 2 Encoder (D10 Motor) and Left


// Counters
volatile int counter_A = 0;
volatile int counter_B = 0;

// PID Speeds
int powerLeft = 0;
int powerRight = 0;

// PID Variables

float derivative = 0;
float integral = 0;
float total_e = 0;
float e = 0;
float lastError = 0;

float Kproportional = 0;
float Kintegral = 0;
float Kderivative = 0;

float thecorrection = 0;
float correction = 0;

double kpGain = 0;
double kiGain = 0;
double kdGain = 0;
float Gain = 0;
float gain = 0;

unsigned long lasttime;
unsigned long thistime;
// Vehicle Variables

float stepcount = 176.00;
float circumference = 15.394;

float powerA = 0;
float powerB = 0;

int pulsecheck = 0;
float currentAngle = 0;
float target = 0;
int targetIterator = 0;

// OLED Conenctions

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define SWITCH_PIN 7
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Gyro Connections

#define BNO08X_CS 34
#define BNO08X_INT 30


// #define FAST_MODE

// For SPI mode, we also need a RESET 
#define BNO08X_RESET 32
// but not for I2C or UART
// #define BNO08X_RESET -1

struct euler_t {
  float yaw;
  float pitch;
  float roll;
} ypr;

Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

// Gyro Functions

#ifdef FAST_MODE
  // Top frequency is reported to be 1000Hz (but freq is somewhat variable)
  sh2_SensorId_t reportType = SH2_GYRO_INTEGRATED_RV;
  long reportIntervalUs = 2000;
#else
  // Top frequency is about 250Hz but this report is more accurate
  sh2_SensorId_t reportType = SH2_ARVR_STABILIZED_RV;
  long reportIntervalUs = 5000;
#endif
void setReports(sh2_SensorId_t reportType, long report_interval) {
  Serial.println("Setting desired reports");
  if (! bno08x.enableReport(reportType, report_interval)) {
    Serial.println("Could not enable stabilized remote vector");
  }
}

void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t* ypr, bool degrees = false) {

    float sqr = sq(qr);
    float sqi = sq(qi);
    float sqj = sq(qj);
    float sqk = sq(qk);

    ypr->yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
    ypr->pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
    ypr->roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));

    if (degrees) {
      ypr->yaw *= RAD_TO_DEG;
      ypr->pitch *= RAD_TO_DEG;
      ypr->roll *= RAD_TO_DEG;
    }
}

void quaternionToEulerRV(sh2_RotationVectorWAcc_t* rotational_vector, euler_t* ypr, bool degrees = false) {
    quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

void quaternionToEulerGI(sh2_GyroIntegratedRV_t* rotational_vector, euler_t* ypr, bool degrees = false) {
    quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

void targetAngle() {
  for (targetIterator = 0; targetIterator < 9; targetIterator++) {
    if (bno08x.wasReset()) {
      setReports(reportType, reportIntervalUs);
    }
  
    if (bno08x.getSensorEvent(&sensorValue)) {
      // in this demo only one report type will be received depending on FAST_MODE define (above)
      switch (sensorValue.sensorId) {
        case SH2_ARVR_STABILIZED_RV:
          quaternionToEulerRV(&sensorValue.un.arvrStabilizedRV, &ypr, true);
        case SH2_GYRO_INTEGRATED_RV:
          // faster (more noise?)
          quaternionToEulerGI(&sensorValue.un.gyroIntegratedRV, &ypr, true);
          break;
      }
    }
    target = ypr.yaw;
  }
}

void AngleMeasure(int time) {
  lasttime = micros();
  thistime = micros();
  while (thistime - lasttime < (time*1000000)) {
    if (bno08x.wasReset()) {
      setReports(reportType, reportIntervalUs);
    }
  
    if (bno08x.getSensorEvent(&sensorValue)) {
      // in this demo only one report type will be received depending on FAST_MODE define (above)
      switch (sensorValue.sensorId) {
        case SH2_ARVR_STABILIZED_RV:
          quaternionToEulerRV(&sensorValue.un.arvrStabilizedRV, &ypr, true);
        case SH2_GYRO_INTEGRATED_RV:
          // faster (more noise?)
          quaternionToEulerGI(&sensorValue.un.gyroIntegratedRV, &ypr, true);
          break;
      }
    }
    currentAngle = float(ypr.yaw);
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.print(" Angle: ");
    display.println(currentAngle);
    display.print(" Time: ");
    display.println(abs(thistime-lasttime));
    display.display();
    thistime = micros();
    delay(10);
  }
  thistime = micros();
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print(" Angle: ");
  display.println(currentAngle);
  display.print(" Time: ");
  display.println(abs(thistime-lasttime));
  display.display();
}


// Encoder Functions

void ISR_countA()  {
  counter_A++;  // increment Motor A counter value
} 

void ISR_countB()  {
  counter_B++;  // increment Motor B counter value
}

int CMtoSteps(float cm) {

  int result;  // Final calculation result. 
  float rotations = cm / circumference;  // total rotations
  
  float f_result = rotations *  stepcount; // Calculate result as a float
  result = (int) f_result; // Convert to an integer (note this is NOT rounded)
  pulsecheck = result;
  
  return result;  // End and return result

}

// PWM Frequency Functions
void PWMChannelSetup() {
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

void PWMASetup(float percent) {
  TC2->TC_CHANNEL[0].TC_RA = 2100 * (percent/100);
}

void PWMBSetup(float percent) {
  TC2->TC_CHANNEL[1].TC_RB = 2100 * (percent/100);
}

void enACounts(float counts) {
  counter_A = 0;
  while (counter_A < counts) {
    PWMASetup(90);
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.print(" Counter A: ");
    display.println(counter_A);
    display.print(" Task: ");
    display.println("In Progress");
    display.display();
  }
  PWMASetup(100);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print(" Counter A: ");
  display.println(counter_A);
  display.print(" Task: ");
  display.println("Done");
  display.display();
}

void enBCounts(float counts) {
  counter_B = 0;
  while (counter_B < counts) {
    PWMBSetup(90);
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.print(" Counter B: ");
    display.println(counter_B);
    display.print(" Task: ");
    display.println("In Progress");
    display.display();
  }
  PWMBSetup(100);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print(" Counter B: ");
  display.println(counter_B);
  display.print(" Task: ");
  display.println("Done");
  display.display();
}
void PDTIME(int time, int power, float gained) {
  counter_A = 0;
  counter_B = 0;
  targetAngle();
  lasttime = micros();
  thistime = micros();
  while (thistime - lasttime < time) {
    if (bno08x.wasReset()) {
      setReports(reportType, reportIntervalUs);
    }
  
    if (bno08x.getSensorEvent(&sensorValue)) {
      // in this demo only one report type will be received depending on FAST_MODE define (above)
      switch (sensorValue.sensorId) {
        case SH2_ARVR_STABILIZED_RV:
          quaternionToEulerRV(&sensorValue.un.arvrStabilizedRV, &ypr, true);
        case SH2_GYRO_INTEGRATED_RV:
          // faster (more noise?)
          quaternionToEulerGI(&sensorValue.un.gyroIntegratedRV, &ypr, true);
          break;
      }
    }
    currentAngle = float(ypr.yaw);
    thecorrection = target - currentAngle;
    powerA = power + (gained * thecorrection);
    powerB = power - (gained * thecorrection);
    PWMASetup(powerA);
    PWMBSetup(powerB);
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.print(" Expect: ");
    display.println(target);
    display.print(" Currnt: ");
    display.println(currentAngle);
    display.print(" PowerA: ");
    display.println(powerA);
    display.print(" PowerB: ");
    display.println(powerB);
    display.print(" Time: ");
    display.println(abs(thistime-lasttime));
    display.print(" Ang Diff: ");
    display.println(abs(target-currentAngle));
    display.display();
    thistime = micros();
	  delay(10);
    } 
  thistime = micros();
  PWMASetup(0);
  PWMBSetup(0);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print(" Expect: ");
  display.println(target);
  display.print(" Currnt: ");
  display.println(currentAngle);
  display.print(" PowerA: ");
  display.println(powerA);
  display.print(" PowerB: ");
  display.println(powerB);
  display.print(" Ang Diff: ");
  display.println(abs(target-currentAngle));
  display.print(" Time: ");
  display.println(abs(thistime-lasttime));
  display.display();
}

void PD(int thesteps, int percent, float gained) {
  counter_A = 0;
  counter_B = 0;
  targetAngle();
  lasttime = micros();
  while (counter_A + counter_B < (thesteps * 2)) {
    if (bno08x.wasReset()) {
      setReports(reportType, reportIntervalUs);
    }
    if (bno08x.getSensorEvent(&sensorValue)) {
      // in this demo only one report type will be received depending on FAST_MODE define (above)
      switch (sensorValue.sensorId) {
        case SH2_ARVR_STABILIZED_RV:
          quaternionToEulerRV(&sensorValue.un.arvrStabilizedRV, &ypr, true);
        case SH2_GYRO_INTEGRATED_RV:
          // faster (more noise?)
          quaternionToEulerGI(&sensorValue.un.gyroIntegratedRV, &ypr, true);
          break;
      }
    }
    currentAngle = float(ypr.yaw);
    thecorrection = target - currentAngle;
    powerA = percent - (gained * thecorrection);
    powerB = percent + (gained * thecorrection);
    PWMASetup(powerA);
    PWMBSetup(powerB);
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.print(" Expect: ");
    display.println(target);
    display.print(" Currnt: ");
    display.println(currentAngle);
    display.print(" PowerA: ");
    display.println(powerA);
    display.print(" PowerB: ");
    display.println(powerB);
    display.print(" Pulses: ");
    display.println(thesteps);
    display.print(" Count A: ");
    display.println(counter_A);
    display.print(" Count B: ");
    display.println(counter_B);
    display.print(" Ang Diff: ");
    display.println(abs(target-currentAngle));
    display.display();
	  delay(10);
    } 
  thistime = micros();
  PWMASetup(100);
  PWMBSetup(100);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print(" Expect: ");
  display.println(target);
  display.print(" Currnt: ");
  display.println(currentAngle);
  display.print(" PowerA: ");
  display.println(powerA);
  display.print(" PowerB: ");
  display.println(powerB);
  display.print(" Pulses: ");
  display.println(thesteps);
  display.print(" Count A: ");
  display.println(counter_A);
  display.print(" Count B: ");
  display.println(counter_B);
  display.print(" Ang Diff: ");
  display.println(abs(target-currentAngle));
  display.print(" Time: ");
  display.println(abs(thistime-lasttime));
  display.display();
}
// Void Setup

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Wire.begin();
  Wire.setClock(400000);
  attachInterrupt(digitalPinToInterrupt(enA), ISR_countA, RISING); 
  attachInterrupt(digitalPinToInterrupt(enB), ISR_countB, RISING);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;);
  }
  delay(100);
  pinMode(enA, INPUT_PULLUP);
  pinMode(enB, INPUT_PULLUP);
  PWMChannelSetup();
  if (!bno08x.begin_SPI(BNO08X_CS, BNO08X_INT)) {
    while (1) { delay(10); }
  }
  setReports(reportType, reportIntervalUs);
  delay(100);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("Welcome Shiven!"));
  display.println(F("Code will run shortly..."));
  display.display();
  delay(2500);
  /*
  PWMASetup(1); // Right
  PWMBSetup(15); // Left
  delay(2000);
  PWMASetup(75);
  PWMBSetup(75);
  delay(1500);
  PWMASetup(100);
  PWMBSetup(100);
  delay(250);*/
  // PD(180,50,5.13);

}

void loop() {
  // put your main code here, to run repeatedly:
  PWMASetup(50);
  PWMBSetup(50);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print(" Count A: ");
  display.println(counter_A);
  display.print(" Count B: ");
  display.println(counter_B);
  display.display();
}
