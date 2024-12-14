#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_BNO08x.h>
#include <cmath>

const int PWM1 = 4;
const int PWM2 = 6;
const byte enA = 3;
const byte enB = 2;
int bufferA = 5;
int bufferB = 7;
int pin0 = 0;
int pin1 = 1;
volatile int counter_A = 0;
volatile int counter_B = 0;
unsigned long lasttime;
unsigned long thistime;
float thecorrection = 0;
int powerA = 0;
int powerB = 0;

int pulsecheck = 0;
float currentAngle = 0;
float target = 0;
int targetIterator = 0;
float angleone = 0;
float angletwo = 0;

// OLED Conenctions

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define SWITCH_PIN 7
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Gyro Connections

#define BNO08X_CS 10
#define BNO08X_INT 8


// #define FAST_MODE

// For SPI mode, we also need a RESET 
#define BNO08X_RESET 9
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
void EVCountsMove(float speed1, float speed2, float counts) {
  counter_A = 0;
  counter_B = 0;
  while ((counts*2) > counter_A + counter_B) {
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
    digitalWrite(bufferA, LOW);
    digitalWrite(bufferB, LOW);
    analogWrite(PWM1, speed1);
    analogWrite(PWM2, speed2);
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.print(" Counts A ");
    display.println(counter_A);
    display.print(" Counts B ");
    display.println(counter_B);
    display.print(" Expect: ");
    display.println(target);
    display.print(" Currnt: ");
    display.println(currentAngle);
    display.print(" Ang Diff: ");
    display.println(abs(target-currentAngle));
    display.print(" Time: ");
    display.println(abs(thistime-lasttime));
    display.display();
    thistime = micros();
    delay(10);
  }
  thistime = micros();
  digitalWrite(bufferA, HIGH);
  digitalWrite(bufferB, HIGH);
  angleone = currentAngle;
}

void EVBackCountsMove(float speed1, float speed2, float counts) {
  digitalWrite(pin0, HIGH);
  digitalWrite(pin1, LOW);
  counter_A = 0;
  counter_B = 0;
  while ((counts*2) > counter_A + counter_B) {
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
    digitalWrite(bufferA, LOW);
    digitalWrite(bufferB, LOW);
    analogWrite(PWM1, speed1);
    analogWrite(PWM2, speed2);
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.print(" Counts A ");
    display.println(counter_A);
    display.print(" Counts B ");
    display.println(counter_B);
    display.print(" Expect: ");
    display.println(target);
    display.print(" Currnt: ");
    display.println(currentAngle);
    display.print(" Ang Diff: ");
    display.println(abs(target-currentAngle));
    display.print(" Time: ");
    display.println(abs(thistime-lasttime));
    display.display();
    thistime = micros();
    delay(10);
  }
  thistime = micros();
  digitalWrite(bufferA, HIGH);
  digitalWrite(bufferB, HIGH);
  angleone = currentAngle;
}

void EVMove(float speed1, float speed2, float time) {
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
    thecorrection = target - currentAngle;
    digitalWrite(bufferA, LOW);
    digitalWrite(bufferB, LOW);
    analogWrite(PWM1, speed1);
    analogWrite(PWM2, speed2);
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.print(" Counts A ");
    display.println(counter_A);
    display.print(" Counts B ");
    display.println(counter_B);
    display.print(" Expect: ");
    display.println(target);
    display.print(" Currnt: ");
    display.println(currentAngle);
    display.print(" Ang Diff: ");
    display.println(abs(target-currentAngle));
    display.print(" Time: ");
    display.println(abs(thistime-lasttime));
    display.display();
    thistime = micros();
    delay(10);
  }
  thistime = micros();
  digitalWrite(bufferA, HIGH);
  digitalWrite(bufferB, HIGH);
  angleone = currentAngle;
}

void EVBackMove(float speed1, float speed2, float time) {
  digitalWrite(pin0, HIGH);
  digitalWrite(pin1, LOW);
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
    thecorrection = target - currentAngle;
    digitalWrite(bufferA, LOW);
    digitalWrite(bufferB, LOW);
    analogWrite(PWM1, speed1);
    analogWrite(PWM2, speed2);
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.print(" Counts A ");
    display.println(counter_A);
    display.print(" Counts B ");
    display.println(counter_B);
    display.print(" Expect: ");
    display.println(target);
    display.print(" Currnt: ");
    display.println(currentAngle);
    display.print(" Ang Diff: ");
    display.println(abs(target-currentAngle));
    display.print(" Time: ");
    display.println(abs(thistime-lasttime));
    display.display();
    thistime = micros();
    delay(10);
  }
  thistime = micros();
  digitalWrite(bufferA, HIGH);
  digitalWrite(bufferB, HIGH);
  angletwo = currentAngle;
  
}


void ISR_countA()  {
  counter_A++;  // increment Motor A counter value
} 

void ISR_countB()  {
  counter_B++;  // increment Motor B counter value
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Wire.begin();
  Wire.setClock(400000);
  pinMode(enA, INPUT_PULLUP);
  pinMode(enB, INPUT_PULLUP);
  pinMode(bufferA, OUTPUT);
  pinMode(bufferB, OUTPUT);
  pinMode(pin0, OUTPUT);
  pinMode(pin1, OUTPUT);
  digitalWrite(pin0, LOW);
  digitalWrite(pin1, HIGH);
  digitalWrite(bufferA,HIGH);
  digitalWrite(bufferB,HIGH);
  attachInterrupt(digitalPinToInterrupt(enA), ISR_countA, RISING);
  attachInterrupt(digitalPinToInterrupt(enB), ISR_countB, RISING);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;);
  }
  delay(100);
  // Issue Located
  // PWMChannelSetup();
  if (!bno08x.begin_SPI(BNO08X_CS, BNO08X_INT)) {
    while (1) { delay(10); }
  }
  setReports(reportType, reportIntervalUs);
  delay(350);
  targetAngle();
  // PD(1200,0,5);
  // EVMove(0,0,2);
  // EVMove(0,5,1);
  EVCountsMove(0,0,1760); 
  EVBackMove(100,100,0.65);
}

void loop() {
  // put your main code here, to run repeatedly:

}