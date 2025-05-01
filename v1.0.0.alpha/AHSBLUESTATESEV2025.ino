#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <cmath>


// Defining Components ~ I can add timer if I want to
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
// ~ Switch Pin connection
#define SWITCH_PIN 10

// ~ Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET -1

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

const int PWM1 = 5;
const int PWM2 = 6;
const byte encoder = 2;
int pin0 = 1;
int pin1 = 4;
volatile int counter = 0;
unsigned long lasttime;
unsigned long thistime;






void EVCountsMove(float counts) {
  counter = 0;
  while (counts * 0.5 > counter) {
    analogWrite(PWM1, 0);
    analogWrite(PWM2, 0);
    thistime = micros();
    delay(10);
  }
  /*
  while (counts * 0.75 > counter) {
    analogWrite(PWM1, 100);
    analogWrite(PWM2, 100);
    thistime = micros();
    delay(10);
  }*/
  while (counts > counter) {
    analogWrite(PWM1, 175);
    analogWrite(PWM2, 175);
    thistime = micros();
    delay(10);
  }
  analogWrite(PWM1, 255);
  analogWrite(PWM2, 255);
}

void EVBackCountsMove(float speed1, float speed2, float counts) {
  digitalWrite(pin0, HIGH);
  digitalWrite(pin1, LOW);
  counter = 0;
  while (counts > counter) {
    analogWrite(PWM1, speed1);
    analogWrite(PWM2, speed2);
    thistime = micros();
    delay(10);
  }
  thistime = micros();
  analogWrite(PWM1, 255);
  analogWrite(PWM2, 255);
}

void EVBackMove(float speed1, float speed2, float time) {
  digitalWrite(pin0, HIGH);
  digitalWrite(pin1, LOW);
  lasttime = micros();
  thistime = micros();
  while (thistime - lasttime < (time*1000000)) {
    analogWrite(PWM1, speed1);
    analogWrite(PWM2, speed2);
    thistime = micros();
    delay(10);
  }
  thistime = micros();
  analogWrite(PWM1, 255);
  analogWrite(PWM2, 255);
}


void ISR_count()  {
  counter++;  // increment Motor A counter value
} 



void setup() {
  Serial.begin(9600);
  Wire.begin();
  Wire.setClock(400000);
  pinMode(encoder, INPUT_PULLUP);
  pinMode(pin0, OUTPUT);
  pinMode(pin1, OUTPUT);
  pinMode(0,INPUT);
  digitalWrite(pin0, LOW);
  digitalWrite(pin1, HIGH);
  attachInterrupt(digitalPinToInterrupt(encoder), ISR_count, RISING);
  while (digitalRead(0) == LOW) {
    // do nothing
  }
  EVCountsMove(75714);  // 887 per 10 cm, 88.7 per cm
  EVBackMove(150,150,0.62);

}

void loop() {
  // put your main code here, to run repeatedly:
}