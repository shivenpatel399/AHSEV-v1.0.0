#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_BNO08x.h>
#include <cmath>

// This is just to remind myself that I have to connect the speed pins to these 2

const int E1 = 5; /// Motor1 Speed

/* For Directional Control, its always moving forward. Therefore, we already put one of our white wires into Ground to Reset it to LOW setting */

// Encoder Pins
const byte enA = 2; /// Motor 1 Encoder (D5 Motor) and Right


// Counters
volatile int counter_A = 0;

// Time
unsigned long lasttime;
unsigned long thistime;
// Vehicle Variables

float stepcount = 176.00;
float circumference = 15.394;

float powerA = 0;

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


struct euler_t {
  float yaw;
  float pitch;
  float roll;
} ypr;



// Encoder Functions

void ISR_countA()  {
  counter_A++;  // increment Motor A counter value
} 


// PWM Frequency Functions

// Void Setup

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Wire.begin();
  Wire.setClock(400000);
  pinMode(enA, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(enA), ISR_countA, RISING); 
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;);
  }
  delay(100);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("Welcome Shiven!"));
  display.println(F("Code will run shortly..."));
  display.display();
  delay(2500);
  

}

void loop() {
  // put your main code here, to run repeatedly:
  analogWrite(E1, 125);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print(" Count A: ");
  display.println(counter_A);
  display.display();
}
