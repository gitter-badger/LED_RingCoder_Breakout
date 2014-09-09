
#include <Metro.h>

#include <Bounce2.h>
#include <Encoder.h>
#include "RingCoder.h"

#define MINUTE 60

// Pin definitions - Encoder:
const int bPin = 0;  // Encoder B pin, D2 is external interrupt 0
const int aPin = 1;  // Encoder A pin, D3 is external interrupt 1
const int redPin = 3;  // Encoder's red LED - D5 is PWM enabled
const int bluPin = 4;  // Encoder's blue LED- D6 is PWM enabled
const int grnPin = 5;  // Encoder's green LED - D9 is PWM enabled
const int swhPin = 2;  // Encoder's switch pin

// Pin definitions - Shift registers:
const int datPin = 6;  // shift registers' SER pin
const int clrPin = 7;  // shift registers' srclr pin
const int clkPin = 8;  // Shift registers' srclk pin
const int latchPin = 9;  // Shift registers' rclk pin
const int enPin = 10;  // Shift registers' Output Enable pin

RingCoder ringcoder = RingCoder(bPin, aPin, redPin, bluPin, grnPin, swhPin, datPin, clrPin, clkPin, latchPin, enPin);

Metro timer = Metro(1000);

void setup() {
  Serial.begin(9600);
  ringcoder.spin();
  ringcoder.reverse_spin();

  expirationAnimation();
}

void expirationAnimation() {
  Serial.println("Times up!");

  ringcoder.setKnobRgb(255, 0, 0);
  delay(333);
  ringcoder.setKnobRgb(0, 255, 0);
  delay(333);
  ringcoder.setKnobRgb(0, 0, 255);
  delay(333);
  ringcoder.setKnobRgb(255, 255, 255);
}

int secondsRemaining = 0;
bool timerRunning = false;

void loop() {
  signed int newPosition;
  bool stateChanged = ringcoder.update();
  int state = ringcoder.button();
  byte elapsedSecond = timer.check();

  if (timerRunning) {
    if (stateChanged && state == HIGH) {
      timerRunning = false;
      Serial.println("Stop timer (sec): " + String(secondsRemaining));
    }
    if (elapsedSecond) {
      secondsRemaining--;
      ringcoder.blink(secondsRemaining / MINUTE);
      //Check timer completion
      if (secondsRemaining <= 0) {
        timerRunning = false;
        expirationAnimation();
      }
    }
  } else {
    //Ajust timer
    int pos = ringcoder.readEncoder();
    if (ringcoder.moved()) {
      secondsRemaining = pos * MINUTE;
      Serial.println("Set timer (sec): " + String(secondsRemaining));
    }
    if (stateChanged && state == HIGH && secondsRemaining > 0) { //Start timer
      Serial.println("Start timer (sec): " + String(secondsRemaining));
      timerRunning = true;
    }
  }

  ringcoder.ledRingFiller();  // Update the bar graph LED
}

