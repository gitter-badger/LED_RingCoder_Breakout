
#include <Bounce2.h>
#include <Metro.h>

#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>

#define LED_COUNT 16
#define ENCODER_STEP 4 //When you feel a single turn, its actually 4 signals
#define ENCODER_RANGE LED_COUNT
#define ENCODER_MAX (ENCODER_RANGE - 1)

#define KNOB_LED_RANGE 256
#define KNOB_LED_MAX (KNOB_LED_RANGE - 1)

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

long encoderPosition = 0;

enum ledCounter {RED = 0, BLUE = 1, GREEN = 2, NONE = 3};
byte ledCount = RED;
byte ledValue[3] = {KNOB_LED_MAX, KNOB_LED_MAX, KNOB_LED_MAX};
byte ledPins[3] = {redPin, bluPin, grnPin};
unsigned int currentShift = 0x0000;

Encoder myEnc(bPin, aPin);
Bounce pushButton = Bounce();
Metro timer = Metro(1000);

inline int positive_modulo(int i, int n) {
  return (i % n + n) % n;
}

void setPushButtonPins() {
  // setup switch pins, set as an input, no pulled up
  pinMode(swhPin, INPUT);
  digitalWrite(swhPin, LOW);  // Disable internal pull-up
  pushButton.attach(swhPin);
  pushButton.interval(5);
}

void setLedPins() {
  // Setup led pins as outputs, and write their intial value.
  // initial value is defined by the ledValue global variable
  pinMode(redPin, OUTPUT);
  analogWrite(redPin, ledValue[RED]);  // Red off
  pinMode(bluPin, OUTPUT);
  analogWrite(bluPin, ledValue[BLUE]);  // Blue off
  pinMode(grnPin, OUTPUT);
  analogWrite(grnPin, ledValue[GREEN]);  // Blue off
}

void setShiftRegisterPins() {
  // Setup shift register pins
  pinMode(enPin, OUTPUT);  // Enable, active low, this'll always be LOW
  digitalWrite(enPin, LOW);  // Turn all outputs on
  pinMode(latchPin, OUTPUT);  // this must be set before calling shiftOut16()
  digitalWrite(latchPin, LOW);  // start latch low
  pinMode(clkPin, OUTPUT);  // we'll control this in shiftOut16()
  digitalWrite(clkPin, LOW);  // start sck low
  pinMode(clrPin, OUTPUT);  // master clear, this'll always be HIGH
  digitalWrite(clrPin, HIGH);  // disable master clear
  pinMode(datPin, OUTPUT);  // we'll control this in shiftOut16()
  digitalWrite(datPin, LOW);  // start ser low
}

void blink(int singleLED) {
  unsigned int shift = 0x0000;
  unsigned int offShift = 0xFFFF;
  if (singleLED > -1) {
    shift = currentShift | calculateShift(false, singleLED);
    offShift = currentShift;
  }
  setShift(shift);
  delay(250);
  setShift(offShift);
  delay(250);
  setShift(currentShift);
}

void blink() {
  blink(-1);
}

void spin(boolean reverse) {
  for(unsigned int i = 0; i < LED_COUNT; i++) {
    if (reverse) {
      setShift(32768 >> i); //32768 has highest bit (msb) 1 and all others 0
    } else {
      setShift(1 << i);
    }
    delay(1000/LED_COUNT);
  }
  setShift(0x0000);
}

void spin() {
  spin(false);
}

void reverse_spin() {
  spin(true);
}

//Read/Write methods to adjust for hardware only settling on multiples of 4
signed int readEncoder() {
  return (myEnc.read() / ENCODER_STEP);
}

void writeEncoder(signed int newPosition) {
  myEnc.write(newPosition * ENCODER_STEP);
}

void setup() {
  Serial.begin(9600);
  setPushButtonPins();
  setLedPins();
  setShiftRegisterPins();

  setShift(0x0000);

  spin();
  reverse_spin();

  expirationAnimation();
}

void setKnobRgb(int r, int g, int b) {
  int safe_r = positive_modulo(r, KNOB_LED_RANGE);
  int safe_g = positive_modulo(g, KNOB_LED_RANGE);
  int safe_b = positive_modulo(b, KNOB_LED_RANGE);
  //LEDs use 255-0 range

  analogWrite(ledPins[RED], KNOB_LED_MAX - safe_r);
  analogWrite(ledPins[GREEN], KNOB_LED_MAX - safe_g);
  analogWrite(ledPins[BLUE], KNOB_LED_MAX - safe_b);
}

void expirationAnimation() {
  Serial.println("Times up!");

  setKnobRgb(255, 0, 0);
  delay(333);
  setKnobRgb(0, 255, 0);
  delay(333);
  setKnobRgb(0, 0, 255);
  delay(333);
  setKnobRgb(255, 255, 255);
}

int secondsRemaining = 0;
bool timerRunning = false;

void loop() {
  signed int newPosition;
  boolean stateChanged = pushButton.update();
  int state = pushButton.read();
  byte elapsedSecond = timer.check();

  if (timerRunning) {
    if (stateChanged && state == HIGH) {
      timerRunning = false;
      Serial.println("Stop timer (sec): " + String(secondsRemaining));
    }
    if (elapsedSecond) {
      secondsRemaining--;
      blink(secondsRemaining / MINUTE);
      //Check timer completion
      if (secondsRemaining <= 0) {
        timerRunning = false;
        expirationAnimation();
      }
    }
  } else {
    //Ajust timer
    newPosition = positive_modulo(readEncoder(), ENCODER_RANGE);
    if (newPosition != encoderPosition) {
      encoderPosition = newPosition;
      secondsRemaining = encoderPosition * MINUTE;
      Serial.println("Set timer (sec): " + String(secondsRemaining));
    }

    if (stateChanged && state == HIGH && secondsRemaining > 0) { //Start timer
      Serial.println("Start timer (sec): " + String(secondsRemaining));
      timerRunning = true;
    }
  }

  ledRingFiller();  // Update the bar graph LED
}

void setShift(unsigned int ledOutput) {
  currentShift = ledOutput;
  digitalWrite(latchPin, LOW);  // first send latch low
  shiftOut16(ledOutput);  // send the ledOutput value to shiftOut16
  digitalWrite(latchPin, HIGH);  // send latch high to indicate data is done sending
}

unsigned int calculateShift(boolean fill) {
  return calculateShift(fill, secondsRemaining / MINUTE);
}

unsigned int calculateShift(boolean fill, int pos) {
  unsigned int ledOutput = 0;

  if (fill) {
    for (int i = 0; i < pos; i++){
      bitSet(ledOutput, i);
    }
  } else {
    bitSet(ledOutput, pos);
  }

  return ledOutput;
}

void ledRingFiller() {
  unsigned int ledOutput = calculateShift(true);
  setShift(ledOutput);
}

void ledRingFollower() {
  unsigned int ledOutput = calculateShift(false);
  setShift(ledOutput);
}

void shiftOut16(uint16_t data) {
  // Isolate the MSB and LSB
  byte datamsb = (data & 0xFF00) >> 8;  // mask out the MSB and shift it right 8 bits
  byte datalsb = (data & 0x00FF);  // Mask out the LSB

  // First shift out the MSB, MSB first.
  shiftOut(datPin, clkPin, MSBFIRST, datamsb);
  // Then shift out the LSB
  shiftOut(datPin, clkPin, MSBFIRST, datalsb);
}

