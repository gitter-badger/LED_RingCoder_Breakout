
#include <Bounce2.h>
#include <EEPROM.h>

#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>

#define EEPROM_MAX 2047 //teensy 3.1
#define LED_COUNT 16
#define ENCODER_STEP 4 //When you feel a single turn, its actually 4 signals
#define ENCODER_RANGE 32
#define ENCODER_MAX (ENCODER_RANGE - 1)

#define KNOB_LED_RANGE 256
#define KNOB_LED_MAX (KNOB_LED_RANGE - 1)

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

long encoderPosition;

enum ledCounter {RED = 0, BLUE = 1, GREEN = 2, NONE = 3};
byte ledCount = RED;
byte ledValue[3] = {KNOB_LED_MAX, KNOB_LED_MAX, KNOB_LED_MAX};
byte ledPins[3] = {redPin, bluPin, grnPin};

Encoder myEnc(bPin, aPin);
Bounce pushButton = Bounce();

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


void blink() {
  setShift(0x0000);
  delay(250);
  setShift(0xFFFF);
  delay(250);
  setShift(0x0000);
}

void spin(boolean reverse) {
  for(unsigned int i = 0; i < LED_COUNT; i++) {
    if (reverse) {
      setShift(32768 >> i); //32768 is the highest bit (msb) one and all others 0
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
}

void loop() {
  signed int newPosition;
  boolean stateChanged = pushButton.update();
  int state = pushButton.read();

  if (stateChanged && state == HIGH) {
    ledCount = ++ledCount % (NONE + 1);

    if (ledCount == NONE) {
      setShift(0x0000);
      checkValues();
    } else {
      //set to the encoder value from the last time we adjusted that LED
      writeEncoder(
        ledValue[ledCount]  / KNOB_LED_RANGE / ENCODER_RANGE
      );
      Serial.print("loading former value ");
      Serial.print(myEnc.read());
      Serial.print(" From stored LED value ");
      Serial.print(ledValue[ledCount]);
      Serial.println("");
    }

    Serial.println("ledCount: " + String(ledCount));
  }

  //Read position after stateChanged conditional to allow for myEnc.write
  newPosition = positive_modulo(readEncoder(), ENCODER_RANGE);

  if (newPosition != encoderPosition || stateChanged) {
    encoderPosition = newPosition;
    Serial.println("Encoder position: " + String(encoderPosition));

    if (ledCount != NONE) {  // Only update the LED if it's RED, GREEN or BLUE
      ledRingFollower();  // Update the bar graph LED
      ledValue[ledCount] = KNOB_LED_MAX * encoderPosition / ENCODER_MAX;
      analogWrite(ledPins[ledCount], KNOB_LED_MAX - ledValue[ledCount]);//LEDs use 255-0 range
      Serial.println("Setting LED to " + String(ledValue[ledCount]));
    }
  }
}

void checkValues() {
  int r = EEPROM.read(RED);
  int g = EEPROM.read(GREEN);
  int b = EEPROM.read(BLUE);

  if (r == ledValue[RED] &&
      g == ledValue[GREEN] &&
      b == ledValue[BLUE]) {
    Keyboard.print("Secret Word");
  } else {
    Serial.print("(");
    Serial.print(ledValue[RED]);
    Serial.print(",");
    Serial.print(ledValue[GREEN]);
    Serial.print(",");
    Serial.print(ledValue[BLUE]);
    Serial.print(")");
    Serial.println("");
    blink();
    blink();
    blink();
  }
}

void setShift(unsigned int ledOutput) {
  digitalWrite(latchPin, LOW);  // first send latch low
  shiftOut16(ledOutput);  // send the ledOutput value to shiftOut16
  digitalWrite(latchPin, HIGH);  // send latch high to indicate data is done sending
}

unsigned int calculateShift(boolean fill) {
  unsigned int ledShift = 0;
  unsigned int ledOutput = 0;

  //Convert from encoderPosition to bit string
  ledShift = encoderPosition / (ENCODER_RANGE / LED_COUNT);

  if (fill) {
    for (int i = 0; i < ledShift; i++){
      bitSet(ledOutput, i);
    }
  } else {
    bitSet(ledOutput, ledShift);
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

