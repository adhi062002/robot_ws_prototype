// Minimal Arduino/Teensy API shim: lets us compile-check the .ino on a PC (syntax and
// type errors only -- this does not simulate hardware).
#pragma once
#include <math.h>
#include <cmath>
using std::isfinite;
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define HIGH 1
#define LOW 0
#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2
template <class T, class L, class H> T constrain(T x, L lo, H hi) {
  return x < (T)lo ? (T)lo : (x > (T)hi ? (T)hi : x);
}
uint32_t millis(); uint32_t micros();
void pinMode(uint8_t, uint8_t); void digitalWrite(uint8_t, uint8_t); int digitalRead(uint8_t);
void analogWrite(uint8_t, int); int analogRead(uint8_t);
void analogWriteResolution(int); void analogWriteFrequency(uint8_t, uint32_t);
struct HardwareSerial {
  void begin(uint32_t); int available(); int read(); size_t write(const uint8_t*, size_t);
  size_t write(uint8_t); void println(const char*); size_t readBytesUntil(char, char*, size_t);
  void setTimeout(unsigned long);
};
extern HardwareSerial Serial, Serial1;
// ---- Arduino-core conveniences used by the sketches ----
#include <algorithm>
using std::max;
using std::min;
void delay(uint32_t ms);
