#pragma once
#include <Arduino.h>
struct Servo { int id_ = 0; void attach(int); void write(int); int read(); };
