#pragma once
#include <Arduino.h>
namespace TeensyTimerTool {
enum TimerGenerator { GPT1, GPT2, TMR1 };
struct PeriodicTimer { explicit PeriodicTimer(TimerGenerator); void begin(void (*)(), unsigned long); };
}
