#include <Arduino.h>
#include "Timer.h"

Timer::Timer(unsigned long interval, std::function<void()> callback)
  : _interval(interval), _callback(callback), _previousMillis(0) {}

void Timer::loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - _previousMillis >= _interval) {
    _previousMillis = currentMillis;
    _callback();
  }
}

void Timer::reset() {
  _previousMillis = millis();
}
