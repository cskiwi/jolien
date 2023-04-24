#ifndef TIMER_H
#define TIMER_H

#include <functional>

class Timer {
  public:
    Timer(unsigned long interval, std::function<void()> callback);
    void loop();
    void reset();

  private:
    unsigned long _interval;
    std::function<void()> _callback;
    unsigned long _previousMillis;
};

#endif
