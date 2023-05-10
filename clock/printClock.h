#ifndef PRINTCLOCK_H
#define PRINTCLOCK_H

#include <Arduino.h>

void printClock(uint64_t epochTime);
void printClock(uint64_t epochTime, bool lineEnd);
void printDigits(int digits);

#endif
