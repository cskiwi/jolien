#include "printClock.h"
#include <Arduino.h>
#include <TimeLib.h>

void printClock(uint64_t epochTime)
{
  printClock(epochTime, true);
}

void printClock(uint64_t epochTime, bool lineEnd)
{
  // digital clock display of the time
  Serial.print(hour(epochTime));
  printDigits(minute(epochTime));
  printDigits(second(epochTime));
  Serial.print(' ');

  // print date
  Serial.print(day(epochTime));
  Serial.print(".");
  Serial.print(month(epochTime));
  Serial.print(".");
  Serial.print(year(epochTime));

  // print epoch time
  Serial.print(" (");
  Serial.print(epochTime);

  if (lineEnd)
  {
    Serial.println(")");
  }
  else
  {
    Serial.print(')');
  }
}


void printDigits(int digits)
{
  // utility function for digital clock display: prints preceding colon and leading 0
  Serial.print(":");
  if (digits < 10)
    Serial.print('0');
  Serial.print(digits);
}
