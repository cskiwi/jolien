#ifndef TRACKER_H
#define TRACKER_H

#include <Arduino.h>
#include <TimeLib.h>

class Tracker
{
public:
  int id;
  String name;
  String description;
  time_t lastHeard;
  bool shouldSync;
  bool shouldLog;

  time_t startSync;
  time_t startLog;
};

#endif