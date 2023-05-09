#ifndef TRACKER_H
#define TRACKER_H

#include <Arduino.h>
#include <TimeLib.h>

class Tracker
{
public:
  Tracker()
  {
    this->id = 0;
    this->name = "";
    this->description = "";
    this->nestId = 0;
    this->lastHeard = 0;
    this->shouldSync = false;
    this->shouldLog = false;
    this->startedLogOn = 0;
    this->startedSyncOn = 0;
  }

  Tracker(int id, String name, String description, int nestId, time_t lastHeard, bool shouldSync, bool shouldLog, time_t startedLogOn, time_t startedSyncOn)
  {
    this->id = id;
    this->name = name;
    this->description = description;
    this->nestId = nestId;
    this->lastHeard = lastHeard;
    this->shouldSync = shouldSync;
    this->shouldLog = shouldLog;
    this->startedLogOn = startedLogOn;
    this->startedSyncOn = startedSyncOn;
  }

  int id;
  String name;
  String description;
  int nestId;
  time_t lastHeard;
  bool shouldSync;
  bool shouldLog;
  time_t startedLogOn;
  time_t startedSyncOn;
};

#endif