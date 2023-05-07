#ifndef APIHANDLER_H
#define APIHANDLER_H

#include <Arduino.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include "Tracker.h"

class ApiHandler
{
public:
  ApiHandler(const char *apiKey);
  void setEndpoint(const char *server, String ssid);
  void updateTrackerStatus(const Tracker& tracker);
  void pingTrackerStatus();
  bool shouldStartLogging();
  bool shouldStartSyncing();

private:
  const char *_apiKey;
  String _serverPath;
  HTTPClient _client;

  bool _shouldSync;
  bool _shouldLog;
};

#endif
