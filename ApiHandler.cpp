#include "ApiHandler.h"
#include "Tracker.h"
#include <Arduino.h>

ApiHandler::ApiHandler(const char *apiKey)
    : _apiKey(apiKey)
{
}

void ApiHandler::setEndpoint(const char *server, const char *ssid)
{
  this->_serverPath = String(server) + "/api/trackers/" + String(ssid);
}

void ApiHandler::updateTrackerStatus(const Tracker& tracker)
{
  this->_client.begin(this->_serverPath.c_str());
  this->_client.addHeader("Content-Type", "application/json");
  this->_client.addHeader("x-api-key", this->_apiKey);

  // Prepare payload
  DynamicJsonDocument doc(1024);
  JsonObject trackerObj = doc.createNestedObject("tracker");
  trackerObj["startSync"] = tracker.startSync;
  trackerObj["startLog"] = tracker.startLog;
  String payload;
  serializeJson(doc, payload);

  // Send HTTP PUT request
  int httpResponseCode = this->_client.POST(payload);

  if (httpResponseCode > 0)
  {
    Serial.print("HTTP Response code: ");
    Serial.println(httpResponseCode);
    String response = this->_client.getString();
    Serial.println(response);
  }
  else
  {
    Serial.print("Error code: ");
    Serial.println(httpResponseCode);
  }
  // Free resources
  this->_client.end();
}

void ApiHandler::pingTrackerStatus()
{
  this->_client.begin(this->_serverPath.c_str());
  this->_client.addHeader("Content-Type", "application/json");
  this->_client.addHeader("x-api-key", this->_apiKey);

  // Send HTTP GET request
  int httpResponseCode = this->_client.GET();

  if (httpResponseCode > 0)
  {
    Serial.print("HTTP Response code: ");
    Serial.println(httpResponseCode);
    String payload = this->_client.getString();
    Serial.println(payload);

    // deserialize json
    DynamicJsonDocument doc(1024);
    // payload: {"tracker":{"id":2,"name":"tracker-2","description":null,"nestId":null,"lastHeard":"2023-04-30T15:35:58.641Z","shouldSync":true,"shouldLog":false}}
    deserializeJson(doc, payload);

    // check if sync flag is set

    bool shouldLog = doc["tracker"]["shouldLog"];

    if (shouldLog)
    {
      _shouldLog = true;
    }

    bool shouldSync = doc["tracker"]["shouldSync"];

    if (shouldSync)
    {
      _shouldSync = true;
    }
  }
  else
  {
    Serial.print("Error code: ");
    Serial.println(httpResponseCode);
  }
  // Free resources
  this->_client.end();
}

bool ApiHandler::shouldStartLogging()
{
}

bool ApiHandler::shouldStartSyncing()
{
}
