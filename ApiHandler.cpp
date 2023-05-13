#include "ApiHandler.h"
#include "Tracker.h"
#include <Arduino.h>

ApiHandler::ApiHandler(const char *apiKey)
    : _apiKey(apiKey)
{
}

void ApiHandler::setEndpoint(const char *server, String ssid)
{
  this->_serverPath = String(server) + "/api/trackers/" + ssid;
}

void ApiHandler::updateTrackerStatus(const Tracker &tracker)
{
  this->_client.begin(this->_serverPath.c_str());
  this->_client.addHeader("Content-Type", "application/json");
  this->_client.addHeader("x-api-key", this->_apiKey);

  Serial.print("Adding api key: ");
  Serial.println(this->_apiKey);

  Serial.print("URL: ");
  Serial.println(this->_serverPath);

  // Prepare payload
  DynamicJsonDocument doc(1024);
  JsonObject trackerObj = doc.createNestedObject("tracker");
  trackerObj["startedSyncOn"] = tracker.startedSyncOn;
  trackerObj["startedLogOn"] = tracker.startedLogOn;
  trackerObj["shouldLog"] = tracker.shouldLog;
  trackerObj["shouldSync"] = tracker.shouldSync;
  String payload;
  serializeJson(doc, payload);

  Serial.print("Payload POST: ");
  Serial.println(payload);

  // Send HTTP PUT request
  int httpResponseCode = this->_client.POST(payload);

  if (httpResponseCode > 0)
  {
    Serial.print("HTTP Response code: ");
    Serial.println(httpResponseCode);
    String response = this->_client.getString();
    Serial.print("Result POST: ");
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

// ping and update tracker instance
void ApiHandler::pingTrackerStatus(Tracker &tracker)
{
  this->_client.begin(this->_serverPath.c_str());
  this->_client.addHeader("Content-Type", "application/json");
  this->_client.addHeader("x-api-key", this->_apiKey);

  Serial.print("Adding api key: ");
  Serial.println(this->_apiKey);

  Serial.print("URL: ");
  Serial.println(this->_serverPath);

  // Send HTTP GET request
  int httpResponseCode = this->_client.GET();

  if (httpResponseCode > 0)
  {
    Serial.print("HTTP Response code: ");
    Serial.println(httpResponseCode);
    String payload = this->_client.getString();
    Serial.print("Result GET: ");
    Serial.println(payload);

    // deserialize json
    DynamicJsonDocument doc(1024);
    deserializeJson(doc, payload);

    // the payload should contain the tracker info
    JsonObject trackerObject = doc["tracker"];
    tracker.id = trackerObject["id"].as<int>();
    tracker.name = trackerObject["name"].as<String>();
    tracker.description = trackerObject["description"].as<String>();
    tracker.nestId = trackerObject["nestId"].as<int>();
    tracker.shouldSync = trackerObject["shouldSync"].as<bool>();
    tracker.shouldLog = trackerObject["shouldLog"].as<bool>();
  }
  else
  {
    Serial.print("Error code: ");
    Serial.println(httpResponseCode);
  }

  // Free resources
  this->_client.end();
}
