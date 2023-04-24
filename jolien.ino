#include <WiFiClientSecure.h>
#include <Arduino.h>
#include <TimeLib.h>
#include "esp_wifi.h"
#include "esp_bt.h"
#include "CardReader.h"
#include "SoundHandler.h"
#include "ConfigHandler.h"
#include <WiFi.h>
#include <WiFiClient.h>
#include "Timer.h"

// // easy format 2 min
// #define RECORD_TIME_MS (120000) // ( 2 * 60 * 1000 )
// // easy format 8 min
// #define SLEEP_TIME_MS (480000) //( 8 * 60 * 1000 )

// // easy format 2 min
#define RECORD_TIME_MS (120000) // ( 1 * 60 * 1000 )
// easy format 10 sec
#define SLEEP_TIME_MS (10000) //( 1 * 10 * 1000 )

// adresses for the eeprom
#define TIME_ADDRESS 0

#define TIME_HEADER "T" // Header tag for serial time sync message
#define TIME_REQUEST 7  // ASCII bell character requests a time sync message

// Setup handlers
CardHandler cardHandler = CardHandler();
SoundHandler soundHandler = SoundHandler();
ConfigHandler configHandler = ConfigHandler(cardHandler);
WiFiClientSecure client;

// depending on the state activate the bleutooth for the app or run in recording mode
#define STATE_SETUP 0
#define STATE_RECORDING 2
#define STATE_IDLE 3

int state = STATE_SETUP;

const char *ssid = "tracker-hotspot";
const char *password = "love-you";
const char *server = "gull.purr.dev"; // Server URL
const char *purr_ca =
    "-----BEGIN CERTIFICATE-----\n"
    "MIIFJDCCBAygAwIBAgISBCwxu/lBFOgF/ydGslJ4zcQBMA0GCSqGSIb3DQEBCwUA\n"
    "MDIxCzAJBgNVBAYTAlVTMRYwFAYDVQQKEw1MZXQncyBFbmNyeXB0MQswCQYDVQQD\n"
    "EwJSMzAeFw0yMzAyMTcxNDExNTVaFw0yMzA1MTgxNDExNTRaMBUxEzARBgNVBAMM\n"
    "CioucHVyci5kZXYwggEiMA0GCSqGSIb3DQEBAQUAA4IBDwAwggEKAoIBAQCqfyNd\n"
    "zrAxhG7zDGP73Ezd+4nn7xatoFdOxecR4WfnpvDJGTz1BBuvlaXyMPem5k35OC3q\n"
    "SRYOLvMNTQJZce7+A82O3uMTDCPQvkV3UNbulYksWuCuf97HZpxWqkc/s2bFzEn8\n"
    "YSNWf447N21WMsRMTuIDf5fPybbbttVt7BXM/Vi5YlsLAmblgutqqQabUkmz71UT\n"
    "6CGsV2WRkM/xHfPDrKDqPME05s+T1Lh/hZcnZR+d94qmwhWbgrEDT6gMPkGstzmd\n"
    "IgrjNF/CiThWXMNgen6kfu5NEG2aOkoNtkB/7dTva1x14SX0IIz1UWBFsZD7T8NY\n"
    "kXUFmVdnFx/gQ0A7AgMBAAGjggJPMIICSzAOBgNVHQ8BAf8EBAMCBaAwHQYDVR0l\n"
    "BBYwFAYIKwYBBQUHAwEGCCsGAQUFBwMCMAwGA1UdEwEB/wQCMAAwHQYDVR0OBBYE\n"
    "FNd6To+GEHIyUlXEJiNLgEem19B7MB8GA1UdIwQYMBaAFBQusxe3WFbLrlAJQOYf\n"
    "r52LFMLGMFUGCCsGAQUFBwEBBEkwRzAhBggrBgEFBQcwAYYVaHR0cDovL3IzLm8u\n"
    "bGVuY3Iub3JnMCIGCCsGAQUFBzAChhZodHRwOi8vcjMuaS5sZW5jci5vcmcvMB8G\n"
    "A1UdEQQYMBaCCioucHVyci5kZXaCCHB1cnIuZGV2MEwGA1UdIARFMEMwCAYGZ4EM\n"
    "AQIBMDcGCysGAQQBgt8TAQEBMCgwJgYIKwYBBQUHAgEWGmh0dHA6Ly9jcHMubGV0\n"
    "c2VuY3J5cHQub3JnMIIBBAYKKwYBBAHWeQIEAgSB9QSB8gDwAHUAtz77JN+cTbp1\n"
    "8jnFulj0bF38Qs96nzXEnh0JgSXttJkAAAGGX+6TFwAABAMARjBEAiBA505CJPnA\n"
    "+2E+vg/epefMfRkWH6fj+KN7hzes5shJxAIgapGSmtyTLLK/XrviZV6uBXehVAHJ\n"
    "Gf6Gs7kSDKf+kYsAdwB6MoxU2LcttiDqOOBSHumEFnAyE4VNO9IrwTpXo1LrUgAA\n"
    "AYZf7pM2AAAEAwBIMEYCIQCNUK/S39QcwCZHUtleWMV3NJ66gAxFBKXmCjhB+6we\n"
    "lgIhAOav/N/uOrMux45jsNdadgGUqy7FECF7aUgM2QjRVqmAMA0GCSqGSIb3DQEB\n"
    "CwUAA4IBAQAdH/CCaOT+7vrAd8+TAlt6Zd/Ad8f5xuqyXTDhpvQsreSDqGbKtD9h\n"
    "ZbgdhGzmFNlE/T/T4bboby+G3Ttw+32dSy/8bFbQHFnqyy3HZS09lkhhUDYUGh77\n"
    "vzqDyp6YK5N7YsRDNHoKo0UxZXPi5qSxv8P2Gq/XIbltXtEjnvEcBVrCTV1vQ+Xg\n"
    "vB5Hgcbqp/xGUx9dmGbAfMHiMfpM6xt49NDo4hKiyVMDWlSjNVNVJrLgXIdKXEtC\n"
    "XYN50u+l/lGmOKvCKZXOQEa6erTekAmnvfZSq+8Ekf+S1PvJcFWBL/NAUq6hSih2\n"
    "zj87Fd/khOMK80Fbbv3BURBd/O7GnFhD\n"
    "-----END CERTIFICATE-----\n";

void setup()
{
  Serial.begin(115200);

  while (!Serial)
  {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");

  // Print the local IP address
  Serial.print("Local IP address: ");
  Serial.println(WiFi.localIP());

  // print the gateway IP address
  Serial.print("Gateway IP address: ");
  Serial.println(WiFi.gatewayIP());

  // print the subnet mask
  Serial.print("Subnet mask: ");
  Serial.println(WiFi.subnetMask());

  client.setCACert(purr_ca);
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");

  // disable bluetooth
  esp_bt_controller_disable();

  // Add some delay so we can see the serial output
  delay(1000);

  // try
  // {
  //   // initialize card reader
  //   cardHandler.init();

  //   Serial.println("Card reader initialized");

  //   // read the config file
  //   configHandler.init();

  //   Serial.println("Config file initialized");

  //   if (configHandler.config.time == 0)
  //   {
  //     // set time to 12:00:00 1.1.2020 if not set
  //     setTime(12, 0, 0, 1, 1, 2020);
  //   }
  //   else
  //   {
  //     // set the time
  //     setTime(configHandler.config.time);
  //   }

  //   Serial.print("Loaded time: ");
  //   printClock(configHandler.config.time);

  //   // initialize sound handler
  //   soundHandler.init();
  // }
  // catch (const std::exception &e)
  // {
  //   Serial.println(e.what());
  //   while (true)
  //   {
  //     delay(1000);
  //   }
  // }
}

void loop()
{

  switch (state)
  {
  case STATE_IDLE:
    // ping the api every minute
    if (second() == 0)
    {
      pingApi();
    }

    break;

  case STATE_RECORDING:
    // recordingLoop();
    goToSleep();
    break;
  case STATE_SETUP:
  default:
    // set state to recording
    state = STATE_IDLE;
    break;
  }
}

void recordingLoop()
{
  uint32_t recordingStartTime = millis();
  uint32_t recordingEndTime = recordingStartTime + RECORD_TIME_MS;
  uint32_t recordingProgress = 0;

  char filename[32];
  sprintf(filename, "/%04d-%02d-%02d_%02d-%02d-%02d.wav", year(), month(), day(), hour(), minute(), second());

  Serial.print("Recording to file: ");
  Serial.println(filename);

  file_t file = cardHandler.getFile(filename);

  // if file size is 0, add header
  if (file.size() == 0)
  {
    // Get the WAV header
    wav_header_t wavh = soundHandler.getWavHeader();

    // Write the WAV header to the file
    file.write((uint8_t *)&wavh, sizeof(wavh));
  }

  // Record audio samples to the file
  while (millis() < recordingEndTime)
  {
    // Allocate a buffer for the samples
    int16_t buffer[I2S_READ_LEN * I2S_NUM_CHANNELS];

    size_t bytesRead = soundHandler.i2sRead(buffer, sizeof(buffer));

    // Write the samples to the file
    file.write((uint8_t *)buffer, bytesRead);

    // print progress every 10 seconds
    if (millis() - recordingProgress > 10000)
    {
      recordingProgress = millis();
      Serial.print("Recording progress: ");
      Serial.print((recordingProgress - recordingStartTime) / 1000);
      Serial.print(" seconds");
      // print percentage
      Serial.print(" (");
      Serial.print((recordingProgress - recordingStartTime) * 100 / RECORD_TIME_MS);
      Serial.println("%)");
    }
  }
  // Close the file
  file.close();
}

void goToSleep()
{

  Serial.println("Recording done");

  delay(1000);
  Serial.print("Going to sleep for ");
  Serial.print(SLEEP_TIME_MS / 1000);
  Serial.println(" seconds...");

  // get the time in seconds
  long time = now();

  // add the sleep time to the time
  time += SLEEP_TIME_MS / 1000;

  // write the time to the eeprom
  configHandler.config.time = time;
  configHandler.saveConfig();

  // go to sleep
  esp_sleep_enable_timer_wakeup(SLEEP_TIME_MS * 1000);
  esp_deep_sleep_start();
  Serial.println("Done");
}

void printClock(long time)
{
  // digital clock display of the time
  Serial.print(hour(time));
  printDigits(minute(time));
  printDigits(second(time));
  Serial.print(" ");
  Serial.print(day(time));
  Serial.print(".");
  Serial.print(month(time));
  Serial.print(".");
  Serial.print(year(time));
  Serial.println();
}

void printDigits(int digits)
{
  // utility function for digital clock display: prints preceding colon and leading 0
  Serial.print(":");
  if (digits < 10)
    Serial.print('0');
  Serial.print(digits);
}

void pingApi()
{
  Serial.println("\nStarting connection to server...");
  if (!client.connect(server, 443))
    Serial.println("Connection failed!");
  else
  {
    Serial.println("Connected to server!");
    // Make a HTTP request:
    client.println("GET http://localhost:3000/api/trackers/5113 HTTP/1.0");
    client.println("Host: gull.purr.dev");
    client.println("Connection: close");
    client.println("Content-Type: application/json");
    client.println("x-api-key: 123456789"); // added header
    client.println();

    while (client.connected())
    {
      String line = client.readStringUntil('\n');
      if (line == "\r")
      {
        Serial.println("headers received");
        break;
      }
    }
    // if there are incoming bytes available
    // from the server, read them and print them:
    while (client.available())
    {
      char c = client.read();
      Serial.write(c);
    }

    client.stop();
  }
}