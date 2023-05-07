#include <Arduino.h>
#include <TimeLib.h>
#include "esp_wifi.h"
#include "esp_bt.h"
#include "CardReader.h"
#include "SoundHandler.h"
#include "ConfigHandler.h"
#include "ApiHandler.h"
#include "Tracker.h"
#include <WiFi.h>
#include "Timer.h"
#include <NTPClient.h>
#include <WiFiUdp.h>

// // easy format 2 min
// #define RECORD_TIME_MS (120000) // ( 2 * 60 * 1000 )
// // easy format 8 min
// #define SLEEP_TIME_MS (480000) //( 8 * 60 * 1000 )

// // easy format 2 min
#define RECORD_TIME_US (1 * 60 * 1000 * 1000) // ( 1 * 60 * 1000 )
// easy format 1 min
#define SLEEP_TIME_US (1 * 60 * 1000 * 1000) //( 1 *  60* 1000 )

const char *ssid = "tracker-hotspot";
const char *password = "love-you";
// const char *server = "https://gull.purr.dev"; // Server URL
const char *server = "http://192.168.1.253:3000"; // Server URL
const char *apiKey = "123456789";                 // Your API key

// Setup handlers
CardHandler cardHandler = CardHandler();
SoundHandler soundHandler = SoundHandler();
ConfigHandler configHandler = ConfigHandler(cardHandler);
ApiHandler apiHandler(apiKey);

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org");

// tracker info
Tracker tracker = Tracker();

// depending on the state activate the bleutooth for the app or run in recording mode
#define STATE_SETUP 0
#define STATE_RECORDING 2
#define STATE_IDLE 3

int state = STATE_SETUP;
bool pingingApi = false;
bool initalStartup = true;
uint32_t Freq = 0;

//
void setup()
{
  Serial.begin(115200);

  while (!Serial)
  {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  // lower the cpu frequency to save power
  setCpuFrequencyMhz(80);

  Freq = getCpuFrequencyMhz();
  Serial.print("CPU Freq = ");
  Serial.print(Freq);
  Serial.println(" MHz");
  Freq = getXtalFrequencyMhz();
  Serial.print("XTAL Freq = ");
  Serial.print(Freq);
  Serial.println(" MHz");
  Freq = getApbFrequency();
  Serial.print("APB Freq = ");
  Serial.print(Freq);
  Serial.println(" Hz");

  // give some breath room for the trackers
  delay(1000);
  connectToWifi();
  // disable bluetooth
  esp_bt_controller_disable();

  // Add some delay so we can see the serial output
  try
  {

    // initialize card reader
    cardHandler.init();
    Serial.println("Card reader initialized");

    // read the config file
    configHandler.init();

    // start by setting the time from the config
    setTime(configHandler.config.time);

    // set tracker info
    tracker.name = configHandler.config.ssid;

    // initialize the time client
    timeClient.begin();
    timeClient.setTimeOffset(3600 * 2);
    timeClient.update();

    // wait for the time to be set
    delay(1000);

    // print the time
    Serial.print("Time: ");
    printClock(timeClient.getEpochTime());

    //  save the config
    configHandler.saveConfig();

    // initialize api handler
    apiHandler.setEndpoint(server, tracker.name);

    // // initialize sound handler
    // soundHandler.init();
  }
  catch (const std::exception &e)
  {
    Serial.println(e.what());
    while (true)
    {
      delay(1000);
    }
  }

  // print clear message that setup is done
  Serial.println("");
  Serial.println("");
  Serial.println("");
  Serial.println("===================");
  Serial.println("Setup done");
  Serial.println("===================");
  Serial.println("");
  Serial.println("");
  Serial.println("");
}

void loop()
{
  try
  {

    switch (state)
    {
    case STATE_IDLE:
      // ping the api every minute
      if (second() == 0 || initalStartup)
      {
        initalStartup = false;
        apiHandler.pingTrackerStatus();

        Serial.println("Checkinf if logging");

        if (apiHandler.shouldStartLogging())
        {
          Serial.println("Start logging");

          tracker.startLog = timeClient.getEpochTime();
          tracker.shouldLog = false;

          Serial.println("Letting API know");
          apiHandler.updateTrackerStatus(tracker);

          Serial.println("disable wifi");
          // disable wifi if enabled
          if (WiFi.status() == WL_CONNECTED)
          {
            WiFi.disconnect();
          }
          Serial.println("switch states");
          state = STATE_RECORDING;
        }
        else
        {
          Serial.println("nothing to do");
        }
      }

      break;

    case STATE_RECORDING:
      // recordingLoop();
      goToDeepSleep(SLEEP_TIME_MS);
      break;
    case STATE_SETUP:
    default:
      // set state to recording
      state = STATE_IDLE;
      break;
    }

    delay(100);
  }
  catch (const std::exception &e)
  {
    Serial.println(e.what());
    while (true)
    {
      delay(1000);
    }
  }
}

void recordingLoop()
{
  uint32_t recordingStartTime = millis();
  uint32_t recordingEndTime = recordingStartTime + RECORD_TIME_US / 1000;
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
      Serial.print((recordingProgress - recordingStartTime) * 100 / (RECORD_TIME_US / 1000));
      Serial.println("%)");
    }
  }
  // Close the file
  cardHandler.closeFile(file);
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

// get the deep sleep time in us
void goToDeepSleep(uint64_t DEEP_SLEEP_TIME)
{
  Serial.println("Going to sleep...");

  // add the sleep time to the time
  long time = timeClient.getEpochTime();
  time += DEEP_SLEEP_TIME / 1000 / 1000;
  // write the time to the eeprom
  configHandler.config.time = time;
  configHandler.saveConfig();

  // print wake up time
  Serial.print("Wake up time: ");
  printClock(time);

  // disconnect wifi and bt
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  btStop();

  adc_power_off();
  esp_wifi_stop();
  esp_bt_controller_disable();

  // Configure the timer to wake us up!
  esp_sleep_enable_timer_wakeup(DEEP_SLEEP_TIME);

  // Go to sleep! Zzzz
  esp_deep_sleep_start();
}

void connectToWifi()
{
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
}