#include <Arduino.h>
#include <TimeLib.h>
#include "esp_wifi.h"
#include "esp_bt.h"
#include "CardReader.h"
#include "SoundHandler.h"
#include "ApiHandler.h"
#include "Tracker.h"
#include <WiFi.h>
#include "Timer.h"
#include <NTPClient.h>
#include <WiFiUdp.h>
#include "clock/printClock.h"
#include "optimisation/cpu.h"
#include "arduinoFFT.h"

#define PROD true
#define INSTANT_LOGGING false

#if PROD
#define RECORD_TIME_US (2 * 60 * 1000 * 1000)
#define NO_RECORD_TIME_US (8 * 60 * 1000 * 1000)
#define DECIBEL_UPDATE_INTERVAL_US (10 * 1000)
#else
#define RECORD_TIME_US (1 * 60 * 1000 * 1000)
#define NO_RECORD_TIME_US (0.5 * 60 * 1000 * 1000)
#define DECIBEL_UPDATE_INTERVAL_US (1 * 1000)
#endif

#define WRITE_SOUND_FILE true
#define WRITE_DECIBEL_FILE true

const char *ssid = "iPhone van Jolien";
const char *password = "wifiJolien";

#if PROD
const char *server = "https://gull.purr.dev"; // Server URL
#else
const char *server = "http://192.168.1.253:3001"; // Server URL
#endif

const char *apiKey = "123456789";       // Your API key
const char *trackerName = "tracker-15"; // Your tracker name

// Setup handlers
CardHandler cardHandler = CardHandler();
SoundHandler soundHandler = SoundHandler();
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
bool initalStartup = true;

void setup()
{
  Serial.begin(115200);

  while (!Serial)
  {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  try
  {
    // disable bluetooth
    esp_bt_controller_disable();

    // rest a bit to free up the cpu
    delay(1000);

    // setup the card reader
    if (WRITE_SOUND_FILE || WRITE_DECIBEL_FILE)
    {
      cardHandler.init();
    }
    else

      Serial.println("Card reader initialized");

    // setup wifi
    setupWifiConnection(ssid, password);

    // initialize the time client
    timeClient.begin();
    timeClient.setTimeOffset(3600 * 2);
    timeClient.update();

    // wait for the time to be set
    delay(1000);

    // initialize api handler
    apiHandler.setEndpoint(server, trackerName);
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
  Serial.println("Setup done :)");
  Serial.println("Environment: ");
  if (PROD)
  {
    Serial.println("PROD");
  }
  else
  {
    Serial.println("DEV");
  }
  Serial.println("===================");
  Serial.println("");
  Serial.println("");
  Serial.println("");

  if (INSTANT_LOGGING)
  {
    delay(1000);
    startLogging();
  }
}

void loop()
{
  try
  {

    if (state == STATE_RECORDING)
    {
      recordingLoop();
    }
    else
    {

      // ping the api every minute
      if (second() == 0 || initalStartup)
      {
        initalStartup = false;
        apiHandler.pingTrackerStatus(tracker);
        Serial.println("Checkinf if logging");
        if (tracker.shouldLog)
        {
          startLogging();
        }
        else
        {
          Serial.println("nothing to do");
        }
      }
    }
  }
  catch (const std::exception &e)
  {
    Serial.println(e.what());
    while (true)
    {
      delay(1000);
    }
  }
  // cath all other
  catch (...)
  {
    Serial.println("Unknown error");
    while (true)
    {
      delay(1000);
    }
  }
}

void startLogging()
{
  Serial.println("Start logging");
  timeClient.update();
  tracker.shouldLog = false;
  tracker.startedLogOn = timeClient.getEpochTime();

  Serial.println("Letting API know");
  apiHandler.updateTrackerStatus(tracker);

  Serial.println("disable wifi");
  // disconnect wifi
  if (WiFi.status() == WL_CONNECTED)
  {
    WiFi.disconnect();
  }

  // turn off wifi
  WiFi.mode(WIFI_OFF);

  Serial.println("Initialize sound handler");
  soundHandler.init();

  Serial.println("switch states");
  state = STATE_RECORDING;
}

void i2s_adc_data_scale(uint8_t *d_buff, uint8_t *s_buff, uint32_t len)
{
  if (s_buff == NULL)
  {
    Serial.println("Error: s_buff pointer is NULL");
    return;
  }

  if (len % 2 != 0)
  {
    Serial.println("Error: len is not a multiple of 2");
    return;
  }

  uint32_t j = 0;
  uint32_t dac_value = 0;
  for (int i = 0; i < len; i += 2)
  {
    dac_value = ((((uint16_t)(s_buff[i + 1] & 0xf) << 8) | ((s_buff[i + 0]))));
    d_buff[j++] = 0;
    d_buff[j++] = dac_value * 256 / 2048;
  }
}

void recordingLoop()
{
  Serial.println("");
  Serial.println("");
  Serial.println("");
  Serial.println("===================");
  Serial.println("Start recording");
  Serial.println("===================");
  Serial.println("");
  Serial.println("");
  Serial.println("");
  const float REFERENCE_SOUND_PRESSURE = 356e-6; // 20 µPa

  while (true)
  {
    uint32_t recordingStartTimeMilis = millis();
    uint32_t recordingEndTimeMilis = recordingStartTimeMilis + (RECORD_TIME_US / 1000);
    uint32_t loopEndTimeMiliss = recordingStartTimeMilis + ((NO_RECORD_TIME_US + RECORD_TIME_US) / 1000);
    uint32_t recordingProgress = 0;
    uint32_t recordingProgressLast = 0;
    uint32_t lastPeak = 0;

    file_t soundFile;
    file_t decibelFile;

    String dbAfilename = getDBFilename();
    String soundFileName = getSoundFilename();

    if (WRITE_SOUND_FILE && !soundFile.isOpen())
    {
      Serial.println("Opening sound file");
      soundFile = getSoundFile(soundFileName, true);
    }

    if (WRITE_DECIBEL_FILE && !decibelFile.isOpen())
    {
      Serial.println("Opening dbA file");
      decibelFile = getDBFile(dbAfilename, true);
    }

    // Record audio samples to the file
    while (millis() < loopEndTimeMiliss)
    {
      char *i2s_read_buff = (char *)calloc(I2S_READ_LEN, sizeof(char));
      uint8_t *flash_write_buff = (uint8_t *)calloc(I2S_READ_LEN, sizeof(uint8_t));
      size_t bytes_read;
      i2s_read(I2S_PORT_NUM, (void *)i2s_read_buff, I2S_READ_LEN, &bytes_read, portMAX_DELAY);
      delay(100);
      i2s_adc_data_scale(flash_write_buff, (uint8_t *)i2s_read_buff, I2S_READ_LEN);

      if (WRITE_SOUND_FILE && millis() < recordingEndTimeMilis)
      {
        // Write the samples to the file
        soundFile.write((const byte *)flash_write_buff, bytes_read);
      }

      if ((millis() - recordingProgress) > DECIBEL_UPDATE_INTERVAL_US)
      {
        recordingProgress = millis();
        // get highest peak
        float highest_peak = 0;
        for (int i = 0; i < bytes_read; i += 2)
        {
          int16_t sample = (int16_t)((flash_write_buff[i + 1] << 8) | flash_write_buff[i]);
          float sample_abs = abs(sample);
          if (sample_abs > highest_peak)
          {
            highest_peak = sample_abs;
          }
        }

        // calculate dbA
        float rms = sqrt(highest_peak / (bytes_read / 2));
        float dbA = 20 * log10(rms / REFERENCE_SOUND_PRESSURE);

        if (WRITE_DECIBEL_FILE)
        {
          // Write the samples to the file
          char csvLine[50];
          unsigned long epoch = timeClient.getEpochTime();
          sprintf(csvLine, "%04d-%02d-%02d %02d:%02d:%02d,%f\n", year(epoch), month(epoch), day(epoch), hour(epoch), minute(epoch), second(epoch), dbA);

          decibelFile.write((const byte *)csvLine, strlen(csvLine));

          // Convert RMS to dB
          Serial.print("DbA: ");
          Serial.println(csvLine);
        }

        printProgress(recordingStartTimeMilis, recordingEndTimeMilis, loopEndTimeMiliss);
      }

      heap_caps_free(i2s_read_buff);
      i2s_read_buff = NULL;
      heap_caps_free(flash_write_buff);
      flash_write_buff = NULL;

      // freeing heap
      delay(100);
    }

    if (WRITE_SOUND_FILE && soundFile.isOpen())
    {
      soundFile.close();
    }

    if (WRITE_DECIBEL_FILE && decibelFile.isOpen())
    {
      decibelFile.close();
    }

    Serial.println("Recording done, wait a second to let the file close");
    delay(1000);
  }
}

String getSoundFilename()
{
  unsigned long epoch = timeClient.getEpochTime();
  // soundfile name with date and time
  char soundFileName[50];
  sprintf(soundFileName, "sound-%04d-%02d-%02d_%02d_%02d_%02d.wav", year(epoch), month(epoch), day(epoch), hour(epoch), minute(epoch), second(epoch));
  Serial.print("Recording sound to file: ");
  Serial.println(soundFileName);

  return String(soundFileName);
}

file_t getSoundFile(const String &soundFileName, bool addHeader)
{
  file_t soundFile;

  int retry = 0; // initialize retry count
  while (retry < 5)
  { // retry up to 5 times
    if (!soundFile.open(soundFileName.c_str(), O_APPEND | O_WRITE | O_CREAT))
    {
      Serial.println("Failed to open file");
      retry++;
      delay(1000); // wait for 1 second before retrying
    }
    else
    {
      if (addHeader)
      {
        Serial.println("If new file, add header");

        // if file size is 0, add header
        if (soundFile.size() == 0)
        {
          // Get the WAV header
          wav_header_t wavh = soundHandler.getWavHeader(RECORD_TIME_US);

          // Write the WAV header to the file
          soundFile.write((uint8_t *)&wavh, sizeof(wavh));
          Serial.println("WAV header written");
        }
      }

      return soundFile;
    }
  }

  throw std::runtime_error("Failed to open file after 5 retries"); // throw an error if all retries failed
}

String getDBFilename()
{
  unsigned long epoch = timeClient.getEpochTime();
  char dbAfilename[50];
  sprintf(dbAfilename, "soundlevels-%04d-%02d-%02d_%02d_00_00.csv", year(epoch), month(epoch), day(epoch), hour(epoch));
  Serial.print("Recording dbA to file: ");
  Serial.println(dbAfilename);
  return String(dbAfilename);
}

file_t getDBFile(const String &dbAfilename, bool addHeader)
{
  file_t dbAfile;

  int retry = 0; // initialize retry count
  while (retry < 5)
  { // retry up to 5 times
    dbAfile = cardHandler.getFile(dbAfilename.c_str(), O_APPEND | O_WRITE | O_CREAT);
    if (!dbAfile.isOpen())
    {
      Serial.println("Failed to create file");
      retry++;
      delay(1000); // wait for 1 second before retrying
    }
    else
    {
      if (addHeader)
      {
        Serial.println("If new file, add header");
        Serial.println("Check if file is empty");
        // if file size is 0, add header
        if (dbAfile.size() == 0)
        {
          dbAfile.write("Time,dbA\n", 8);
          Serial.println("dbA header written");
        }
        return dbAfile;
      }
    }
  }

  throw std::runtime_error("Failed to create file after 5 retries"); // throw an error if all retries failed
}

void setupWifiConnection(const char *ssid, const char *password)
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

void printProgress(uint32_t startTime, uint32_t endTime, uint32_t loopEndTime)
{
  uint32_t now = millis();
  uint32_t elapsed = now - startTime;
  uint32_t remaining = endTime - now;
  uint32_t loopRemaining = loopEndTime - now;

  uint32_t elapsedSeconds = elapsed / 1000;
  uint32_t remainingSeconds = remaining / 1000;
  uint32_t loopRemainingSeconds = loopRemaining / 1000;

  uint32_t progress = (elapsed * 100) / (loopEndTime - startTime);

  // when the remaining time is passed only print the loop remaining time

  if (endTime < now || !WRITE_SOUND_FILE)
  {
    Serial.print("Loop ends in ");
    Serial.print(loopRemainingSeconds);
    Serial.println("s");
  }
  else
  {
    Serial.print("Recording for ");
    Serial.print(elapsedSeconds);
    Serial.print("s, until ");
    Serial.print(remainingSeconds);
    Serial.print("s remaining (");
    Serial.print(progress);
    Serial.print("%), loop ends in ");
    Serial.print(loopRemainingSeconds);
    Serial.println("s");
  }
}
