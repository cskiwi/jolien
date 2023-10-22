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

#define PROD false
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
#define GAIN_FACTOR 3.0

#define BLINK_INTERVAL 500
#define STOP_BLINK 5
#define LED_1 LED_BUILTIN

#define CONNECTION_TIMEOUT 10

const char *ssid = "iPhone van Jolien";
const char *password = "wifiJolien";

#if PROD
const char *server = "https://gull.purr.dev"; // Server URL
#else
const char *server = "http://192.168.1.253:3001"; // Server URL
#endif

const char *apiKey = "123456789";       // Your API key
const char *trackerName = "tracker-00"; // Your tracker name

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
int retryCounter = 0;
int blinkAmount = 0;

void blinkLed(int times, uint8_t pin = LED_1)
{
  for (int i = 0; i < times; i++)
  {
    digitalWrite(pin, HIGH);
    delay(BLINK_INTERVAL);
    digitalWrite(pin, LOW);
    delay(BLINK_INTERVAL);
  }
}

void setup()
{
  pinMode(LED_1, OUTPUT);

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

    // blink led once
    blinkLed(2);

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

    retryCounter++;

    if (retryCounter < 5)
    {
      // wait a bit and try again
      delay(1000);
      ESP.restart();
    }

    while (true)
    {
      // if it keeps failing, just stop
      // set the led to high to indicate something is wrong
      digitalWrite(LED_BUILTIN, HIGH);

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

      // ping the api every 10 seconds
      if (second() % 10 == 0 || initalStartup)
      {
        blinkLed(1);
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

  float gain = GAIN_FACTOR;

  for (uint32_t i = 0; i < len; i += sizeof(int32_t))
  {
    int32_t sample = *((int32_t *)(s_buff + i));
    sample = (int32_t)(sample * gain);
    *((int32_t *)(d_buff + i)) = sample;
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

    // get filenames
    String dbAfilename = getDBFilename();
    delay(100);

    String soundFileName = getSoundFilename();
    delay(100);

    if (WRITE_SOUND_FILE && !soundFile.isOpen())
    {
      Serial.println("Opening sound file");
      soundFile = getSoundFile(soundFileName, true);
      delay(100);
    }

    if (WRITE_DECIBEL_FILE && !decibelFile.isOpen())
    {
      Serial.println("Opening dbA file");
      decibelFile = getDBFile(dbAfilename, true);
      delay(100);
    }

    Serial.println("Start recording");

    // blink led 3 times to indicate recording on a different thread
    blinkLed(3);

    char *i2s_read_buff = (char *)calloc(I2S_READ_LEN, sizeof(char));
    uint8_t *flash_write_buff = (uint8_t *)calloc(I2S_READ_LEN, sizeof(char));
    size_t bytes_read;

    // Record audio samples to the file
    while (millis() < loopEndTimeMiliss)
    {
      i2s_read(I2S_PORT_NUM, (void *)i2s_read_buff, I2S_READ_LEN, &bytes_read, portMAX_DELAY);

      // Serial.println("Scaling data...");
      i2s_adc_data_scale(flash_write_buff, (uint8_t *)i2s_read_buff, I2S_READ_LEN);
      // Serial.println("Data scaled");
      // Write the scaled audio samples to flash memory here

      if (WRITE_SOUND_FILE && millis() < recordingEndTimeMilis)
      {
        if (!soundFile.isWritable())
        {
          Serial.println("Sound file is not writable??");
        }

        // Write the samples to the file
        // Serial.println("Writing data to file...");
        soundFile.write((const byte *)flash_write_buff, bytes_read);
        // Serial.println("Data written");
      }

      if (WRITE_DECIBEL_FILE && (millis() - recordingProgress) > DECIBEL_UPDATE_INTERVAL_US)
      {
        // Serial.println("Calculating dbA...");
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

        Serial.print("Writing dbA to file... ");
        // Write the samples to the file
        char csvLine[50];
        unsigned long epoch = timeClient.getEpochTime();
        sprintf(csvLine, "%04d-%02d-%02d %02d:%02d:%02d,%f\n", year(epoch), month(epoch), day(epoch), hour(epoch), minute(epoch), second(epoch), dbA);

        if (!decibelFile.isWritable())
        {
          Serial.println("Dba File is not writable??");
        }

        decibelFile.write((const byte *)csvLine, strlen(csvLine));

        // Convert RMS to dB
        Serial.print("DbA: ");
        Serial.println(csvLine);
      }

      // print progress every 5 seconds
      if (millis() - recordingProgressLast > 5000)
      {
        printProgress(recordingStartTimeMilis, recordingEndTimeMilis, loopEndTimeMiliss);
        recordingProgressLast = millis();

        if (blinkAmount < STOP_BLINK)
        {
          // blink led to indicate recording
          blinkLed(1);

          blinkAmount++;
        }
      }
    }

    if (WRITE_SOUND_FILE && soundFile.isOpen())
    {
      Serial.println("Closing sound file");
      soundFile.close();
    }

    if (WRITE_DECIBEL_FILE && decibelFile.isOpen())
    {
      Serial.println("Closing dbA file");
      decibelFile.close();
    }

    // freeing heap
    free(i2s_read_buff);
    i2s_read_buff = NULL;
    free(flash_write_buff);
    flash_write_buff = NULL;

    Serial.println("");
    Serial.println("");
    Serial.println("");
    Serial.println("===================");
    Serial.println("Recording done, wait a second to let the file close");
    Serial.println("===================");
    Serial.println("");
    Serial.println("");
    Serial.println("");
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
      Serial.println("Failed to open sound file");
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
    if (!dbAfile.open(dbAfilename.c_str(), O_APPEND | O_WRITE | O_CREAT))
    {
      Serial.println("Failed to dbA file");
      retry++;
      delay(1000); // wait for 1 second before retrying
    }
    else
    {
      if (addHeader)
      {
        delay(1000); // wait for 1 second before retrying
        Serial.println("Check if file is empty");
        // if file size is 0, add header
        if (dbAfile.size() == 0)
        {
          Serial.println("Adding header");
          String dbHeader = "Time,dbA\n";
          dbAfile.write((const byte *)dbHeader.c_str(), dbHeader.length());
        }
        return dbAfile;
      }
    }
  }

  throw std::runtime_error("Failed to create file after 5 retries"); // throw an error if all retries failed
}

void setupWifiConnection(const char *ssid, const char *password)
{
  int timeout_counter = 0;
  WiFi.begin(ssid, password);
  wl_status_t wifi_status = WiFi.status();
  while (wifi_status != WL_CONNECTED)
  {
    switch (WiFi.status())
    {
    case WL_NO_SHIELD:
      Serial.println("No WiFi shield is present.");
      break;
    case WL_IDLE_STATUS:
      Serial.println("Attempting to connect...");
      break;
    case WL_NO_SSID_AVAIL:
      Serial.println("No SSID available.");
      break;
    case WL_SCAN_COMPLETED:
      Serial.println("Scan Networks is complete.");
      break;
    case WL_CONNECT_FAILED:
      Serial.println("Connection FAILED.");
      break;
    case WL_CONNECTION_LOST:
      Serial.println("Connection LOST.");
      break;
    case WL_DISCONNECTED:
      Serial.println("Device has been DISCONNECTED from the Network.");
      break;
    default:
      Serial.println("UNKNOWN ERROR");
      break;
    }

    delay(1000);
    Serial.println("Connecting to WiFi...");
    timeout_counter++;
    if (timeout_counter >= CONNECTION_TIMEOUT)
    {
      ESP.restart();
    }

    wifi_status = WiFi.status();
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
    Serial.print("We have been recording for ");
    Serial.print(elapsedSeconds);
    Serial.print("s, ");
    Serial.print(remainingSeconds);
    Serial.print("s remaining, loop ends in ");
    Serial.print(loopRemainingSeconds);
    Serial.print("s");
    Serial.print(", progress: ");
    Serial.print(progress);
    Serial.println("%");
  }
}
