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
#include "clock/printClock.h"
#include "optimisation/cpu.h"

// #define RECORD_TIME_US (2 * 60 * 1000 * 1000)
// #define NO_RECORD_TIME_US (8 * 60 * 1000 * 1000)

#define RECORD_TIME_US (2 * 60 * 1000 * 1000)
#define NO_RECORD_TIME_US (1 * 60 * 1000 * 1000)

const char *ssid = "tracker-hotspot";
const char *password = "love-you";
// const char *server = "https://gull.purr.dev"; // Server URL
const char *server = "http://192.168.1.253:3001"; // Server URL
const char *apiKey = "123456789";                 // Your API key
const char *trackerName = "tracker-02";           // Your tracker name

// Setup handlers
CardHandler cardHandler = CardHandler();
SoundHandler soundHandler = SoundHandler();
// ConfigHandler configHandler = ConfigHandler(cardHandler);
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

  try
  {
    // disable bluetooth
    esp_bt_controller_disable();

    // rest a bit to free up the cpu
    delay(1000);

    // setup the card reader
    cardHandler.init();
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

    // temp directly start
    delay(1000);
    startLogging();
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

void recordingLoop()
{
  while (true)
  {
    uint32_t recordingStartTimeMilis = millis();
    uint32_t recordingEndTimeMilis = recordingStartTimeMilis + (RECORD_TIME_US / 1000);
    uint32_t loopEndTimeMiliss = recordingStartTimeMilis + ((NO_RECORD_TIME_US + RECORD_TIME_US) / 1000);
    uint32_t recordingProgress = 0;
    uint32_t recordingProgressLast = 0;

    file_t soundFile = getSoundFile();

    // Record audio samples to the file
    while (millis() < loopEndTimeMiliss)
    {
      // Allocate a buffer for the samples
      int16_t buffer[I2S_READ_LEN * I2S_NUM_CHANNELS];

      // Read samples from the I2S bus
      size_t bytesRead;
      i2s_read(I2S_PORT_NUM, &buffer, I2S_READ_LEN * sizeof(int16_t), &bytesRead, portMAX_DELAY);

      if (millis() < recordingEndTimeMilis)
      {
        // Write the samples to the file
        soundFile.write((uint8_t *)buffer, bytesRead);
      }

      // print progress every 10 seconds
      if (millis() - recordingProgress > 10000)
      {
        recordingProgress = millis();
        float dbA = 0;// code here
        Serial.print("DbA: ");
        Serial.println(dbA);
        printProgress(recordingStartTimeMilis, recordingEndTimeMilis, loopEndTimeMiliss);
      }
    }

    // Close the file
    soundFile.close();

    Serial.println("Recording done, wait a second to let the file close");
    delay(1000);
  }
}

file_t getSoundFile()
{
  unsigned long epoch = timeClient.getEpochTime();
  // soundfile name with date and time
  char soundFileName[50];
  sprintf(soundFileName, "sound-%04d-%02d-%02d_%02d_%02d_%02d.wav", year(epoch), month(epoch), day(epoch), hour(epoch), minute(epoch), second(epoch));
  Serial.print("Recording sound to file: ");
  Serial.println(soundFileName);

  file_t soundFile;

  if (!soundFile.open(soundFileName, O_APPEND | O_WRITE | O_CREAT))
  {
    Serial.println("Failed to open file");
    throw std::runtime_error("Failed to open file");
  }

  Serial.println("If new file, add header");

  // if file size is 0, add header
  if (soundFile.size() == 0)
  {
    // Get the WAV header
    wav_header_t wavh = soundHandler.getWavHeader();

    // Write the WAV header to the file
    soundFile.write((uint8_t *)&wavh, sizeof(wavh));
    Serial.println("WAV header written");
  }

  return soundFile;
}

file_t getDbAFile()
{
  unsigned long epoch = timeClient.getEpochTime();
  char dbAfilename[50];
  sprintf(dbAfilename, "soundlevels-%04d-%02d-%02d_%02d_00_00.csv", year(epoch), month(epoch), day(epoch), hour(epoch));
  Serial.print("Recording dbA to file: ");
  Serial.println(dbAfilename);

  // initialize the dbA file if not already done
  file_t dbAfile = cardHandler.getFile(dbAfilename, O_RDWR | O_CREAT | O_AT_END | O_APPEND | O_TRUNC);

  Serial.println("Check if file is empty");
  // if file size is 0, add header
  if (dbAfile.size() == 0)
  {
    dbAfile.write("Time (s),dbA\n", 12);

    Serial.println("dbA header written");
  }

  return dbAfile;
}

int16_t *getAudioBuffer(const int recording_ms)
{
  // calculate the number of samples to read at a time
  const int sample_size = I2S_READ_LEN;
  // make sure sample_size is a multiple of I2S_NUM_CHANNELS
  const int buffer_size = I2S_SAMPLE_RATE * recording_ms / 1000 * I2S_NUM_CHANNELS;
  const int samples_remaining = buffer_size / (sample_size);

  // check if there is enough space in the heap for the audio buffer
  if (ESP.getFreeHeap() < buffer_size)
  {
    Serial.println("Error: not enough free heap for audio buffer");
    Serial.print("Free heap: ");
    Serial.println(ESP.getFreeHeap());

    Serial.print("Required heap: ");
    Serial.println(buffer_size);
    return nullptr; // return null pointer to indicate failure
  }

  int16_t *audio_buffer = new int16_t[buffer_size]; // dynamically allocate memory to hold the audio data
  size_t bytes_read;                                // declare variable to hold the number of bytes read
  int read_offset = 0;

  for (int i = 0; i < samples_remaining; i++)
  {
    esp_err_t result = i2s_read(I2S_PORT_NUM, audio_buffer + read_offset, I2S_READ_LEN, &bytes_read, portMAX_DELAY); // read the audio data
    read_offset += bytes_read;                                                                                       // update the read offset based on the number of bytes read
  }

  return audio_buffer; // return the audio buffer
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

void printSummary(uint32_t recordingStartTimeMilis, uint32_t recordingEndTimeMilis, uint32_t loopEndTimeMiliss)
{
  // print milis
  Serial.print("Recording started at ");
  Serial.println(recordingStartTimeMilis);

  Serial.print("Ends at ");
  Serial.println(recordingEndTimeMilis);

  Serial.print("Loop ends at ");
  Serial.println(loopEndTimeMiliss);

  // Serial.print("Recording started at ");
  // printClock(timeClient.getEpochTime(), true);

  Serial.print("Recording for ");
  Serial.print(RECORD_TIME_US / 1000 / 1000);

  Serial.print("s, until ");

  Serial.print("Then only fetching dbA for ");
  Serial.print(NO_RECORD_TIME_US / 1000 / 1000);

  Serial.println("s");
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

  if (endTime < now)
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
