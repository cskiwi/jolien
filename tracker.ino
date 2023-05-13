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
#include "arduinoFFT.h"

// #define RECORD_TIME_US (2 * 60 * 1000 * 1000)
// #define NO_RECORD_TIME_US (8 * 60 * 1000 * 1000)

#define RECORD_TIME_US (1 * 60 * 1000 * 1000)
#define NO_RECORD_TIME_US (0.5 * 60 * 1000 * 1000)
#define DECIBEL_UPDATE_INTERVAL_US (1 * 1000)

#define WRITE_SOUND_FILE false
#define WRITE_DECIBEL_FILE false

#define INSTANT_LOGGING true

const char *ssid = "iPhone van Jolien";
const char *password = "wifiJolien";
// const char *server = "https://gull.purr.dev"; // Server URL
const char *server = "http://192.168.1.253:3001"; // Server URL
const char *apiKey = "123456789";                 // Your API key
const char *trackerName = "tracker-tst";          // Your tracker name

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
  Serial.println("Setup done");
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

  while (true)
  {
    uint32_t recordingStartTimeMilis = millis();
    uint32_t recordingEndTimeMilis = recordingStartTimeMilis + (RECORD_TIME_US / 1000);
    uint32_t loopEndTimeMiliss = recordingStartTimeMilis + ((NO_RECORD_TIME_US + RECORD_TIME_US) / 1000);
    uint32_t recordingProgress = 0;
    uint32_t recordingProgressLast = 0;
    uint32_t lastPeak = 0;

    file_t soundFile;

    if (WRITE_SOUND_FILE)
    {
      soundFile = getSoundFile();
    }

    // Record audio samples to the file
    while (millis() < loopEndTimeMiliss)
    {
      int i2s_read_len = I2S_READ_LEN;
      char *i2s_read_buff = (char *)calloc(i2s_read_len, sizeof(char));
      uint8_t *flash_write_buff = (uint8_t *)calloc(i2s_read_len, sizeof(char));

      // Read samples from the I2S bus
      size_t bytes_read;
      i2s_read(I2S_PORT_NUM, (void *)i2s_read_buff, i2s_read_len, &bytes_read, portMAX_DELAY);
      i2s_adc_data_scale(flash_write_buff, (uint8_t *)i2s_read_buff, i2s_read_len);

      if (WRITE_SOUND_FILE && millis() < recordingEndTimeMilis)
      {
        // Write the samples to the file
        soundFile.write((const byte *)flash_write_buff, bytes_read);
      }

      if ((millis() - recordingProgress) > DECIBEL_UPDATE_INTERVAL_US)
      {
        recordingProgress = millis();

        // get highest peak

        // calculate dbA
        int dbA = 0;

        // Convert RMS to dB
        Serial.print("DbA: ");
        Serial.println(dbA);
        printProgress(recordingStartTimeMilis, recordingEndTimeMilis, loopEndTimeMiliss);
      }

      free(i2s_read_buff);
      i2s_read_buff = NULL;
      free(flash_write_buff);
      flash_write_buff = NULL;
    }

    if (WRITE_SOUND_FILE)
    {
      // Close the file
      soundFile.close();
    }

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
    wav_header_t wavh = soundHandler.getWavHeader(RECORD_TIME_US);

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
