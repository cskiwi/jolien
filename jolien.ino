
#include <Arduino.h>
#include <TimeLib.h>
#include "esp_wifi.h"
#include "esp_bt.h"
#include "CardReader.h"
#include "SoundHandler.h"

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

//  save configuration to a config.txt file
char configFileName[12] = "/config.txt";


// depending on the state activate the bleutooth for the app or run in recording mode
#define STATE_SETUP 0
#define STATE_RECORDING 1

int state = STATE_SETUP;

void setup()
{
  Serial.begin(115200);

  while (!Serial)
  {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  // setSyncProvider( requestSync);  //set function to call when sync required

  // disable wifi and bluetooth
  esp_wifi_set_mode(WIFI_MODE_NULL);
  esp_bt_controller_disable();

  try
  {
    // initialize card reader
    cardHandler.init();

    // wait some time for the serial monitor to open
    delay(1000);

    // read the config file
    file_t configFile = cardHandler.getFile(configFileName);

    // read the config file
    while (configFile.available())
    {
      String line = configFile.readStringUntil('\n');
      if (line.startsWith("ssid:"))
      {
        config.ssid = line.substring(5);
      }
      else if (line.startsWith("time:"))
      {
        config.time = line.substring(5);
      }
    }

    if (time == 0)
    {
      // set time to 12:00:00 1.1.2020 if not set
      setTime(12, 0, 0, 1, 1, 2020);
    }
    else
    {
      // set the time
      setTime(time);
    }

    Serial.print("Time to: ");
    printClock(time);

    // initialize sound handler
    soundHandler.init();

    // wait some time for the serial monitor to open
    delay(5000);
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

void loop()
{

  switch (state)
  {

  case STATE_RECORDING:
    // recordingLoop();
    goToSleep();
    break;
  case STATE_SETUP:
  default:
    // setup the time
    int currentTime = now();

    // set state to recording
    state = STATE_RECORDING;

    // Print some info
    Serial.print("Current time: ");
    Serial.println(currentTime);
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
  Serial.print("Writing time to eeprom: ");
  printClock(time);
  // write the time to the eeprom
  eepromManager.writeLong(time, TIME_ADDRESS);

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

void processSyncMessage()
{
  unsigned long pctime;
  const unsigned long DEFAULT_TIME = 1357041600; // Jan 1 2013

  if (Serial.find(TIME_HEADER))
  {
    pctime = Serial.parseInt();
    if (pctime >= DEFAULT_TIME)
    {                  // check the integer is a valid time (greater than Jan 1 2013)
      setTime(pctime); // Sync Arduino clock to the time received on the serial port
    }
  }
}

time_t requestSync()
{
  Serial.write(TIME_REQUEST);
  return 0; // the time will be sent later in response to serial mesg
}
