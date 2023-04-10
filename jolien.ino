
#include "FS.h"
#include "SPI.h"
#include <SdFat.h>
#include "Wire.h"
#include "esp_wifi.h"
#include "esp_bt.h"

#include <driver/i2s.h>
#include <math.h>
#include <Arduino.h>
#include <TimeLib.h>
#define SD_FAT_TYPE 2

// SDCARD_SS_PIN is defined for the built-in SD on some boards.
#ifndef SDCARD_SS_PIN
const uint8_t SD_CS_PIN = GPIO_NUM_5;
#else  // SDCARD_SS_PIN
// Assume built-in SD is used.
const uint8_t SD_CS_PIN = SDCARD_SS_PIN;
SDCARD_SS_PIN
#endif //

// Try max SPI clock for an SD. Reduce SPI_CLOCK if errors occur.
#define SPI_CLOCK SD_SCK_MHZ(15)

// Try to select the best SD card configuration.
#if HAS_SDIO_CLASS
#define SD_CONFIG SdioConfig(FIFO_SDIO)
#elif ENABLE_DEDICATED_SPI
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, DEDICATED_SPI, SPI_CLOCK)
#else // HAS_SDIO_CLASS
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, SHARED_SPI, SPI_CLOCK)
#endif // HAS_SDIO_CLASS

#if SD_FAT_TYPE == 0
SdFat sd;
typedef File file_t;
#elif SD_FAT_TYPE == 1
SdFat32 sd;
typedef File32 file_t;
#elif SD_FAT_TYPE == 2
SdExFat sd;
typedef ExFile file_t;
#elif SD_FAT_TYPE == 3
SdFs sd;
typedef FsFile file_t;
#else // SD_FAT_TYPE
#error Invalid SD_FAT_TYPE
#endif // SD_FAT_TYPE

#define I2S_WS GPIO_NUM_22
#define I2S_SD GPIO_NUM_21
#define I2S_SCK GPIO_NUM_26
#define I2S_PORT_NUM I2S_NUM_0
#define I2S_SAMPLE_RATE (44100)
#define I2S_READ_LEN (1024)
#define I2S_NUM_CHANNELS (1)

// easy format 2 min
#define RECORD_TIME_MS (120000) // ( 2 * 60 * 1000 )
// easy format 8 min
#define SLEEP_TIME_MS (480000) //( 8 * 60 * 1000 )

// Set up the WAV header
struct wav_header_t
{
  char riff[4];         /* "RIFF"                                  */
  long flength;         /* file length in bytes                    */
  char wave[4];         /* "WAVE"                                  */
  char fmt[4];          /* "fmt "                                  */
  long chunk_size;      /* size of FMT chunk in bytes (usually 16) */
  short format_tag;     /* 1=PCM, 257=Mu-Law, 258=A-Law, 259=ADPCM */
  short num_chans;      /* 1=mono, 2=stereo                        */
  long srate;           /* Sampling rate in samples per second     */
  long bytes_per_sec;   /* bytes per second = srate*bytes_per_samp */
  short bytes_per_samp; /* 2=16-bit mono, 4=16-bit stereo          */
  short bits_per_samp;  /* Number of bits per sample               */
  char data[4];         /* "data"                                  */
  long dlength;         /* data length in bytes (filelength - 44)  */
} wavh;

void setup()
{
  Serial.begin(115200);

  while (!Serial)
  {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  esp_wifi_set_mode(WIFI_MODE_NULL);
  esp_bt_controller_disable();

  // wait some time for the serial monitor to open
  delay(5000);
  Serial.println("Setup");

  if (!sd.begin(SD_CONFIG))
  {
    sd.initErrorHalt(&Serial);
    Serial.println("SD card initialization failed!");
    return;
  }

  uint32_t size = sd.card()->sectorCount();
  if (size == 0)
  {
    Serial.println("Can't determine the card size.\n");
    return;
  }

  Serial.print("Card size: ");
  Serial.print(size / 1024);
  Serial.println(" MB");

  delay(1000);
  i2s_install();
  i2s_setpin();
  i2s_start(I2S_PORT_NUM);

  delay(5000);
  Serial.println("Setup done");
}

void loop()
{
  uint32_t recordingStartTime = millis();
  uint32_t recordingEndTime = recordingStartTime + RECORD_TIME_MS;
  uint32_t recordingProgress = 0;

  char filename[32];
  sprintf(filename, "/%04d-%02d-%02d_%02d-%02d-%02d.wav", year(), month(), day(), hour(), minute(), second());

  Serial.print("Recording to file: ");
  Serial.println(filename);
  file_t file;

  if (!file.open(filename, O_APPEND | O_WRITE | O_CREAT))
  {
    sd.errorHalt(&Serial, F("open failed"));
    return;
  }

  wav_header_t wavh = {
      {'R', 'I', 'F', 'F'},
      0, // Placeholder for file length
      {'W', 'A', 'V', 'E'},
      {'f', 'm', 't', ' '},
      16, // FMT chunk size
      1,  // PCM format
      I2S_NUM_CHANNELS,
      I2S_SAMPLE_RATE,
      I2S_SAMPLE_RATE * I2S_READ_LEN * I2S_NUM_CHANNELS,
      I2S_NUM_CHANNELS * (I2S_READ_LEN / 2), // bytes per sample = 2 for 16-bit audio
      16,                                    // 16-bit audio
      {'d', 'a', 't', 'a'},
      0 // Placeholder for data length
  };

  // Write the WAV header to the file
  file.write((uint8_t *)&wavh, sizeof(wavh));

  // Record audio samples to the file
  while (millis() < recordingEndTime)
  {
    // Allocate a buffer for the samples
    int16_t buffer[I2S_READ_LEN * I2S_NUM_CHANNELS];

    // Read samples from the I2S bus
    size_t bytesRead;
    i2s_read(I2S_PORT_NUM, &buffer, I2S_READ_LEN * sizeof(int16_t), &bytesRead, portMAX_DELAY);

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

  Serial.println("Recording done");

  // Stop the I2S bus
  i2s_stop(I2S_PORT_NUM);

  delay(1000);
  Serial.println("Going to sleep...");
  esp_sleep_enable_timer_wakeup(SLEEP_TIME_MS * 1000);
  esp_deep_sleep_start();
  Serial.println("Done");
  ESP.restart();
}

void i2s_install()
{
  i2s_config_t i2s_config = {
      .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
      .sample_rate = I2S_SAMPLE_RATE,
      .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
      .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
      .communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
      .intr_alloc_flags = 0, // default interrupt priority
      .dma_buf_count = 4,
      .dma_buf_len = I2S_READ_LEN,
      .use_apll = false};


  i2s_driver_install(I2S_PORT_NUM, &i2s_config, 0, NULL);
  i2s_set_sample_rates(I2S_PORT_NUM, I2S_SAMPLE_RATE); // set sample rates
}

void i2s_setpin()
{
  const i2s_pin_config_t pin_config = {
      .bck_io_num = I2S_SCK,
      .ws_io_num = I2S_WS,
      .data_out_num = I2S_PIN_NO_CHANGE,
      .data_in_num = I2S_SD};

  i2s_set_pin(I2S_PORT_NUM, &pin_config);
  i2s_set_clk(I2S_PORT_NUM, I2S_SAMPLE_RATE, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_MONO);
}
