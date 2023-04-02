#include <SdFat.h>
#include "FS.h"
#include "SPI.h"
#include "Wire.h"

#include <driver/i2s.h>
#include <math.h>
#include <Arduino.h>

#define SD_MISO 19
#define SD_MOSI 23
#define SD_SCLK 18
#define SD_CS 5

// SD_FAT_TYPE = 0 for SdFat/File as defined in SdFatConfig.h,
// 1 for FAT16/FAT32, 2 for exFAT, 3 for FAT16/FAT32 and exFAT.
#define SD_FAT_TYPE 2
//
// Set DISABLE_CHIP_SELECT to disable a second SPI device.
// For example, with the Ethernet shield, set DISABLE_CHIP_SELECT
// to 10 to disable the Ethernet controller.
const int8_t DISABLE_CHIP_SELECT = -1;
//
// Test with reduced SPI speed for breadboards.  SD_SCK_MHZ(4) will select
// the highest speed supported by the board that is not over 4 MHz.
// Change SPI_SPEED to SD_SCK_MHZ(50) for best performance.
#define SPI_SPEED SD_SCK_MHZ(50)
//------------------------------------------------------------------------------
#if SD_FAT_TYPE == 0
SdFat sd;
File file;
#elif SD_FAT_TYPE == 1
SdFat32 sd;
File32 file;
#elif SD_FAT_TYPE == 2
SdExFat sd;
ExFile file;
#elif SD_FAT_TYPE == 3
SdFs sd;
FsFile file;
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

// keep track if the card is mounted or not
bool cardMounted = false;

void setup()
{
  Serial.begin(115200);

  while (!Serial)
  {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  if (!sd.begin(SD_CS, SPI_SPEED))
  {
    Serial.println("SD card initialization failed!");
    return;
  }
  else
  {
    cardMounted = true;
  }

  uint32_t size = sd.card()->sectorCount();
  if (size == 0) {
    Serial.println("Can't determine the card size.\n");
    return;
  }

  Serial.print("Card size: ");
  Serial.print(size / 1024);
  Serial.println(" MB");

  delay(10000);
  i2s_install();
  i2s_setpin();
  i2s_start(I2S_PORT_NUM);

  delay(500);
  Serial.println("Setup done");
}

void loop()
{
  if (!cardMounted)
  {
    return;
  }

  Serial.println("Start recording");

  // delete old file
  // SD.remove("/soundfile.wav");

  // Start recording

  // set sample size
  static uint32_t samples[I2S_READ_LEN / sizeof(uint32_t)];

  size_t bytes_read;
  // i2s_read(I2S_PORT_NUM, samples, I2S_READ_LEN, &bytes_read, portMAX_DELAY);
  writeWaveHeader("/soundfile.wav", I2S_SAMPLE_RATE, sizeof(samples), I2S_NUM_CHANNELS);

  // Record for 2 minutes
  for (unsigned long start = millis(); millis() - start < RECORD_TIME_MS;)
  {
    // calculate the remaining time as a percentage
    int remaining = 100 - ((millis() - start) * 100 / RECORD_TIME_MS);

    // print the remaining time as a percentage every 10 seconds
    if (remaining % 10 == 0)
    {
      Serial.print("Remaining: ");
      Serial.print(remaining);
      Serial.println("%");
    }

    size_t bytes_read;
    i2s_read(I2S_PORT_NUM, samples, I2S_READ_LEN, &bytes_read, portMAX_DELAY);

    // Append audio samples to file
    appendFile("/soundfile.wav", (const uint8_t *)samples, bytes_read);
  }

  // Stop recording
  i2s_stop(I2S_PORT_NUM);

  Serial.println("Recording done");

  // Sleep for 8 minutes before taking another measurement
  esp_sleep_enable_timer_wakeup(SLEEP_TIME_MS); // Set sleep timer to 8 minutes
  esp_deep_sleep_start();
  // Enter sleep mode
}

void readFile(SdExFat &sd, const char *path)
{
  Serial.printf("Reading file: %s\n", path);

  file = sd.open(path);
  if (!file)
  {
    Serial.println("Failed to open file for reading");
    return;
  }

  Serial.print("Read from file: ");
  while (file.available())
  {
    Serial.write(file.read());
  }
  file.close();
}

void appendFile(const char *path, const void *data, size_t len)
{
  // Serial.printf("Appending to file: %s\n", path);

  file = sd.open(path, O_APPEND | O_WRITE | O_CREAT);
  if (!file)
  {
    Serial.println("Failed to open file for appending");
    return;
  }
  if (file.write((const uint8_t *)data, len) == len)
  {
    // Serial.println("Data appended");
  }
  else
  {
    Serial.println("Append failed");
  }
  file.close();
}

void writeWaveHeader(const char *path, uint32_t sampleRate, uint16_t bitsPerSample, uint16_t channels)
{
  file = sd.open(path, O_WRITE | O_CREAT);
  if (!file)
  {
    Serial.println("Failed to open file for writing");
    return;
  }

  uint16_t blockAlign = channels * bitsPerSample / 8;
  uint32_t byteRate = sampleRate * blockAlign;

  // Write the RIFF header
  file.write((const uint8_t *)"RIFF", 4);
  uint32_t fileSize = file.size() - 8;
  file.write((const uint8_t *)&fileSize, 4);
  file.write((const uint8_t *)"WAVE", 4);

  // Write the fmt subchunk
  file.write((const uint8_t *)"fmt ", 4);
  uint32_t subchunk1Size = 16;
  file.write((const uint8_t *)&subchunk1Size, 4);
  uint16_t audioFormat = 1;
  file.write((const uint8_t *)&audioFormat, 2);
  file.write((const uint8_t *)&channels, 2);
  file.write((const uint8_t *)&sampleRate, 4);
  file.write((const uint8_t *)&byteRate, 4);
  file.write((const uint8_t *)&blockAlign, 2);
  file.write((const uint8_t *)&bitsPerSample, 2);

  // Write the data subchunk header
  file.write((const uint8_t *)"data", 4);
  uint32_t subchunk2Size = file.size() - 44;
  file.write((const uint8_t *)&subchunk2Size, 4);

  file.close();
}

void deleteFile(const char *path)
{
  Serial.printf("Deleting file: %s\n", path);
  if (sd.remove(path))
  {
    Serial.println("File deleted");
  }
  else
  {
    // Serial.println("Delete failed");
  }
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