#include "FS.h"
#include "SPI.h"
#include <SdFat.h>
#include "Wire.h"
#include "esp_wifi.h"
#include "esp_bt.h"
#include "CardReader.h"

// Try to select the best SD card configuration.
#if HAS_SDIO_CLASS
#define SD_CONFIG SdioConfig(FIFO_SDIO)
#elif ENABLE_DEDICATED_SPI
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, DEDICATED_SPI, SPI_CLOCK)
#else // HAS_SDIO_CLASS
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, SHARED_SPI, SPI_CLOCK)
#endif // HAS_SDIO_CLASS

CardHandler::CardHandler()
{
}

void CardHandler::init()
{
  Serial.println("CardHandler");
  delay(5000);
  Serial.println("Setup");

  if (!_sd.begin(SD_CONFIG))
  {
    _sd.initErrorHalt(&Serial);
    Serial.println("SD card initialization failed!");
    return;
  }

  uint32_t size = _sd.card()->sectorCount();
  if (size == 0)
  {
    Serial.println("Can't determine the card size.\n");
    return;
  }

  Serial.print("Card size: ");
  Serial.print(size / 1024);
  Serial.println(" MB");
}

file_t CardHandler::getFile(const char *filename)
{
  file_t file;

  if (!file.open(filename, O_APPEND | O_WRITE | O_CREAT))
  {
    _sd.errorHalt(&Serial, F("open failed"));
    return file_t{};
  }
  return file;
}