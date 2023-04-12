#include "FS.h"
#include "SPI.h"
#include <SdFat.h>
#include "Wire.h"
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
  Serial.println("CardHandler Setup");

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

file_t CardHandler::getFile(const char *filename, uint8_t oflag)
{
  #if SD_FAT_TYPE == 0
    File file;
  #elif SD_FAT_TYPE == 1
    File32 file;
  #elif SD_FAT_TYPE == 2
    ExFile file;
  #elif SD_FAT_TYPE == 3
    FsFile file;
  #else // SD_FAT_TYPE
  #error Invalid SD_FAT_TYPE
  #endif // SD_FAT_TYPE

  // Open or create
  if (!file.open(filename, oflag))
  {
    _sd.errorHalt(&Serial, "Failed to open file for writing");
    return file_t();
  }

  // Return
  return file;
}

void CardHandler::removeFile(const char *filename)
{
  if (!_sd.remove(filename))
  {
    _sd.errorHalt(&Serial, "Failed to remove file");
  }
}
