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
  try
  {
    Serial.println("CardHandler Setup");

    delay(1000);

    if (!_sd.begin(SD_CONFIG))
    {
      Serial.println("SD card initialization failed!");
      _sd.initErrorHalt(&Serial);
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
  catch (const std::exception &e)
  {
    Serial.println("CardHandler Setup Failed");

    Serial.print("Exception: ");
    Serial.println(e.what());
  }
  catch (...)
  {
    Serial.println("CardHandler Setup Failed");
  }
}

void CardHandler::deinit()
{
  _sd.end();
}

file_t CardHandler::getFile(const char *filename, oflag_t oflag)
{
  // Serial.print("Opening file: ");
  // Serial.println(filename);
  
  file_t file;
  if (!file.open(filename, oflag))
  {
    _sd.errorHalt(&Serial, "Failed to open file ");
    return file_t();
  }

  // Return the file
  return file;
}

void CardHandler::removeFile(const char *filename)
{
  if (!_sd.remove(filename))
  {
    _sd.errorHalt(&Serial, "Failed to remove file");
  }
}

void CardHandler::closeFile(file_t file)
{
  file.close();
}
