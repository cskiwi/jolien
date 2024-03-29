#ifndef CARD_READER_H
#define CARD_READER_H

#include <FS.h>
#include <SD.h>
#include <SdFat.h>
#include <driver/i2s.h>
#include <Arduino.h>

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
#define SPI_CLOCK SD_SCK_MHZ(1)

#if SD_FAT_TYPE == 0
typedef File file_t;
typedef SdFat sd_t;
#elif SD_FAT_TYPE == 1
typedef SdFat32 sd_t;
typedef File32 file_t;
#elif SD_FAT_TYPE == 2
typedef SdExFat sd_t;
typedef ExFile file_t;
#elif SD_FAT_TYPE == 3
typedef SdFs sd_t;
typedef FsFile file_t;
#else // SD_FAT_TYPE
#error Invalid SD_FAT_TYPE
#endif // SD_FAT_TYPE

class CardHandler
{
public:
  CardHandler();
  void init();
  void deinit();
  file_t getFile(const char *filename, oflag_t oflag = O_APPEND | O_WRITE | O_CREAT);
  void removeFile(const char *filename);
  void closeFile(file_t file);
  bool initialized = false;

private:
  sd_t _sd;
  bool _connected;
};

#endif // CARD_READER_H
