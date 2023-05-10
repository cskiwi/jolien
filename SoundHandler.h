#ifndef SOUND_HANDLER_H
#define SOUND_HANDLER_H
#include <Arduino.h>

#define I2S_WS GPIO_NUM_22
#define I2S_SD GPIO_NUM_21
#define I2S_SCK GPIO_NUM_26
#define I2S_PORT_NUM I2S_NUM_0
#define I2S_SAMPLE_RATE (44100)
#define I2S_READ_LEN (1024)
#define I2S_NUM_CHANNELS (1)

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
};

class SoundHandler
{
public:
  SoundHandler();
  ~SoundHandler();
  void init();
  uint8_t i2sRead(int16_t *buffer, size_t max_bytes);
  wav_header_t getWavHeader();

  float getDbA(int16_t *buffer, size_t buffer_size);

private:
  bool _connected;
  bool _initialized;
};

#endif // CARD_READER_H
