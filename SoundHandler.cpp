#include "SoundHandler.h"
#include "SPI.h"
#include "Wire.h"

#include <driver/i2s.h>
#include <Arduino.h>

SoundHandler::SoundHandler()
{
}

SoundHandler::~SoundHandler()
{
  i2s_stop(I2S_PORT_NUM);
}

void SoundHandler::init()
{
  Serial.println("i2s init");

  // print the config
  Serial.print("I2S_SAMPLE_RATE: ");
  Serial.println(I2S_SAMPLE_RATE);
  Serial.print("I2S_READ_LEN: ");
  Serial.println(I2S_READ_LEN);
  Serial.print("I2S_NUM_CHANNELS: ");
  Serial.println(I2S_NUM_CHANNELS);

  // install
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

  esp_err_t result_install = i2s_driver_install(I2S_PORT_NUM, &i2s_config, 0, NULL);
  assert(result_install == ESP_OK);

  esp_err_t result_sample = i2s_set_sample_rates(I2S_PORT_NUM, I2S_SAMPLE_RATE); // set sample rates
  assert(result_sample == ESP_OK);

  // pin config
  const i2s_pin_config_t pin_config = {
      .bck_io_num = I2S_SCK,
      .ws_io_num = I2S_WS,
      .data_out_num = I2S_PIN_NO_CHANGE,
      .data_in_num = I2S_SD};

  esp_err_t result_pin = i2s_set_pin(I2S_PORT_NUM, &pin_config);
  assert(result_pin == ESP_OK);

  esp_err_t result_clk = i2s_set_clk(I2S_PORT_NUM, I2S_SAMPLE_RATE, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_MONO);
  assert(result_clk == ESP_OK);

  esp_err_t result_strt = i2s_start(I2S_PORT_NUM);
  assert(result_strt == ESP_OK);
}

uint8_t SoundHandler::i2sRead(int16_t *buffer, size_t max_bytes)
{
  size_t bytes_read;
  i2s_read(I2S_PORT_NUM, buffer, max_bytes, &bytes_read, portMAX_DELAY);
  return bytes_read;
}

wav_header_t SoundHandler::getWavHeader()
{
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

  return wavh;
}

// Calculates the dB(A) value of an audio buffer.
// Assumes that the buffer contains signed 16-bit PCM audio samples.
float SoundHandler::getDbA(int16_t *buffer, size_t buffer_size)
{
  float decibel_sum = 0;
  size_t samples_count = buffer_size / sizeof(int16_t);

  // Calculate the decibel value of each sample.
  for (size_t i = 0; i < samples_count; i++)
  {
    float sample = buffer[i] / 32768.0;
    float decibel = 20.0 * log10(fabs(sample));

    decibel_sum += decibel;
  }

  // Apply A-weighting filter.
  float freqs[] = {20, 25, 31.5, 40, 50, 63, 80, 100, 125, 160, 200, 250, 315, 400, 500, 630, 800, 1000, 1250, 1600, 2000, 2500, 3150, 4000, 5000, 6300, 8000, 10000, 12500, 16000, 20000};
  float coeffs[] = {-39.4, -36.2, -32.5, -29.4, -26.7, -24.4, -22.4, -20.7, -19.1, -17.8, -16.6, -15.6, -14.7, -13.9, -13.2, -12.6, -12.0, -11.5, -11.1, -10.7, -10.4, -10.2, -10.1, -10.0, -10.0, -10.1, -10.2, -10.4, -10.6, -10.8, -10.8};

  float a_weighted_sum = 0;
  for (size_t i = 0; i < samples_count; i++)
  {
    float freq = i * 44100.0 / samples_count;
    size_t j = 0;
    while (j < 29 && freqs[j] < freq)
    {
      j++;
    }
    float coeff = coeffs[j - 1] + (freq - freqs[j - 1]) * (coeffs[j] - coeffs[j - 1]) / (freqs[j] - freqs[j - 1]);

    a_weighted_sum += pow(10.0, 0.1 * (decibel_sum / samples_count + coeff));
  }

  float a_weighted_dB;
  if (decibel_sum <= 0)
  {
    // If decibel_sum is negative or zero, return -inf dB(A).
    a_weighted_dB = -INFINITY;
  }
  else
  {
    a_weighted_dB = 10.0 * log10(a_weighted_sum);
  }

  return a_weighted_dB;
}
