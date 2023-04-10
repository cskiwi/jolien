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

void SoundHandler::begin()
{
  i2s_start(I2S_PORT_NUM);
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
}

uint8_t SoundHandler::i2sRead(int16_t *buffer, size_t max_bytes) {
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
