#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include "WiFiCredentials.h"
#include "I2SMEMSSampler.h"
#include "ADCSampler.h"

WiFiClient *wifiClientADC = NULL;
HTTPClient *httpClientADC = NULL;
WiFiClient *wifiClientI2S = NULL;
HTTPClient *httpClientI2S = NULL;
ADCSampler *adcSampler = NULL;
I2SSampler *i2sSampler = NULL;

// i2s config for using the internal ADC
i2s_config_t adcI2SConfig = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN),
    .sample_rate = 16000,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_I2S_LSB,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 4,
    .dma_buf_len = 1024,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0};

// i2s config for reading from left channel of I2S
i2s_config_t i2sMemsConfigLeftChannel = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = 16000,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,

    //.channel_format = I2S_STD_FORMAT,
    .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_I2S),
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 4,
    .dma_buf_len = 1024,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0};

// i2s pins
i2s_pin_config_t i2sPins = {
    .bck_io_num = 18,
    .ws_io_num = 19,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = 23};

// how many samples to read at once
//const int SAMPLE_SIZE = 16384;
const int SAMPLE_SIZE = 16000*2;


// Task to write samples to our server
void i2sMemsWriterTask(void *param)
{
  I2SSampler *sampler = (I2SSampler *)param;
  int16_t *samples = (int16_t *)malloc(sizeof(uint16_t) * SAMPLE_SIZE);
  //int16_t *samples2 = (int16_t *)malloc(sizeof(uint16_t) * SAMPLE_SIZE);

  if (!samples)
  {
    Serial.println("Failed to allocate memory for samples");
    return;
  }
  while (true)
  {
    int samples_read = sampler->read(samples, SAMPLE_SIZE);
    int sum_read = 0;
    for (int16_t i = 0; i <= samples_read; i++){
      /*if (samples[i]<=0){
        samples[i] = samples[i]*-1;
        sum_read = sum_read + samples[i];
      }
      if (samples[i]<=(sum_read/samples_read)*10){
        samples[i] = 0;
      }
      */

      Serial.println(samples[i]);

    }
    Serial.println(samples_read);
  }
}

void setup()
{
  Serial.begin(115200*2);
  pinMode(2, OUTPUT);
  // Direct i2s input from INMP441 or the SPH0645
  i2sSampler = new I2SMEMSSampler(I2S_NUM_0, i2sPins, i2sMemsConfigLeftChannel, false);
  i2sSampler->start();
  // set up the i2s sample writer task
  TaskHandle_t i2sMemsWriterTaskHandle;
  xTaskCreatePinnedToCore(i2sMemsWriterTask, "I2S Writer Task", 4096, i2sSampler, 1, &i2sMemsWriterTaskHandle, 1);
}

void loop()
{
  // nothing to do here - everything is taken care of by tasks
}