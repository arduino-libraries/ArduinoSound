/*
  Copyright (c) 2016 Arduino LLC. All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
*/

#include "AudioInI2S.h"

AudioInI2SClass::AudioInI2SClass() :
  _sampleRate(-1),
  _bitsPerSample(-1),
  _callbackTriggered(true),
  _initialized(false)
{
}

AudioInI2SClass::~AudioInI2SClass()
{
}


#if defined ESP_PLATFORM
  #if defined ESP32
    int AudioInI2SClass::begin(long sampleRate/*=44100*/, int bitsPerSample/*=16*/, int bit_clock_pin/*=5*/, int word_select_pin/*=25*/, int data_in_pin/*=26*/, int esp32_i2s_port_number/*=0*/)
    {
      _esp32_i2s_port_number = esp32_i2s_port_number;
  #elif defined ESP32S2
    int AudioInI2SClass::begin(long sampleRate/*=44100*/, int bitsPerSample/*=16*/, int bit_clock_pin/*=5*/, int word_select_pin/*=25*/, int data_in_pin/*=26*/)
    {
  #endif //ESP 32 or 32S2
    if(_initialized){
      i2s_driver_uninstall((i2s_port_t) _esp32_i2s_port_number); //stop & destroy i2s driver
      _initialized = false;
    }
    _use_adc = false;

    static i2s_config_t i2s_config = {
	  .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
	  .sample_rate =  sampleRate, // default 44100,
	  .bits_per_sample = (i2s_bits_per_sample_t) bitsPerSample, // default 16,
	  .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_PCM_SHORT,
	  .intr_alloc_flags = 0, // default interrupt priority
	  .dma_buf_count = 8,
	  .dma_buf_len = 64,
	  .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
	};
	static i2s_pin_config_t pin_config = {
	    .bck_io_num = bit_clock_pin,
	    .ws_io_num = word_select_pin,
	    .data_out_num = I2S_PIN_NO_CHANGE,
	    .data_in_num = data_in_pin
	};
	if (ESP_OK != i2s_driver_install((i2s_port_t) _esp32_i2s_port_number, &i2s_config, 0, NULL)){
		return 0;
	}

	if (ESP_OK != i2s_set_pin((i2s_port_t) _esp32_i2s_port_number, &pin_config)){
    return 0;
  }

#else
  int AudioInI2SClass::begin(long sampleRate, int bitsPerSample)
  {
    if (!I2S.begin(I2S_PHILIPS_MODE, sampleRate, bitsPerSample)) {
      return 0;
    }
#endif // ifdef ESP_PLATFORM

  _sampleRate = sampleRate;
  _bitsPerSample = bitsPerSample;

  #ifndef ESP_PLATFORM
    // add the receiver callback
    I2S.onReceive(&(AudioInI2SClass::onI2SReceive));

    // trigger a read to kick things off
    I2S.read();
  #endif // #ifndef ESP_PLATFORM

   _channels = 2;
   _initialized = true;
  return 1;
}

#if defined ESP_PLATFORM
  #if defined ESP32
    int AudioInI2SClass::beginADC(long sampleRate/*=44100*/, int bitsPerSample/*=12*/, int adc_unit/*=1*/, int adc_channel/*=0*/, int esp32_i2s_port_number/*=0*/)
    {
      _esp32_i2s_port_number = esp32_i2s_port_number;
  #elif defined ESP32S2
    int int AudioInI2SClass::beginADC(long sampleRate/*=44100*/, int bitsPerSample/*=12*/, int adc_unit/*=1*/, int adc_channel/*=0*/)
    {
  #endif //ESP 32 or 32S2
    if(_initialized){
      i2s_driver_uninstall((i2s_port_t) _esp32_i2s_port_number); //stop & destroy i2s driver
      _initialized = false;
      //return 0; // ERR
    }
    _use_adc = true;
    _channels = 1; // TODO enable multichannel

    int dma_buf_count = 4;
    static i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN),
    .sample_rate =  sampleRate, // default 44100,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_MSB,
    .intr_alloc_flags = 0, // default interrupt priority
    .dma_buf_count = dma_buf_count, // original
    .dma_buf_len = 1024,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };

  if (ESP_OK != i2s_driver_install((i2s_port_t) _esp32_i2s_port_number, &i2s_config, dma_buf_count, &_i2s_queue)){
    return 0;
  }
  i2s_set_adc_mode((adc_unit_t)adc_unit, (adc1_channel_t)adc_channel);
  i2s_set_pin((i2s_port_t) _esp32_i2s_port_number, NULL);

  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten((adc1_channel_t)adc_channel, ADC_ATTEN_DB_11);

  i2s_adc_enable((i2s_port_t) _esp32_i2s_port_number);

#endif // ifdef ESP_PLATFORM

  _sampleRate = sampleRate;
  _bitsPerSample = bitsPerSample;

  #ifndef ESP_PLATFORM
    // add the receiver callback
    I2S.onReceive(&(AudioInI2SClass::onI2SReceive));

    // trigger a read to kick things off
    I2S.read();
  #endif // #ifndef ESP_PLATFORM

  _initialized = true;
  return 1;
}


#ifdef I2S_HAS_SET_BUFFER_SIZE
int AudioInI2SClass::begin(long sampleRate, int bitsPerSample, int bufferSize)
{
  setBufferSize(bufferSize);

  return begin(sampleRate, bitsPerSample);
}
#endif

void AudioInI2SClass::end()
{
  _sampleRate = -1;
  _bitsPerSample = -1;
  _callbackTriggered = true;
  #ifdef ESP_PLATFORM
    if(_use_adc){
      i2s_adc_disable((i2s_port_t) _esp32_i2s_port_number);
    }
    i2s_driver_uninstall((i2s_port_t) _esp32_i2s_port_number);
  #else
    I2S.end();
  #endif
}

long AudioInI2SClass::sampleRate()
{
  return _sampleRate;
}

int AudioInI2SClass::bitsPerSample()
{
  return _bitsPerSample;
}

int AudioInI2SClass::channels()
{
  return _channels;
}

#ifdef I2S_HAS_SET_BUFFER_SIZE
void AudioInI2SClass::setBufferSize(int bufferSize)
{
  I2S.setBufferSize(bufferSize);
}
#endif

int AudioInI2SClass::begin()
{
  _callbackTriggered = false;

  return 0;
}

int AudioInI2SClass::read(void* buffer, size_t size)
{
  int read;

  #ifdef ESP_PLATFORM
    i2s_read((i2s_port_t) _esp32_i2s_port_number, buffer, (size_t) size, (size_t*) &read, 10);
    if(_use_adc){
      for(int i = 0; i < read / 2; ++i){
        ((uint16_t*)buffer)[i] = ((uint16_t*)buffer)[i] & 0x0FFF;
      }
    }
  #else
    read = I2S.read(buffer, size);
  #endif

  if (read) {
    samplesRead(buffer, read);
  }
  return read;
}

int AudioInI2SClass::reset()
{
  return 0;
}

void AudioInI2SClass::onReceive()
{
  if (_callbackTriggered) {
    #ifndef ESP_PLATFORM
      size_t length;
      length = I2S.available();
      uint8_t data[length];

      read(data, length);
    #endif
  }
}

void AudioInI2SClass::onI2SReceive()
{
  AudioInI2S.onReceive();
}

AudioInI2SClass AudioInI2S;
