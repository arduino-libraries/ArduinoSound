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
  _callbackTriggered(true)
  #if defined ESP_PLATFORM
    , _esp32_i2s_port_number(0)
  #endif
{
}

AudioInI2SClass::~AudioInI2SClass()
{
}


#if defined ESP_PLATFORM && defined ESP32
  int AudioInI2SClass::begin(long sampleRate/*=44100*/, int bitsPerSample/*=16*/, const int bit_clock_pin/*=26*/, const int word_select_pin/*=25*/, const int data_out_pin/*=22*/, const int esp32_i2s_port_number/*=0*/)
  {
    _esp32_i2s_port_number = esp32_i2s_port_number;
#elif defined ESP_PLATFORM && defined ESP32S2
  int AudioInI2SClass::begin(long sampleRate/*=44100*/, int bitsPerSample/*=16*/, const int bit_clock_pin/*=26*/, const int word_select_pin/*=25*/, const int data_out_pin/*=22*/)
  {
    static const i2s_config_t i2s_config = {
	  .mode = I2S_MODE_SLAVE | I2S_MODE_RX,
	  .sample_rate = sampleRate, // default 44100,
	  .bits_per_sample = bitsPerSample, // default 16,
	  .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
	  .communication_format = I2S_COMM_FORMAT_STAND_I2S,
	  .intr_alloc_flags = 0, // default interrupt priority
	  .dma_buf_count = 8,
	  .dma_buf_len = 64,
	  .use_apll = false
	};
	static const i2s_pin_config_t pin_config = {
	    .bck_io_num = bit_clock_pin,
	    .ws_io_num = word_select_pin,
	    .data_out_num = I2S_PIN_NO_CHANGE,
	    .data_in_num = data_in_pin
	};
	if (!i2s_driver_install((i2s_port_t) _esp32_i2s_port_number, &i2s_config, 0, NULL){
		return 0;
	}
	i2s_set_pin(i2s_port_t) esp32_i2s_port_number, &i2s_pin_config);
  i2s_set_sample_rates((i2s_port_t) _esp32_i2s_port_number, 22050); //set sample rates
#else
  int AudioInI2SClass::begin(long sampleRate, int bitsPerSample)
  {
    if (!I2S.begin(I2S_PHILIPS_MODE, sampleRate, bitsPerSample)) {
      return 0;
    }
#endif // ifdef ESP_PLATFORM

  _sampleRate = sampleRate;
  _bitsPerSample = bitsPerSample;


  #ifdef ESP_PLATFORM
   // TODO
  #else
    // add the receiver callback
    I2S.onReceive(AudioInI2SClass::onI2SReceive);

    // trigger a read to kick things off
    I2S.read();
  #endif

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
  return 2;
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
  	i2s_read((i2s_port_t) _esp32_i2s_port_number, buffer, (size_t) size, (size_t*) &read, 0);
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
    #ifdef ESP_PLATFORM
      size_t length = 1; // TODO : ESP does not have availability info - decide the constant
    #else
      size_t length = I2S.available();
    #endif
    uint8_t data[length];

    read(data, length);
  }
}

void AudioInI2SClass::onI2SReceive()
{
  AudioInI2S.onReceive();
}

AudioInI2SClass AudioInI2S;
