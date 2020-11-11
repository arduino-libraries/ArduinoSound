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

#include "AudioOutI2S.h"
#include <Arduino.h> // only for debug outputs

  AudioOutI2SClass::AudioOutI2SClass() :
  _input(NULL),
  _loop(false),
  _paused(false)
{
}

AudioOutI2SClass::~AudioOutI2SClass()
{
}

int AudioOutI2SClass::canPlay(AudioIn& input)
{
  int bitsPerSample = input.bitsPerSample();
  int channels = input.channels();

  if (bitsPerSample != 8 && bitsPerSample != 16 && bitsPerSample != 32) {
    return 0;
  }

  if (channels != 1 && channels != 2) {
    return 0;
  }

  return 1;
}

#if defined ESP_PLATFORM
  #if defined ESP32
    int AudioOutI2SClass::outBegin(long sampleRate/*=44100*/, int bitsPerSample/*=16*/, const int bit_clock_pin/*=5*/, const int word_select_pin/*=25*/, const int data_out_pin/*=35*/, const int esp32_i2s_port_number/*=0*/){
      _esp32_i2s_port_number = esp32_i2s_port_number;
  #elif defined ESP_PLATFORM && defined ESP32S2
    int AudioOutI2SClass::outBegin(long sampleRate/*=44100*/, int bitsPerSample/*=16*/, const int bit_clock_pin/*=5*/, const int word_select_pin/*=25*/, const int data_out_pin/*=35*/) :
  #endif // ESP chip model
    i2s_driver_uninstall((i2s_port_t) _esp32_i2s_port_number); //stop & destroy i2s driver

    i2s_bits_per_sample_t bits;
      if(bitsPerSample <=8)                         bits = I2S_BITS_PER_SAMPLE_8BIT;
      if(bitsPerSample > 8  && bitsPerSample <= 16) bits = I2S_BITS_PER_SAMPLE_16BIT;
      if(bitsPerSample > 16 && bitsPerSample <= 24) bits = I2S_BITS_PER_SAMPLE_24BIT;
      if(bitsPerSample > 24)                        bits = I2S_BITS_PER_SAMPLE_32BIT;

    i2s_mode_t i2s_mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX);
    //if(use_dac == true){i2s_mode |= I2S_MODE_DAC_BUILT_IN;}

    static const i2s_config_t i2s_config = {
          .mode = i2s_mode,
          .sample_rate = sampleRate, // default 44100
          .bits_per_sample = bits, // default 16
          .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
          .communication_format = I2S_COMM_FORMAT_STAND_MAX,
          .intr_alloc_flags = 0, // default interrupt priority
          .dma_buf_count = 8,
          .dma_buf_len = 64,
          .use_apll = false
      };
      static const i2s_pin_config_t pin_config = {
          .bck_io_num = bit_clock_pin,
          .ws_io_num = word_select_pin,
          .data_out_num = data_out_pin,
          .data_in_num = I2S_PIN_NO_CHANGE
      };
      int ret = i2s_driver_install((i2s_port_t) _esp32_i2s_port_number, &i2s_config, 0, NULL);   //install and start i2s driver
      if(ret != ESP_OK){
        return 0;
      }
      ret = i2s_set_pin((i2s_port_t) _esp32_i2s_port_number, &pin_config);
      if(ret != ESP_OK){
        return 0;
      }

      return 1; // OK
  }
#endif  // ESP_PLATFORM

int AudioOutI2SClass::play(AudioIn& input)
{
  return startPlayback(input, false);
}

int AudioOutI2SClass::loop(AudioIn& input)
{
  return startPlayback(input, true);
}

int AudioOutI2SClass::pause()
{
  if (!isPlaying()) {
    return 0;
  }

  _paused = true;

  return 1;
}

int AudioOutI2SClass::resume()
{
  if (!_paused) {
    return 0;
  }

  _paused = false;

  #ifdef ESP_PLATFORM
    size_t length = 64;
  #else
    size_t length = I2S.availableForWrite();
  #endif

  // play some silence to get things going
  uint8_t silence[length];
  memset(silence, 0x00, length);

  #ifdef ESP_PLATFORM
    i2s_write((i2s_port_t) _esp32_i2s_port_number, silence, length, NULL, 100);
    i2s_write((i2s_port_t) _esp32_i2s_port_number, silence, length, NULL, 100);
  #else
    I2S.write(silence, length);
    I2S.write(silence, length);
  #endif

  return 1;
}

int AudioOutI2SClass::stop()
{
  if (!_input) {
    return 0;    
  }

  endInput(_input);
  _input = NULL;

  #ifdef ESP_PLATFORM
    i2s_driver_uninstall((i2s_port_t) _esp32_i2s_port_number); //stop & destroy i2s driver
  #else
    I2S.end();
  #endif

  return 1;
}

int AudioOutI2SClass::isPlaying()
{
  return !_paused && (_input != NULL);
}

int AudioOutI2SClass::isPaused()
{
  return _paused;
}

#ifdef I2S_HAS_SET_BUFFER_SIZE
void AudioOutI2SClass::setBufferSize(int bufferSize)
{
  I2S.setBufferSize(bufferSize);
}
#endif

int AudioOutI2SClass::startPlayback(AudioIn& input, bool loop)
{
  if (_input) {
    stop();
  }
#ifdef ESP_PLATFORM
  if (!outBegin(input.sampleRate(), input.bitsPerSample())) {
    return 0;
  }

  // TODO register on transmit callback / isr

  if (!beginInput(&input)) {
	i2s_driver_uninstall((i2s_port_t) _esp32_i2s_port_number); //stop & destroy i2s driver
	  Serial.println("Error");
    return 0;
  }
  _input = &input;
  _loop = loop;

  size_t length = 8;
  uint8_t silence[length];
  memset(silence, 0x00, length);

  size_t bytes_written;
#else
  I2S.onTransmit(AudioOutI2SClass::onI2STransmit);

  if (!I2S.begin(I2S_PHILIPS_MODE, input.sampleRate(), input.bitsPerSample())) {
    return 0;
  }

  if (!beginInput(&input)) {
    I2S.end();
    return 0;
  }

  _input = &input;
  _loop = loop;

  size_t length = I2S.availableForWrite();
  uint8_t silence[length];
  memset(silence, 0x00, length);

  I2S.write(silence, length);
  I2S.write(silence, length);
#endif
  return 1;
}

void AudioOutI2SClass::onTransmit()
{
  if (!_input || _paused) {
    return;
  }

  int channels = _input->channels();
#ifdef ESP_PLATFORM
  size_t length = 1024;
#else
  size_t length = I2S.availableForWrite();
#endif
  uint8_t data[length];

  if (channels == 1) {
    length /= 2;
  }

  int n = readInput(_input, data, length);
  if (n == 0) {
    if (!_loop) {
      // non-looped playback, we are done
      stop();
      return;
    }

    // reset the input
    if (!resetInput(_input)) {
      #ifdef ESP_PLATFORM
	    i2s_driver_uninstall((i2s_port_t) _esp32_i2s_port_number); //stop & destroy i2s driver
      #else
        I2S.end();
      #endif
      return;
    }

    // read the input (again)
    n = readInput(_input, data, length);
  }

  if (channels == 1) {
    monoToStereo(data, n, _input->bitsPerSample());

    n *= 2;
  }
#ifdef ESP_PLATFORM
  size_t bytes_written;
  i2s_write((i2s_port_t) _esp32_i2s_port_number, data, n, &bytes_written, 100);
#else
  I2S.write(data, n);
#endif
}

void AudioOutI2SClass::onI2STransmit()
{
  AudioOutI2S.onTransmit();
}

void AudioOutI2SClass::transmit()
{
  onTransmit();
}

AudioOutI2SClass AudioOutI2S;
