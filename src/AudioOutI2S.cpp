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
    Serial.print("clk pin = "); Serial.println(bit_clock_pin);
    Serial.print("ws pin = "); Serial.println(word_select_pin);
    Serial.print("ESP out > codec in pin = "); Serial.println(data_out_pin);

    Serial.print("AudioOutI2SClass::outBegin: sampleRate=");Serial.println(sampleRate);
    Serial.print("AudioOutI2SClass::outBegin: bitsPerSample=");Serial.println(bitsPerSample);

    i2s_driver_uninstall((i2s_port_t) _esp32_i2s_port_number); //stop & destroy i2s driver

    i2s_bits_per_sample_t bits;
      if(bitsPerSample <=8)                         bits = I2S_BITS_PER_SAMPLE_8BIT;
      if(bitsPerSample > 8  && bitsPerSample <= 16) bits = I2S_BITS_PER_SAMPLE_16BIT;
      if(bitsPerSample > 16 && bitsPerSample <= 24) bits = I2S_BITS_PER_SAMPLE_24BIT;
      if(bitsPerSample > 24)                        bits = I2S_BITS_PER_SAMPLE_32BIT;

      Serial.print("AudioOutI2SClass::outBegin: bits enum=");Serial.println(bits);

    i2s_mode_t i2s_mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX);
    //if(use_dac == true){i2s_mode |= I2S_MODE_DAC_BUILT_IN;}

    static const i2s_config_t i2s_config = {
          .mode = i2s_mode,
          .sample_rate = sampleRate, // default 44100,
          .bits_per_sample = bits, // default 16,
          .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
          //.communication_format = I2S_COMM_FORMAT_STAND_I2S, // almost no scrathing noises, but silent in comparison to other formats
          //.communication_format = I2S_COMM_FORMAT_STAND_MSB, // few scrathing noises
          //.communication_format = I2S_COMM_FORMAT_STAND_PCM_SHORT, // few scrathing noises
          //.communication_format = I2S_COMM_FORMAT_STAND_PCM_LONG, // lots scrathing noises
          .communication_format = I2S_COMM_FORMAT_STAND_MAX, // almost no scrathing noises, but silent in comparison to other formats
          .intr_alloc_flags = 0, // default interrupt priority
          .dma_buf_count = 8,
          //.dma_buf_len = 64, // orig
          .dma_buf_len = 256, // orig
          .use_apll = false
          //.use_apll = true
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

      //i2s_set_sample_rates((i2s_port_t) _esp32_i2s_port_number, 22050); //set sample rates
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

  // play some silence to get things going
  #ifdef ESP_PLATFORM
    size_t length = 64; // TODO : ESP does not have availability info - decide the constant
  #else
    size_t length = I2S.availableForWrite();
  #endif

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
  Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
  Serial.println("!!! AudioOutI2SClass::stop() _input = NULL; !!!");
  Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
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
  //Serial.println("AudioOutI2SClass::isPlaying():");
  //Serial.print("!_paused = "); Serial.println(!_paused);
  //Serial.print("_input != NULL = "); Serial.println(_input != NULL);
  //Serial.print("!_paused && (_input != NULL)=");  Serial.println(!_paused && (_input != NULL));
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
    Serial.println("AudioOutI2SClass::startPlayback: something wrong with _input -> stop()");
    stop();
  }
#ifdef ESP_PLATFORM
  //Serial.print("AudioOutI2SClass::startPlayback: check if outBegin ... ");
  if (!outBegin(input.sampleRate(), input.bitsPerSample())) {
    Serial.println("Error");
    return 0;
  }
  //Serial.println("Ok");

  // TODO register on transmit callback / isr

  //Serial.print("AudioOutI2SClass::startPlayback: check beginInput(&input) ... ");
  if (!beginInput(&input)) {
	i2s_driver_uninstall((i2s_port_t) _esp32_i2s_port_number); //stop & destroy i2s driver
	  Serial.println("Error");
    return 0;
  }
  //Serial.println("Ok");
     //erial.print("AudioOutI2SClass::startPlayback(AudioIn& input, bool loop): BEFORE _input = &input; // _input=");
     //Serial.println((int)_input);
  _input = &input;
     //Serial.print("AudioOutI2SClass::startPlayback(AudioIn& input, bool loop): AFTER  _input = &input; // _input=");
     //Serial.println((int)_input);
  _loop = loop;

  size_t length = 8;
  uint8_t silence[length];
  memset(silence, 0x00, length);

  //Serial.println("call i2s_write");
  size_t bytes_written;
  //i2s_write((i2s_port_t) _esp32_i2s_port_number, silence, length, &bytes_written, 100);
  //Serial.println("AudioOutI2SClass::startPlayback: after i2s_write");
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
  //Serial.print("AudioOutI2SClass::onTransmit: ");
  //Serial.print("_input=");
  //Serial.println((int)_input);

  if (!_input || _paused) {
    //Serial.println("do nothing");
    //Serial.print("_input=");Serial.println(_input==NULL ? "NULL" : "exists");
    //Serial.print("_paused=");Serial.println(_paused);
    return;
  }

  int channels = _input->channels();
#ifdef ESP_PLATFORM
  size_t length = 1000; // TODO : ESP does not have availability info - decide the constant
#else
  size_t length = I2S.availableForWrite();
#endif
  uint8_t data[length];

  if (channels == 1) {
    length /= 2;
  }

  //Serial.println("read from input");
  //Serial.print("&data=");Serial.println((int)&data);
  //Serial.print("read from input length=");Serial.println(length);
  int n = readInput(_input, data, length);
  //Serial.print("actualy read n="); Serial.println(n);
  if (n == 0) {
    if (!_loop) {
      // non-looped playback, we are done
      //Serial.println("call stop()");
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
    //Serial.println("read from input (again)");
    n = readInput(_input, data, length);
    //Serial.print("n="); Serial.println(n);
  }

  if (channels == 1) {
    Serial.println("convert from mono to stereo");
    monoToStereo(data, n, _input->bitsPerSample());

    n *= 2;
  }
#ifdef ESP_PLATFORM
  size_t bytes_written;
  //Serial.println("i2s_write");
  //((bits+8)/16)*SAMPLE_PER_CYCLE*4))
  i2s_write((i2s_port_t) _esp32_i2s_port_number, data, n, &bytes_written, 100); // original - distorted audio

  //Serial.print("bytes_written="); Serial.println(bytes_written);
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

// Compatibility workaround for C code
/*
void AudioOutI2SClass::onI2STransmitStatic(void* arg)
{
  reinterpret_cast<AudioOutI2SClass*>(arg)->onTransmit();
}
*/

AudioOutI2SClass AudioOutI2S;
