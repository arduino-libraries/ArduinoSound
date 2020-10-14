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
#include "Arduino.h"// only for debug prints

AudioInI2SClass::AudioInI2SClass() :
  _sampleRate(-1),
  _bitsPerSample(-1),
  _callbackTriggered(true)
{
}

AudioInI2SClass::~AudioInI2SClass()
{
}


#if defined ESP_PLATFORM
  #if defined ESP32
    int AudioInI2SClass::begin(long sampleRate/*=44100*/, int bitsPerSample/*=16*/, const int bit_clock_pin/*=5*/, const int word_select_pin/*=25*/, const int data_in_pin/*=26*/ /*, const bool use_adc=true*/, const int esp32_i2s_port_number/*=0*/)
    {
      _esp32_i2s_port_number = esp32_i2s_port_number;
  #elif defined ESP32S2
    int AudioInI2SClass::begin(long sampleRate/*=44100*/, int bitsPerSample/*=16*/, const int bit_clock_pin/*=5*/, const int word_select_pin/*=25*/, const int data_in_pin/*=26*//*, const bool use_adc=true*/)
    {
  #endif //ESP 32 or 32S2
    int i2s_mode = I2S_MODE_SLAVE | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN;
    //if(use_adc == true){i2s_mode |= I2S_MODE_ADC_BUILT_IN;}
    static const i2s_config_t i2s_config = {
	  .mode = (i2s_mode_t) i2s_mode ,
	  .sample_rate =  sampleRate, // default 44100,
	  .bits_per_sample = (i2s_bits_per_sample_t) bitsPerSample, // default 16,
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
	Serial.println("AudioInI2SClass::begin : installing...");
	if (ESP_OK != i2s_driver_install((i2s_port_t) _esp32_i2s_port_number, &i2s_config, 0, NULL)){
	  Serial.println("AudioInI2SClass::begin : install failed");
		return 0;
	}
	Serial.println("AudioInI2SClass::begin : install Ok; setting pins...");
	i2s_set_pin((i2s_port_t) _esp32_i2s_port_number, &pin_config);
	Serial.println("AudioInI2SClass::begin : pins set; setting sample rates...");
  i2s_set_sample_rates((i2s_port_t) _esp32_i2s_port_number, 22050); //set sample rates
  Serial.println("AudioInI2SClass::begin : sample rates set - trigger read...");
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
    // dummy read outputs
    size_t bytes_read;
    uint8_t dummy_buffer;

    // add the receiver callback
    /*
    Serial.println("in lib: setup on receive");Serial.flush();
    esp_err_t ret = i2s_on_receive((i2s_port_t) esp32_i2s_port_number, &(AudioInI2SClass::onI2SReceiveStatic));
    if (ret == ESP_ERR_INVALID_ARG){ Serial.println("ESP_ERR_INVALID_ARG if the combination of arguments is invalid.");}
    else if (ret == ESP_ERR_NOT_FOUND){ Serial.println("ESP_ERR_NOT_FOUND No free interrupt found with the specified flags");}
    else if (ret == ESP_OK){ Serial.println("ESP_OK i2s_on_receive setup OK");}
    else{ Serial.println("i2s_on_receive returned Unknown Error");}
    Serial.flush();
    Serial.println("foo");
    */

    // trigger a read to kick things off
    i2s_read((i2s_port_t) esp32_i2s_port_number, &dummy_buffer, 0, &bytes_read, 0); // do we really need this?
    Serial.println("AudioInI2SClass::begin : after read; return 1; // OK");
  #else
    // add the receiver callback
    I2S.onReceive(&(AudioInI2SClass::onI2SReceive));

    // trigger a read to kick things off
    I2S.read();
  #endif // #ifdef ESP_PLATFORM

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
  Serial.println(" void AudioInI2SClass::read()");
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
  Serial.println(" void AudioInI2SClass::onReceive()");
  if (_callbackTriggered) {
    #ifdef ESP_PLATFORM
    /*
      Serial.println("   if (_callbackTriggered) call ESP i2s_available");
      length = i2s_available((i2s_port_t) _esp32_i2s_port_number);
      Serial.print("available for read (len) ="); Serial.println(length);
      */
    #else
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

// Compatibility workaround for C code
void AudioInI2SClass::onI2SReceiveStatic(void* arg)
{
  reinterpret_cast<AudioInI2SClass*>(arg)->onReceive();
}

/*
int AudioInI2SClass::ESP32Available(){
  size_t buffer_size = 64;
  uint32_t data_buffer[buffer_size];
  size_t bytes_read;
  i2s_read((i2s_port_t) _esp32_i2s_port_number, data_buffer, buffer_size, &bytes_read, 0);
}
*/
AudioInI2SClass AudioInI2S;
