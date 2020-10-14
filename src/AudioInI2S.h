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

#ifndef _AUDIO_IN_I2S_H_INCLUDED
#define _AUDIO_IN_I2S_H_INCLUDED

#ifdef ESP_PLATFORM
  #include "driver/i2s.h"
#else
  #include <I2S.h>
#endif

#include "AudioIn.h"

class AudioInI2SClass : public AudioIn
{
public:
  AudioInI2SClass();
  virtual ~AudioInI2SClass();

  #if defined ESP_PLATFORM && defined ESP32
	 int begin(long sampleRate=44100, int bitsPerSample=16, const int bit_clock_pin=5, const int word_select_pin=25, const int data_in_pin=26, /*const bool use_adc=true,*/ const int esp32_i2s_port_number=0);
  #elif defined ESP_PLATFORM && defined ESP32S2
   int begin(long sampleRate=44100, int bitsPerSample=16, const int bit_clock_pin=5, const int word_select_pin=25, const int data_in_pin=26/*, const bool use_adc=true*/);
  #else
    int begin(long sampleRate, int bitsPerSample);
    #ifdef I2S_HAS_SET_BUFFER_SIZE
      int begin(long sampleRate, int bitsPerSample, int bufferSize);
    #endif // ifdef I2S_HAS_SET_BUFFER_SIZE
  #endif // ifdef ESP_PLATFORM
  virtual void end();

  virtual long sampleRate();
  virtual int bitsPerSample();
  virtual int channels();

#ifdef I2S_HAS_SET_BUFFER_SIZE
  void setBufferSize(int bufferSize);
#endif

protected:
  virtual int begin();
  virtual int read(void* buffer, size_t size);
  virtual int reset();

private:
  void onReceive();

  static void onI2SReceive();
  static void onI2SReceiveStatic(void * arg);

private:
  long _sampleRate;
  int _bitsPerSample;
  bool _callbackTriggered;
};

extern AudioInI2SClass AudioInI2S;

#endif
