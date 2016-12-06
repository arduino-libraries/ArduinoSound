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

#include "AudioIn.h"

class AudioInI2SClass : public AudioIn
{
public:
  AudioInI2SClass();
  virtual ~AudioInI2SClass();

  int begin(long sampleRate, int bitsPerSample);
  virtual void end();

  virtual long sampleRate();
  virtual int bitsPerSample();
  virtual int channels();

protected:
  virtual int begin();
  virtual int read(void* buffer, size_t size);
  virtual int reset();

private:
  void onReceive();

  static void onI2SReceive();

private:
  long _sampleRate;
  int _bitsPerSample;
  bool _callbackTriggered;
};

extern AudioInI2SClass AudioInI2S;

#endif
