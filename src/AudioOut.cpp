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

#include <stddef.h>
#include <stdint.h>

#include "AudioOut.h"

AudioOut::AudioOut() :
  _volume(50)
{
}

AudioOut::~AudioOut()
{
}

void AudioOut::volume(float level)
{
  _volume = (level * 1024.0) / 100.0;
}

int AudioOut::beginInput(AudioIn* input)
{
  return input->begin();
}

int AudioOut::readInput(AudioIn* input, void* buffer, size_t size)
{
  int bitsPerSample = input->bitsPerSample();
  int n = input->read(buffer, size);

  if (n > 0) {
    adjustVolume(buffer, n, bitsPerSample);
  }

  return n;
}

int AudioOut::resetInput(AudioIn* input)
{
  return input->reset();
}

void AudioOut::endInput(AudioIn* input)
{
  return input->end();
}

void AudioOut::monoToStereo(void* buffer, size_t size, int bitsPerSample)
{
  int samples = size / (bitsPerSample / 8);

  if (bitsPerSample == 8) {
    uint8_t* s = &((uint8_t*)buffer)[samples - 1];
    uint8_t* d = &((uint8_t*)buffer)[samples * 2 - 1];

    for (int i = 0; i < samples; i++) {
      *d = *s;
      d--;
      *d = *s;
      d--;
      s--;
    }
  } else if (bitsPerSample == 16) {
    int16_t* s = &((int16_t*)buffer)[samples - 1];
    int16_t* d = &((int16_t*)buffer)[samples * 2 - 1];

    for (int i = 0; i < samples; i++) {
      *d = *s;
      d--;
      *d = *s;
      d--;
      s--;
    }
  } else if (bitsPerSample == 32) {
    int32_t* s = &((int32_t*)buffer)[samples - 1];
    int32_t* d = &((int32_t*)buffer)[samples * 2 - 1];

    for (int i = 0; i < samples; i++) {
      *d = *s;
      d--;
      *d = *s;
      d--;
      s--;
    }
  }
}

void AudioOut::adjustVolume(void* buffer, size_t size, int bitsPerSample)
{
  int samples = size / (bitsPerSample / 8);

  if (bitsPerSample == 8) {
    uint8_t* s = (uint8_t*)buffer;

    for (int i = 0; i < samples; i++) {
      *s = (*s * _volume) >> 10;

      s++;
    }
  } else if (bitsPerSample == 16) {
    int16_t* s = (int16_t*)buffer;

    for (int i = 0; i < samples; i++) {
      *s = (*s * _volume) >> 10;

      s++;
    }
  } else if (bitsPerSample == 32) {
    int32_t* s = (int32_t*)buffer;

    for (int i = 0; i < samples; i++) {
      *s = (*s * _volume) >> 10;

      s++;
    }
  }
}
