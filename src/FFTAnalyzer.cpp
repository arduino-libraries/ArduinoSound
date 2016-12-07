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

#include "AudioIn.h"

#include "FFTAnalyzer.h"

FFTAnalyzer::FFTAnalyzer(int length) :
  _length(length),
  _bitsPerSample(-1),
  _channels(-1),
  _available(0),
  _sampleBuffer(NULL),
  _fftBuffer(NULL),
  _spectrumBuffer(NULL)
{
}

FFTAnalyzer::~FFTAnalyzer()
{
  if (_sampleBuffer) {
    free(_sampleBuffer);
  }

  if (_fftBuffer) {
    free(_fftBuffer);
  }

  if (_spectrumBuffer) {
    free(_spectrumBuffer);
  }
}

int FFTAnalyzer::available()
{
  return _available;
}

int FFTAnalyzer::read(int spectrum[], int size)
{
  if (!_available) {
    return 0;
  }

  if (size > (_length / 2)) {
    size = _length / 2;
  }

  if (_bitsPerSample == 16) {
    arm_q15_to_q31((q15_t*)_spectrumBuffer, (q31_t*)spectrum, size);
  } else {
    memcpy(spectrum, _spectrumBuffer, sizeof(int) * size);
  }

  _available = 0;

  return size;
}

int FFTAnalyzer::configure(AudioIn* input)
{
  int bitsPerSample = input->bitsPerSample();
  int channels = input->channels();

  if (bitsPerSample != 16 && bitsPerSample != 32) {
    return 0;
  }

  if (channels != 1 && channels != 2) {
    return 0;
  }

  if (bitsPerSample == 16) {
    if (ARM_MATH_SUCCESS != arm_rfft_init_q15(&_S15, _length, 0, 1)) {
      return 0;
    }
  } else {
    if (ARM_MATH_SUCCESS != arm_rfft_init_q31(&_S31, _length, 0, 1)) {
      return 0;
    }
  }

  _bitsPerSample = bitsPerSample;
  _channels = channels;

  if (bitsPerSample == 16) {
    _sampleBufferSize = _length * sizeof(q15_t);
    _sampleBuffer = calloc(_sampleBufferSize, 1);
    _spectrumBuffer = calloc(_length, sizeof(q15_t));
  } else {
    _sampleBufferSize = _length * sizeof(q31_t);
    _sampleBuffer = calloc(_sampleBufferSize, 1);
    _spectrumBuffer = calloc(_length, sizeof(q31_t));
  }

  _fftBuffer = calloc(_sampleBufferSize * 2, 1);

  if (_sampleBuffer == NULL || _fftBuffer == NULL || _spectrumBuffer == NULL) {
    if (_sampleBuffer) {
      free(_sampleBuffer);
      _sampleBuffer = NULL;
    }

    if (_fftBuffer) {
      free(_fftBuffer);
      _fftBuffer = NULL;
    }

    if (_spectrumBuffer) {
      free(_spectrumBuffer);
      _spectrumBuffer = NULL;
    }

    return 0;
  }

  return 1;
}

void FFTAnalyzer::update(const void* buffer, size_t size)
{
  int newSamplesSize = (size / _channels);

  if (newSamplesSize > _sampleBufferSize) {
    // more samples than buffer size, cap
    newSamplesSize = _sampleBufferSize;
  }

  int newSamplesOffset = _sampleBufferSize - newSamplesSize;

  if (newSamplesOffset) {
    // shift over the previous samples
    memmove(_sampleBuffer, ((uint8_t*)_sampleBuffer) + newSamplesSize, newSamplesOffset);
  }

  uint8_t* newSamples = ((uint8_t*)_sampleBuffer) + newSamplesOffset;
  int samples = size / (_bitsPerSample / 8);

  if (_channels == 2) {
    // average the stereo samples to mono
    if (_bitsPerSample == 16) {
      const int16_t *src = (const int16_t*)buffer;
      int16_t* dst = (int16_t*)newSamples;

      for (int i = 0; i < samples; i += 2) {
        *dst = (*src++) / 2;
        *dst += (*src++) / 2;

        dst++;
      }
    } else {
      const int32_t *src = (const int32_t*)buffer;
      int32_t* dst = (int32_t*)newSamples;

      for (int i = 0; i < samples; i += 2) {
        *dst = *src / 2;
        src++;
        *dst += *src / 2;
        src++;

        dst++;
      }
    }
  } else {
    memcpy(newSamples, buffer, size);
  }

  if (_bitsPerSample == 16) {
    arm_rfft_q15(&_S15, (q15_t*)_sampleBuffer, (q15_t*)_fftBuffer);

    arm_cmplx_mag_q15((q15_t*)_spectrumBuffer, (q15_t*)_fftBuffer, _length);
  } else {
    arm_rfft_q31(&_S31, (q31_t*)_sampleBuffer, (q31_t*)_fftBuffer);

    arm_cmplx_mag_q31((q31_t*)_fftBuffer, (q31_t*) _spectrumBuffer, _length);
  }

  _available = 1;
}
