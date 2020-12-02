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

#ifndef _FFT_ANALYZER_H_INCLUDED
#define _FFT_ANALYZER_H_INCLUDED

#ifdef ESP_PLATFORM
  #include "esp_dsp.h"
  #include "driver/i2s.h"
  #include <cmath>
  typedef uint16_t q15_t;
  typedef uint32_t q31_t;
#else
  #define ARM_MATH_CM0PLUS
  #include <arm_math.h>
#endif // #ifdef ESP_PLATFORM

#include "AudioAnalyzer.h"
#include <cstring>

class FFTAnalyzer : public AudioAnalyzer
{
public:
  FFTAnalyzer(int length);
  virtual ~FFTAnalyzer();

  int available();
  int read(int spectrum[], int size); // original
  int readFloat(float spectrum[], int size);

protected:
  virtual int configure(AudioIn* input);
  virtual void update(const void* buffer, size_t size);
  void real_int16_to_complex_int16(int16_t* input, int length, int16_t* output);
  void real_uint32_to_complex_float(uint32_t* input, int length, float* output);
  void float_cmplx_mag(float *pSrc, float *pDst, uint32_t numSamples);
  void int16_cmplx_mag(int16_t *pSrc, float *pDst, uint32_t numSamples);

private:
  int _length;
  int _bitsPerSample;
  int _channels;

  #ifndef ESP_PLATFORM
    arm_rfft_instance_q15 _S15;
    arm_rfft_instance_q31 _S31;
  #endif // #ifndef ESP_PLATFORM

  int _available;
  void* _sampleBuffer;
  int _sampleBufferSize;
  void* _fftBuffer;
  void* _spectrumBuffer;
  #ifdef ESP_PLATFORM
    uint8_t* _data_buffer;
    AudioIn* _input;
  #endif
};

#endif // #ifndef _FFT_ANALYZER_H_INCLUDED
