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

#include <Arduino.h>

#ifdef ESP_PLATFORM
  #include "esp_dsp.h"
  typedef uint16_t q15_t;
  typedef uint32_t q31_t;
#else
  #define ARM_MATH_CM0PLUS
  #include <arm_math.h>
#endif

#include "AudioAnalyzer.h"

class FFTAnalyzer : public AudioAnalyzer
{
public:
  FFTAnalyzer(int length);
  virtual ~FFTAnalyzer();

  int available();
  int read(int spectrum[], int size);

protected:
  virtual int configure(AudioIn* input);
  virtual void update(const void* buffer, size_t size);
  float* ieee_float_array(uint32_t* f, int length);

private:
  int _length;
  int _bitsPerSample;
  int _channels;

  #ifdef ESP_PLATFORM
    // TODO modify structures to fit needs of ESP-DPS
    // https://www.keil.com/pack/doc/CMSIS/DSP/html/structarm__rfft__instance__q15.html
    typedef struct _cfft_instance_q15 {
      uint16_t   fftLen;
      const uint16_t *   pTwiddle;
      const uint16_t *  pBitRevTable;
      uint16_t  bitRevLength;
    } cfft_instance_q15;
    typedef struct _fft_instance_16b {
      uint32_t   fftLenReal;
      uint8_t   ifftFlagR;
      uint8_t   bitReverseFlagR;
      uint32_t  twidCoefRModifier;
      const uint16_t *   pTwiddleAReal;
      const uint16_t *   pTwiddleBReal;
      const cfft_instance_q15 *   pCfft;
    } fft_instance_16b;
    fft_instance_16b _S15;

    typedef struct _cfft_instance_q31 {
      uint16_t   fftLen;
      const uint32_t *  pTwiddle;
      const uint16_t *  pBitRevTable;
      uint16_t  bitRevLength;
    } cfft_instance_q31;
    typedef struct _fft_instance_32b {
      uint32_t   fftLenReal;
      uint8_t   ifftFlagR;
      uint8_t   bitReverseFlagR;
      uint32_t  twidCoefRModifier;
      const uint32_t *   pTwiddleAReal;
      const uint32_t *   pTwiddleBReal;
      const cfft_instance_q31 *   pCfft;
    } fft_instance_32b;
    fft_instance_32b _S31;

  #else
    arm_rfft_instance_q15 _S15;
    arm_rfft_instance_q31 _S31;
  #endif

  int _available;
  void* _sampleBuffer;
  int _sampleBufferSize;
  void* _fftBuffer;
  void* _spectrumBuffer;
};

#endif
