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

#define ARM_MATH_CM0PLUS
#include <arm_math.h>

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

private:
  int _length;
  int _bitsPerSample;
  int _channels;

  arm_rfft_instance_q15 _S15;
  arm_rfft_instance_q31 _S31;

  int _available;
  void* _sampleBuffer;
  int _sampleBufferSize;
  void* _fftBuffer;
  void* _spectrumBuffer;
};

#endif
