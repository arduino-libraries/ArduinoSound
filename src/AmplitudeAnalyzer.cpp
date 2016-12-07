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

#include "AmplitudeAnalyzer.h"

AmplitudeAnalyzer::AmplitudeAnalyzer() :
  _bitsPerSample(-1),
  _available(0),
  _analysis(0)
{
}

AmplitudeAnalyzer::~AmplitudeAnalyzer()
{
}

int AmplitudeAnalyzer::available()
{
  return _available;
}

int AmplitudeAnalyzer::read()
{
  if (_available) {
    _available = 0;
    return _analysis;
  }

  return 0;
}

int AmplitudeAnalyzer::configure(AudioIn* input)
{
  int bitsPerSample = input->bitsPerSample();

  if (bitsPerSample != 16 && bitsPerSample != 32) {
    return 0;
  }

  _bitsPerSample = bitsPerSample;

  return 1;
}

void AmplitudeAnalyzer::update(const void* buffer, size_t size)
{
  int analysis = 0;

  if (_bitsPerSample == 16) {
    arm_rms_q15((q15_t*)buffer, size / 2, (q15_t*)&analysis);
  } else if (_bitsPerSample == 32) {
    arm_rms_q31((q31_t*)buffer, size / 4, (q31_t*)&analysis);
  }

  _analysis = analysis;
  _available = 1;
}
