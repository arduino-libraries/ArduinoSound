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

#ifndef _AMPLITUDE_ANALYZER_H_INCLUDED
#define _AMPLITUDE_ANALYZER_H_INCLUDED

#include <Arduino.h>

#define ARM_MATH_CM0PLUS
#include <arm_math.h>

#include "AudioAnalyzer.h"

class AmplitudeAnalyzer : public AudioAnalyzer
{
public:
  AmplitudeAnalyzer();
  virtual ~AmplitudeAnalyzer();

  int available();
  int read();

protected:
  virtual int configure(AudioIn* input);
  virtual void update(const void* buffer, size_t size);

private:
  int _bitsPerSample;
  int _available;
  int _analysis;
};

#endif
