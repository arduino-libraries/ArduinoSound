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
    #ifdef ESP_PLATFORM
	    rms_16b((uint16_t*)buffer,  size / 2, (uint16_t*)&analysis);
    #else
      arm_rms_q15((q15_t*)buffer, size / 2, (q15_t*)&analysis);
    #endif
  } else if (_bitsPerSample == 32) {
    #ifdef ESP_PLATFORM
      rms_32b((uint32_t*)buffer,  size / 4, (uint32_t*)&analysis);
    #else
      arm_rms_q31((q31_t*)buffer, size / 4, (q31_t*)&analysis);
    #endif
  }

  _analysis = analysis;
  _available = 1;
}

#ifdef ESP_PLATFORM
void rms_16b(uint16_t* buffer, uint32_t blockSize, uint16_t* analysis){
  uint32_t sum = 0;		                           /* accumulator */
  uint16_t in;                                   /* temporary variable to store the input value */
  uint32_t blkCnt;                               /* loop counter */

  /* Loop over blockSize number of values */
	blkCnt = blockSize;

	while(blkCnt > 0u)
	{
	  /* C = (A[0] * A[0] + A[1] * A[1] + ... + A[blockSize-1] * A[blockSize-1]) */
	  /* Compute sum of the squares and then store the results in a temporary variable, sum */
	  in = *buffer++;
	  sum += (in * in);

	  /* Decrement the loop counter */
	  blkCnt--;
	}

	*analysis = (uint16_t)sqrt(sum / blockSize);
}

void rms_32b(uint32_t* buffer, uint32_t blockSize, uint32_t* analysis){
  uint64_t sum = 0;                              /* accumulator */
  uint32_t in;                                   /* temporary variable to store the input value */
  uint32_t blkCnt;                               /* loop counter */

  /* Loop over blockSize number of values */
  blkCnt = blockSize;

  while(blkCnt > 0u)
  {
    /* C = (A[0] * A[0] + A[1] * A[1] + ... + A[blockSize-1] * A[blockSize-1]) */
    /* Compute sum of the squares and then store the results in a temporary variable, sum */
    in = *buffer++;
    sum += (in * in);

    /* Decrement the loop counter */
    blkCnt--;
  }

  *analysis = (uint16_t)sqrt(sum / blockSize);
}
#endif
