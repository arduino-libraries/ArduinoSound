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
#include "Arduino.h"// only for debug prints
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
#ifdef ESP_PLATFORM
  if (_data_buffer == NULL){
    _data_buffer = (uint8_t*)malloc(_length);
  }
#endif
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

#ifdef ESP_PLATFORM
  if (_data_buffer) {
      free(_data_buffer);
    }
#endif
}

int FFTAnalyzer::available()
{
  #ifdef ESP_PLATFORM
    int bytes_read;
    bytes_read = _input->read(_data_buffer, _length);
    //bytes_read = _input->read(_data_buffer, 2); // debug - TODO delete
    /*
    Serial.print("FFT_available() bytes_read =");Serial.println(bytes_read);
    for(int i = 0; i < _length/2; ++i){
      Serial.print("[");Serial.print(i);Serial.print("]=");
      Serial.print(((uint16_t*)_data_buffer)[i]);Serial.print("=0x");
      Serial.println(((uint16_t*)_data_buffer)[i],HEX);
    }
    Serial.print("adc=");
    int val = adc1_get_raw(ADC1_CHANNEL_6);
    Serial.print(val);Serial.print("=0x");Serial.println(val,HEX);
    uint16_t sampleA = (uint16_t)(_data_buffer[0])<<8 | _data_buffer[1];
    uint16_t sampleB = (uint16_t)(_data_buffer[1])<<8 | _data_buffer[0];
    if(bytes_read){
      Serial.print("A=");Serial.print(sampleA);Serial.print("=0x");Serial.println(sampleA,HEX);
      Serial.print("B=");Serial.print(sampleB);Serial.print("=0x");Serial.println(sampleB,HEX);
    }
    */
  #endif

  return _available;
}

int FFTAnalyzer::readFloat(float spectrum[], int size){
  memcpy(spectrum, _spectrumBuffer, sizeof(float) * size);
  _available = 0;
  return size;
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
    #ifdef ESP_PLATFORM
      // convert from float to int even if that means often overflowing the int
      for (int i = 0; i < size; i++) {
        spectrum[i] = (long unsigned int)((float*)_spectrumBuffer)[i];
      }
    #else
      q31_t* dst = (q31_t*)spectrum;
      q15_t* src = (q15_t*)_spectrumBuffer;

      for (int i = 0; i < size; i++) {
        *dst++ = *src++;
      }
    #endif
  } else {
    #ifdef ESP_PLATFORM
      // convert from float to int even if that means often overflowing the int
      for (int i = 0; i < size; i++) {
        spectrum[i] = (long unsigned int)((float*)_spectrumBuffer)[i];
        }
    #else
      memcpy(spectrum, _spectrumBuffer, sizeof(int) * size);
    #endif
  }

  _available = 0;

  return size;
}

int FFTAnalyzer::configure(AudioIn* input){
  _input = input;

  int bitsPerSample = input->bitsPerSample();
  int channels = input->channels();

  if (bitsPerSample != 16 && bitsPerSample != 32) {
    return 0;
  }

  if (channels != 1 && channels != 2) {
    return 0;
  }

  if (bitsPerSample == 16) {
    #ifdef ESP_PLATFORM
      // FFT using 16-bit fixed point
      if (ESP_OK != dsps_fft2r_init_sc16(NULL, _length)) {
    #else
      if (ARM_MATH_SUCCESS != arm_rfft_init_q15(&_S15, _length, 0, 1)) {
    #endif
        return 0;
      }
  } else {
    #ifdef ESP_PLATFORM
      // FFT using 32-bit floating point
      if (ESP_OK != dsps_fft2r_init_fc32(NULL, _length)) {
    #else
      //  (struct , length of the FFT, 1=forward transform, 0=enable bit reversal of output)
      if (ARM_MATH_SUCCESS != arm_rfft_init_q31(&_S31, _length, 0, 1)) {
    #endif
        return 0;
      }
  }

  _bitsPerSample = bitsPerSample;
  _channels = channels;

  if (bitsPerSample == 16) {
  #ifdef ESP_PLATFORM
      _sampleBufferSize = _length * sizeof(int16_t);
      _sampleBuffer = calloc(_sampleBufferSize, 1);
      _spectrumBuffer = calloc(_length, sizeof(int16_t));
    } else {
      _sampleBufferSize = _length * sizeof(uint32_t);
      _sampleBuffer = calloc(_sampleBufferSize, 1);
      _spectrumBuffer = calloc(_length, sizeof(uint32_t));
  #else
      _sampleBufferSize = _length * sizeof(q15_t);
      _sampleBuffer = calloc(_sampleBufferSize, 1);
      _spectrumBuffer = calloc(_length, sizeof(q15_t));
    } else {
      _sampleBufferSize = _length * sizeof(q31_t);
      _sampleBuffer = calloc(_sampleBufferSize, 1);
      _spectrumBuffer = calloc(_length, sizeof(q31_t));
  #endif
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

/*
 * 1. Recompute sample size - we only take number of samples disregarding number of channels
 * 2. Shift samples from high addresses to low addresses while dropping old samples on low addresses
 * 3. Make new pointer into sample buffer pointing to start of new samples
 * 4. Based on number of channels (1 or 2) and bits per second (16 or 32) move new samples to sample buffer
 * 5. Based on number of samples (16 or 32) compute Real Fast Fourier Transform followed by Complex Magnitude
 * 6. Set up _available = 1
 */
void FFTAnalyzer::update(const void* buffer, size_t size)
{
  Serial.println("FFT Update; 12b buffer=");
  for(int i = 0; i < _length/2; ++i){
    //Serial.print(((uint16_t*)buffer)[i] & 0x0FFF);Serial.print(" ");
    Serial.print("[");Serial.print(i);Serial.print("]=");
    Serial.print(((uint16_t*)buffer)[i]);Serial.print("=0x");
    Serial.println(((uint16_t*)buffer)[i],HEX);
    //Serial.print(((uint16_t*)buffer)[i] & 0x0FFF);Serial.print("=0x");
    //Serial.println(((uint16_t*)buffer)[i] & 0x0FFF,HEX);
  }
  Serial.println("");
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

  if (_channels == 2) { // For microphone it's always 2 because its hardcoded ¯\_(ツ)_/¯
    // average the stereo samples to mono
    if (_bitsPerSample == 16) {
      const int16_t *src = (const int16_t*)buffer;
      int16_t* dst = (int16_t*)newSamples;

      for (int i = 0; i < samples; i += 2) {

        *dst = (*src++) / 2;

        *dst += (*src++) / 2;

        dst++;
      }
    } else { // 32 bit
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

  Serial.println("FFT Update; _sampleBuffer =");
  for(int i = 0; i < _length; ++i){
    Serial.print(((uint16_t*)_sampleBuffer)[i]);Serial.print(" ");
  }
  Serial.println("");

  #ifdef ESP_PLATFORM
    if (_bitsPerSample == 16) {
      int16_t real_buffer[_length*2];
      real_int16_to_complex_int16((int16_t*)_sampleBuffer, _length, real_buffer);
      dsps_fft2r_sc16_ae32(real_buffer, _length); // FFT using 16-bit fixed point optimized for ESP32
      int16_cmplx_mag(real_buffer, (float*)_spectrumBuffer, _length);
    } else { // assuming 32 bit input
      float real_buffer[_length*2] = {0.0};
      real_uint32_to_complex_float((uint32_t*)_sampleBuffer, _length, real_buffer);
      dsps_fft2r_fc32_ae32(real_buffer, _length); // FFT using 32-bit floating point optimized for ESP32
      float_cmplx_mag(real_buffer, (float*)_spectrumBuffer, _length);
    }
  #else
    if (_bitsPerSample == 16) {
      arm_rfft_q15(&_S15, (q15_t*)_sampleBuffer, (q15_t*)_fftBuffer);

      arm_cmplx_mag_q15((q15_t*)_fftBuffer, (q15_t*)_spectrumBuffer, _length);
    } else {
      //           struct   input( is modified)             output
      arm_rfft_q31(&_S31, (q31_t*)_sampleBuffer, (q31_t*)_fftBuffer);

      // _spectrumBuffer[n] = sqrt(_fftBuffer[(2*n)+0]^2 + _fftBuffer[(2*n)+1]^2);
      arm_cmplx_mag_q31((q31_t*)_fftBuffer, (q31_t*) _spectrumBuffer, _length);
    }
  #endif // #ifdef ESP_PLATFORM
  _available = 1;
}

// convert uint32_t array with real members to complex array with zero Im
// from input[0]=Re[0], input[1]=Re[1], ...
// to output[0]=Re[0], output[1]=Im[0]=0.0, output[2]=Re[1], output[3]=Im[1]=0.0, ...
// length of output is input_length*2
void FFTAnalyzer::real_uint32_to_complex_float(uint32_t* input, int length, float* output){

  for(int i = 0 ; i < length; ++i){
    output[i*2] = (float)input[i]; // Re
    //output[i*2+1] = 0.0; // Im=0
  }
}

// convert uint16_t array with real members to complex array with zero Im
// from input[0]=Re[0], input[1]=Re[1], ...
// to output[0]=Re[0], output[1]=Im[0]=0.0, output[2]=Re[1], output[3]=Im[1]=0.0, ...
// length of output is input_length*2
void FFTAnalyzer::real_int16_to_complex_int16(int16_t* input, int length, int16_t* output){
  for(int i = 0 ; i < length; ++i){
    output[i*2] = input[i]; // Re
    output[i*2+1] = 0; // Im=0
  }
}

/*
Computes the magnitude of the elements of a complex data vector.
The pSrc points to the source data and pDst points to the where the result should be written.
numSamples specifies the number of complex samples in the input array and the data is stored
in an interleaved fashion (real, imag, real, imag, ...).
The input array has a total of 2*numSamples values; the output array has a total of numSamples values.
*/
void FFTAnalyzer::float_cmplx_mag(float *pSrc, float *pDst, uint32_t numSamples){
  for (int n = 0; n < numSamples; n++) {
    pDst[n] = (float)sqrt(pow(pSrc[(2*n)+0], 2.0) + pow(pSrc[(2*n)+1], 2.0));
  }
}
void FFTAnalyzer::int16_cmplx_mag(int16_t *pSrc, float *pDst, uint32_t numSamples){
  for (int n = 0; n < numSamples; n++) {
      pDst[n] = (float)sqrt(pow(pSrc[(2*n)+0], 2.0) + pow(pSrc[(2*n)+1], 2.0));
  }
}
