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
    //Serial.print("ESP read available= "); Serial.println((int)i2s_available((i2s_port_t)_esp32_i2s_port_number));
    //esp_err_t i2s_read(i2s_port_t i2s_num, void *dest, size_t size, size_t *bytes_read, TickType_t ticks_to_wait);
    static size_t bytes_read;

    // Always returns OK
    /*esp_err_t ret =*/ i2s_read((i2s_port_t) _esp32_i2s_port_number, _data_buffer, _length, &bytes_read, 1); // without error
    //Serial.print("i2s_read ret = "); Serial.println(ret);
    //i2s_read((i2s_port_t) _esp32_i2s_port_number, &_data_buffer, _length, &bytes_read, 0); // watchdog triggered
    //Serial.print("ESP read available: bytes_read= "); Serial.println((int)bytes_read);

    if(bytes_read > 0){
      //Serial.println("_data_buffer = ");
/*
      for(int i = 0; i < bytes_read; ++i){ // debug print; TODO remove this block
        if(_data_buffer[i] != 0){
          Serial.print("[");Serial.print(i);Serial.print("]=");Serial.print(_data_buffer[i]); Serial.print(" ");
        }
      }
*/
      //Serial.println("");

      //Serial.println("FFTAnalyzer::available > call update()");
      update(_data_buffer, bytes_read);
    }
  #endif
  return _available;
}

int FFTAnalyzer::read(int spectrum[], int size)
{
  //Serial.print("FFT read() ");
  if (!_available) {
    Serial.println("available == 0");
    return 0;
  }

  if (size > (_length / 2)) {
    size = _length / 2;
  }

  if (_bitsPerSample == 16) {
    #ifdef ESP_PLATFORM
      uint32_t* dst = (uint32_t*)spectrum;
      uint16_t* src = (uint16_t*)_spectrumBuffer;
    #else
      q31_t* dst = (q31_t*)spectrum;
      q15_t* src = (q15_t*)_spectrumBuffer;
    #endif

    for (int i = 0; i < size; i++) {
      *dst++ = *src++;
    }
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
    #ifdef ESP_PLATFORM
      this->_esp32_i2s_port_number = input->get_esp32_i2s_port_number();
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
  //Serial.println("FFT::update()");

/*  Serial.println("input buffer = ");
  for(int i = 0; i < size; ++i){ // debug print; TODO remove this block
    Serial.print(((uint8_t*)buffer)[i]); Serial.print(" ");
  }
  Serial.println("");
*/
  int newSamplesSize = (size / _channels);

  if (newSamplesSize > _sampleBufferSize) {
    // more samples than buffer size, cap
    newSamplesSize = _sampleBufferSize;
  }

  int newSamplesOffset = _sampleBufferSize - newSamplesSize;

  if (newSamplesOffset) {
    //Serial.println("FFT::memmove()");
    // shift over the previous samples
    memmove(_sampleBuffer, ((uint8_t*)_sampleBuffer) + newSamplesSize, newSamplesOffset);
    //Serial.println("after FFT::memmove()");
  }

  uint8_t* newSamples = ((uint8_t*)_sampleBuffer) + newSamplesOffset;
  int samples = size / (_bitsPerSample / 8);

  if (_channels == 2) { // For microphone it's always 2 because its hardcoded ¯\_(ツ)_/¯
    // average the stereo samples to mono
    if (_bitsPerSample == 16) {
      Serial.println("16 bit branch");
      const int16_t *src = (const int16_t*)buffer;
      int16_t* dst = (int16_t*)newSamples;

      for (int i = 0; i < samples; i += 2) {
        *dst = (*src++) / 2;
        *dst += (*src++) / 2;

        dst++;
      }
    } else { // 32 bit
      //Serial.println("else than 16 bit branch");
      const int32_t *src = (const int32_t*)buffer;
      int32_t* dst = (int32_t*)newSamples;

      for (int i = 0; i < samples; i += 2) {
        *dst = *src / 2;
        //Serial.print("*dst = *src / 2 = "); Serial.println(*src / 2);
        src++;
        *dst += *src / 2;
        //Serial.print("*dst += *src / 2; *src / 2 = "); Serial.println(*src / 2);
        src++;
        //Serial.print("final *dst = "); Serial.println(*dst);Serial.println("");
        dst++;
      }
/*
      Serial.println("dst after avaraging = ");
      for(int i = 0; i < samples/2; ++i){ // debug print; TODO remove this block
        Serial.print(((int32_t*)dst)[i]); Serial.print(" ");
      }
      Serial.println("");
*/
    }
  } else {
      Serial.println("FFT:: not 2 channels, just memcpy()");
      memcpy(newSamples, buffer, size);
      //Serial.println("after FFT::memcpy()");
  }
/*
  Serial.println("newSamples = ");
   for(int i = 0; i < size/(_bitsPerSample/8); ++i){ // debug print; TODO remove this block
     Serial.print(((int32_t*)newSamples)[i]); Serial.print(" ");
   }
   Serial.println("");
*/
  #ifdef ESP_PLATFORM
    //Serial.println("FFT:: esp fft computations");
    if (_bitsPerSample == 16) {
      //Serial.println("FFT:: 16bit fft");
      dsps_fft2r_sc16_ae32((int16_t*)_sampleBuffer, _length); // FFT using 16-bit fixed point optimized for ESP32
      //Serial.println("FFT:: 16bit mul");
      dsps_mul_f32_ae32((float *) _fftBuffer, (float *) _fftBuffer+sizeof(uint16_t), (float *)_spectrumBuffer, _length/2,  2,  2, 1);
    } else { // assuming 32 bit input
      float float_buffer[_length];
/*
      Serial.println("_sampleBuffer = ");
      for(int i = 0; i < _sampleBufferSize; ++i){ // debug print; TODO remove this block
        Serial.print(((uint8_t*)_sampleBuffer)[i]); Serial.print(" ");
      }
      Serial.println("");
*/
/*
      Serial.println("");
      Serial.println("_spectrumBuffer = ");
      for(int i = 0; i < size; ++i){ // debug print; TODO remove this block
        Serial.print(((int*)_spectrumBuffer)[i]); Serial.print(" ");
      }
      Serial.println("");
*/

      // convert uint32_t array to float array
      ieee_float_array((uint32_t*)_sampleBuffer, _length, float_buffer);
/*
      Serial.println("");
      Serial.println("float_buffer = ");
      for(int i = 0; i < _length; ++i){ // debug print; TODO remove this block
        //Serial.print(((int*)float_buffer)[i]); Serial.print(" ");
        Serial.print(float_buffer[i]); Serial.print(" ");
      }
      Serial.println("");
*/
      float real_buffer[_length*2];
      real_to_complex(float_buffer, _length, real_buffer); // convert float array with real members to complex array with zero Im
/*
      Serial.println("");
      Serial.println("real_buffer = ");
      for(int i = 0; i < _length*2; ++i){ // debug print; TODO remove this block
        Serial.print(real_buffer[i]); Serial.print(" ");
      }
      Serial.println("");
*/
      //Serial.println("FFT:: 32bit fft");
      //dsps_fft2r_fc32_ae32(float_buffer, _length); // FFT using 32-bit floating point optimized for ESP32
      dsps_fft2r_fc32_ae32(real_buffer, _length); // FFT using 32-bit floating point optimized for ESP32
/*
      Serial.println("");
      Serial.println("after FFT real_buffer = ");
      for(int i = 0; i < _length; ++i){ // debug print; TODO remove this block
        Serial.print("Re"); Serial.print(((int*)real_buffer)[i*2]); Serial.print(" ");
        Serial.print("Im"); Serial.print(((int*)real_buffer)[i*2+1]); Serial.print(" ");
      }
      Serial.println("");
*/
      //Serial.println("FFT:: complex magnitude");
      cmplx_mag(real_buffer, (float*)_spectrumBuffer, _length);
/*
      Serial.println("");
      Serial.println("after complex magnitude _spectrumBuffer = ");
      for(int i = 0; i < _length; ++i){ // debug print; TODO remove this block
        Serial.print(((float*)_spectrumBuffer)[i]); Serial.print(" ");

      }
      Serial.println("");
*/
      //dsps_mul_f32_ae32(float_buffer, &float_buffer[1] , (float*) _spectrumBuffer, _length/2,  2,  2, 1);
      //dsps_mul_f32_ae32(ieee_float_array((uint32_t*)_fftBuffer, _length), ieee_float_array((uint32_t*)_fftBuffer+sizeof(uint32_t), _length-1), (float*) _spectrumBuffer, _length/2,  2,  2, 1);
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

/*
Computes the magnitude of the elements of a complex data vector.
The pSrc points to the source data and pDst points to the where the result should be written.
numSamples specifies the number of complex samples in the input array and the data is stored
in an interleaved fashion (real, imag, real, imag, ...).
The input array has a total of 2*numSamples values; the output array has a total of numSamples values.
*/
void FFTAnalyzer::cmplx_mag(float *pSrc, float *pDst, uint32_t numSamples){
  for (int n = 0; n < numSamples; n++) {
      pDst[n] = sqrt(pow(pSrc[(2*n)+0], 2.0) + pow(pSrc[(2*n)+1], 2.0));
  }
}

// convert float array with real members to complex array with zero Im
// from input[0]=Re[0], input[0]=Re[1], ...
// to output[0]=Re[0], output[1]=Im[0]=0.0, output[2]=Re[1], output[3]=Im[1]=0.0, ...
// length of output is input_length*2
void FFTAnalyzer::real_to_complex(float* input, int input_length, float* output){
  for(int i = 0; i < input_length; ++i){
    output[i*2] = input[i]; // Re
    output[i*2+1] = 0.0; // Im=0
  }
}

void FFTAnalyzer::ieee_float_array(uint32_t* input, int length, float* output){
    memcpy(output, input, sizeof(float)*length);

}
