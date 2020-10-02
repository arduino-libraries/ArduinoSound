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
  #ifdef ESP_PLATFORM
    //Serial.print("ESP read available= "); Serial.println((int)i2s_available((i2s_port_t)_esp32_i2s_port_number));
    //esp_err_t i2s_read(i2s_port_t i2s_num, void *dest, size_t size, size_t *bytes_read, TickType_t ticks_to_wait)ů
    size_t buffer_byte_size = _length / 2;
    uint8_t data_buffer[buffer_byte_size];
    size_t bytes_read;
    i2s_read((i2s_port_t) _esp32_i2s_port_number, data_buffer, buffer_byte_size, &bytes_read, 0);
    //Serial.print("ESP read available: bytes_read= "); Serial.println((int)bytes_read);
    if(bytes_read > 0){
      Serial.print("ESP read available: bytes_read= "); Serial.println((int)bytes_read);
      update(data_buffer, bytes_read);
    }
  #endif
  return _available;
}

int FFTAnalyzer::read(int spectrum[], int size)
{
  Serial.print("FFT read() ");
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
  Serial.println("FFT::update()");

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
  #ifdef ESP_PLATFORM
    if (_bitsPerSample == 16) {
      dsps_fft2r_sc16_ae32((int16_t*)_sampleBuffer, _length); // FFT using 16-bit fixed point optimized for ESP32

      dsps_mul_f32_ae32((float *) _fftBuffer, (float *) _fftBuffer+sizeof(uint16_t), (float *)_spectrumBuffer, _length/2,  2,  2, 1);
    } else {
      float float_buffer[_length];
      ieee_float_array((uint32_t*)_fftBuffer, _length, float_buffer);

      dsps_fft2r_fc32_ae32(float_buffer, _length); // FFT using 32-bit floating point optimized for ESP32

      dsps_mul_f32_ae32(float_buffer, &float_buffer[1] , (float*) _spectrumBuffer, _length/2,  2,  2, 1);
      //dsps_mul_f32_ae32(ieee_float_array((uint32_t*)_fftBuffer, _length), ieee_float_array((uint32_t*)_fftBuffer+sizeof(uint32_t), _length-1), (float*) _spectrumBuffer, _length/2,  2,  2, 1);
    }
  #else
    if (_bitsPerSample == 16) {
      arm_rfft_q15(&_S15, (q15_t*)_sampleBuffer, (q15_t*)_fftBuffer);

      arm_cmplx_mag_q15((q15_t*)_fftBuffer, (q15_t*)_spectrumBuffer, _length);
    } else {
      arm_rfft_q31(&_S31, (q31_t*)_sampleBuffer, (q31_t*)_fftBuffer);

      arm_cmplx_mag_q31((q31_t*)_fftBuffer, (q31_t*) _spectrumBuffer, _length);
    }
  #endif // #ifdef ESP_PLATFORM
  _available = 1;
}

void FFTAnalyzer::ieee_float_array(uint32_t* input, int length, float* output){
    memcpy(output, input, sizeof(float)*length);

}
