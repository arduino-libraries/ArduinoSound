/*
 * Generate Sine wave at 1250Hz to simulate whistling sound recorded with 12bit ADC
 * Process those data by FFT and detect peak at 1250Hz
 */

#include "Arduino.h"
#include "esp_dsp.h"
#include <cmath>

const int sampleRate = 44100;
const int fftSize = 128;

// frequency of whistle to detect
const int whistleFrequency = 1250;

// map whistle frequency to FFT bin
const int whistleBin = (whistleFrequency * fftSize / sampleRate);

////////////////////////////////////////////////////////////////
// Sine wave generator
typedef unsigned long phase_t;
double maxphase = (double)((phase_t)0-(phase_t)1)+1.0;
double fs = sampleRate;
phase_t hz_to_delta( double hz ){
  return maxphase*hz/fs+0.5;
}

float sample_phase( phase_t phase ){
  return sin( phase/maxphase*M_TWOPI );
}

void get_wave_samples(int16_t *samples, int num_of_samples){
  static phase_t delta, iphase = 0;
  delta = hz_to_delta( 1250.0 );
  for(long i=0; i<num_of_samples; ++i ){
    float sample = 2048*sample_phase( iphase += delta );
    samples[i] = (int16_t)(sample + 0xfff/2);
  }
}
////////////////////////////////////////////////////////////////

void setup() {
  // setup the serial
  Serial.begin(115200);
  if (ESP_OK != dsps_fft2r_init_sc16(NULL, fftSize)){
    Serial.println("ERROR initializing FFT");
    while(1);
  }
}

void loop() {
  int16_t buffer[fftSize]; // raw data - simulated 12bit ADC input
  get_wave_samples(buffer, fftSize); // simulate reading ADC

  Serial.println("generated sine wave data:");
  for (int i = 0; i < fftSize; i++) {
    Serial.print(buffer[i]); Serial.print(" ");
  }
  Serial.println("");

  //convert to interleaving fashion of Real and Imaginary values. Im=0
  int16_t complex_buffer[fftSize*2];
  for(int i = 0 ; i < fftSize; ++i){
    complex_buffer[i*2] = buffer[i]; // Re
    complex_buffer[i*2+1] = 0; // Im=0
  }

  Serial.println("Interleaved complex data Re[0], Im[0]=0, Re[1], Im[1]=0,...");
  for (int n = 0; n < fftSize*2; n++) {
    Serial.print(complex_buffer[n]); Serial.print(" ");
  }
  Serial.println("");

  dsps_fft2r_sc16_ae32(complex_buffer, fftSize);
  Serial.println("complex_buffer after FFT:");
  for (int i = 0; i < fftSize*2; i++) {
    Serial.print(complex_buffer[i]); Serial.print(" ");
  }
  Serial.println("");

  // compute complex magnitude
  float spectrum[fftSize];
  Serial.println("Spectrum:");
  for (int i = 0; i < fftSize; i++) {
    spectrum[i] = (float)sqrt(pow(complex_buffer[(2*i)+0], 2.0) + pow(complex_buffer[(2*i)+1], 2.0));
    Serial.print(spectrum[i]); Serial.print(" ");
  }
  Serial.println("");
  Serial.print("Expected whistle peak: spectrum[");Serial.print(whistleBin);Serial.print("]="); Serial.println(spectrum[whistleBin]);
  Serial.println("");
}


