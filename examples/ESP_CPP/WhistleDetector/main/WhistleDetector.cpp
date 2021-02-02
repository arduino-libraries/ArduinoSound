/*
 This example reads audio data from an Invensense's ICS43432 I2S microphone
 breakout board, and uses the input to detect whistling sounds at a particular
 frequency. When a whistle is detected, it's level is used to control the
 brightness of an LED

 Circuit:
 * Arduino/Genuino Zero, MKRZero or MKR1000 board
 * ICS43432:
   * GND connected GND
   * 3.3V connected 3.3V (Zero) or VCC (MKR1000, MKRZero)
   * WS connected to pin 0 (Zero) or pin 3 (MKR1000, MKRZero)
   * CLK connected to pin 1 (Zero) or pin 2 (MKR1000, MKRZero)
   * SD connected to pin 9 (Zero) or pin A6 (MKR1000, MKRZero)

 created 30 November 2016
 by Sandeep Mistry
 */


#include <ArduinoSound.h>

// lyrat v4_3
ES8388 *codec_chip;

// the LED pin to use as output
#ifdef ESP_PLATFORM
const int ledPin = GPIO_NUM_22;
#else
  const int ledPin = LED_BUILTIN;
#endif

// sample rate for the input
const int sampleRate = 44100;

const int bitsPerSample = 16;

// size of the FFT to compute
const int fftSize = 1024;

// size of the spectrum output, half of FFT size
const int spectrumSize = fftSize / 2;

// frequency of whistle to detect
const int whistleFrequency = 1250;

// map whistle frequency to FFT bin
const int whistleBin = (whistleFrequency * fftSize / sampleRate);

// array to store spectrum output
float spectrum[spectrumSize];

// create an FFT analyzer to be used with the I2S input
FFTAnalyzer fftAnalyzer(fftSize);

void setup() {
  disableCore1WDT();
  // setup the serial
#ifdef ESP_PLATFORM
  Serial.begin(115200);
#else
  Serial.begin(9600);
#endif
  Wire.begin(GPIO_NUM_18, GPIO_NUM_23);
  //Wire.begin(GPIO_NUM_18, GPIO_NUM_13);
  codec_chip = new ES8388(GPIO_NUM_21, Wire);

  // configure the pin for output mode
  pinMode(ledPin, OUTPUT);

  bool use_external_mic = false;
  if (!codec_chip->inBegin(sampleRate, bitsPerSample, use_external_mic)){
    Serial.println("Failed to initialize I2S input!");
    while (1); // do nothing
  }
  Serial.println("codec / audio begin OK.");

  // configure the I2S input as the input for the FFT analyzer
  if (!fftAnalyzer.input(*codec_chip)) {
    Serial.println("Failed to set FFT analyzer input!");
    while (1); // do nothing
  }
  Serial.println("All set - end of setup()");
}

float float_map(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void loop() {
  if (fftAnalyzer.available()) {
    // analysis available, read in the spectrum
    fftAnalyzer.readFloat(spectrum, spectrumSize);

    // map the value of the whistle bin magnitude between 0 and 255
    int ledValue;
    if (bitsPerSample == 16){
      ledValue = (int)float_map(spectrum[whistleBin], 250.0, 2500.0, 0.0, 255.0);
    }else{
      ledValue = (int)float_map(spectrum[whistleBin], 30000000000.0, 180000000000.0, 0.0, 255.0);
    }

    // cap the values
    if (ledValue < 0) {
      ledValue = 0;
    } else if (ledValue > 255) {
      ledValue = 255;
    }

    // set LED brightness based on whistle bin magnitude
#ifdef ESP_PLATFORM
    //ledcWrite(ledPin, ledValue); //GPIO_NUM_22 does not have PWM/analog output
    digitalWrite(ledPin, ledValue > 128 ? HIGH : LOW);
#else
    analogWrite(ledPin, ledValue);
#endif
  }
}
