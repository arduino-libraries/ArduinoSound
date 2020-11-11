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
const int sampleRate = 8000;

const int bitsPerSample = 32;
//const int bitsPerSample = 16;

// size of the FFT to compute
const int fftSize = 128;

// size of the spectrum output, half of FFT size
const int spectrumSize = fftSize / 2;

// frequency of whistle to detect
const int whistleFrequency = 1250;

// map whistle frequency to FFT bin
const int whistleBin = (whistleFrequency * fftSize / sampleRate);
//const int whistleBin = 30;

// array to store spectrum output
//int spectrum[spectrumSize];
float spectrum[spectrumSize];

// create an FFT analyzer to be used with the I2S input
FFTAnalyzer fftAnalyzer(fftSize);

//////////////////////////////////////////////*
//SDWaveFile waveFile;
//SPIClass sdspi(HSPI);
//////////////////////////////////////////////*

void setup() {
  // setup the serial
#ifdef ESP_PLATFORM
  Serial.begin(115200);
#else
  Serial.begin(9600);
#endif
  Serial.flush();
  //Serial.println("Create new codec chip...");
  //codec_chip = new ES8388(GPIO_NUM_21, GPIO_NUM_23, GPIO_NUM_18); // only i2c pins
  //codec_chip = new ES8388(GPIO_NUM_21, GPIO_NUM_23, GPIO_NUM_18, 5, 25, 26); // i2c+i2s pins
  Wire.begin(GPIO_NUM_18, GPIO_NUM_23);
  codec_chip = new ES8388(GPIO_NUM_21, Wire); // only i2s pins, i2c is wire
  // configure the pin for output mode
  pinMode(ledPin, OUTPUT);

  // setup the I2S audio input for the sample rate with 32-bits per sample

  bool use_external_mic = true;
  if (!codec_chip->begin(sampleRate, bitsPerSample, use_external_mic)){
  //if (!AudioInI2S.begin(sampleRate, bitsPerSample)){
    Serial.println("Failed to initialize I2S input!");
    while (1); // do nothing
  }
  Serial.println("codec / audio begin OK.");

//////////////////////////////////////////////////////////
/*
  Serial.print("Initializing SD card...");
  sdspi.begin(14, 2, 15); // Pin configuration
  if (!SD.begin(13, sdspi)) {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println("initialization done.");

  // use wav file as FFT input
  waveFile = SDWaveFile("/sin.wav");
  // Check if the WaveFile is valid
  if (!waveFile) {
    Serial.println("wave file is invalid!");
    while(1);
  }
  // recompute settings
  sampleRate = waveFile.sampleRate();
  bitsPerSample = waveFile.bitsPerSample();
  whistleBin = (whistleFrequency * fftSize / sampleRate);

  Serial.print("Sample rate = "); Serial.println(sampleRate);
  Serial.print("bitsPerSample = "); Serial.println(bitsPerSample);
  Serial.print("whistleBin = "); Serial.println(whistleBin);
*/
  /////////////////////////////////////////////////////////

  // configure the I2S input as the input for the FFT analyzer
  //if (!fftAnalyzer.input(waveFile)) {
  if (!fftAnalyzer.input(*codec_chip)) {
  //if (!fftAnalyzer.input(AudioInI2S)) {
    Serial.println("Failed to set FFT analyzer input!");
    while (1); // do nothing
  }
  Serial.println("FFT input set");
  Serial.println("All set - end of setup()");
  Serial.flush();
}

void find_peak(){
  //Serial.println("Computing Peaks...");
  int peaks[sizeof(spectrum)/sizeof(spectrum[0])];
    int pi = 0;

    for (int i = 0; i < sizeof(spectrum)/sizeof(spectrum[0]); i++) {
      if (
          (i == 0 && spectrum[i + 1] < spectrum[i] && spectrum[i + 2] < spectrum[i]) ||   /* first element */
          (i == sizeof(spectrum) / sizeof(spectrum[0]) && spectrum[i - 1] < spectrum[i] && spectrum[i - 2] < spectrum[i]) || /* last element */
          (i == 1 && spectrum[i] > spectrum[i - 1] && spectrum[i] > spectrum[i + 1] &&
           spectrum[i] > spectrum[i - 2] && spectrum[i] > spectrum[i + 2]) || /* second */
          (spectrum[i] > spectrum[i - 1] && spectrum[i] > spectrum[i + 1] &&
           spectrum[i] > spectrum[i - 2] && spectrum[i] > spectrum[i + 2]) || /* second last */
          (i+1 == sizeof(spectrum) / sizeof(spectrum[0]) && spectrum[i] > spectrum[i - 1] && spectrum[i] > spectrum[i + 1] &&
           spectrum[i] > spectrum[i - 2] && spectrum[i] > spectrum[i + 2]) /* elements in the middle */
        )
      {
        //peaks[pi++] = spectrum[i]; // value
        peaks[pi++] = i; // index
      }
    }

    Serial.print("Peak frequencies: ");
    for (int i = 0; i < pi; i++){
      int freq = (peaks[i] * sampleRate) / fftSize ;
      Serial.print(freq); Serial.print(" "); // index
      //Serial.print(peaks[i]); Serial.print(" "); // value
    }
    Serial.println("");
}

float float_map(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void loop() {
  static int min = 1334116324; // debug
  static int max = 0; // debug
  static long avg = 0; // debug
  long long_whistle_spectrum;

  if (fftAnalyzer.available()) {
    // analysis available, read in the spectrum
    fftAnalyzer.readFloat(spectrum, spectrumSize);

    // map the value of the whistle bin magnitude between 0 and 255
    int ledValue;
    //int ledValue = map(spectrum[whistleBin], 50000, 60000, 0, 255);
    if (bitsPerSample == 16){
      //ledValue = map(spectrum[whistleBin], 0.0, 12000.0, 0, 255); // 16bit - wav sample
      long_whistle_spectrum = (long)spectrum[whistleBin];
      ledValue = map(long_whistle_spectrum , 100, 1500, 0, 255); // 16bit - real whistle
    }else{
      //long_whistle_spectrum = (long)(spectrum[whistleBin])/1000;
      //ledValue = map(long_whistle_spectrum, 150000000, 180000000, 0, 255); // 32bit
      ledValue = (int)float_map(spectrum[whistleBin], 30000000000.0, 180000000000.0, 0.0, 255.0); // 32bit
    }

    char str[30]; // for debug printing large floats
/*
    Serial.println("");
    Serial.println("main loop spectrum = ");
    for(int i = 0; i < spectrumSize; ++i){ // debug print; TODO remove this block
      //Serial.print(spectrum[i]); Serial.print(" ");
      sprintf(str, "%.2f ", spectrum[i]); Serial.print(str);
    }
    Serial.println("");
*/

    //find_peak();
    //sprintf(str, "%.2f ", spectrum[whistleBin]);
    //Serial.print("read from spectrum[");Serial.print(whistleBin);Serial.print("] = ");Serial.print(str);
    //Serial.print("read from spectrum[");Serial.print(whistleBin);Serial.print("] = ");Serial.print(long_whistle_spectrum );
    Serial.print(" new ledValue = "); Serial.println(ledValue);

    //sprintf(str, "%.2f ", spectrum[whistleBin]); Serial.print(str);
    //Serial.print(ledValue); Serial.print(" ");

    // debug part to find out min/max
    if(spectrum[whistleBin] > max){
      max = spectrum[whistleBin];
      //Serial.print("new max = ");Serial.println(max);
    }
    avg = (avg+spectrum[whistleBin])/2;
    //Serial.print("avg = ");Serial.println(avg);

    //Serial.print("current <min;max> = ");Serial.print(min);Serial.print(";");Serial.println(max);

    // cap the values
    if (ledValue < 0) {
      ledValue = 0;
    } else if (ledValue > 255) {
      ledValue = 255;
    }

    // set LED brightness based on whistle bin magnitude
#ifdef ESP_PLATFORM
    //ledcWrite(ledPin, ledValue); //GPIO_NUM_22 does not have PWM/analog output
    if(ledValue > 16){
      digitalWrite(ledPin, HIGH);
    }else{
      digitalWrite(ledPin, LOW);
    }
#else
    analogWrite(ledPin, ledValue);
#endif
  }
}
