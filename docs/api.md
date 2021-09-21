# Arduino Sound

## AmplitudeAnalyzer Class

amplitudeAnalyzer.input()`

#### Description
Set the input of the analyzer.

#### Syntax

```
amplitudeAnalyzer.input(in);

```

#### Parameters
in: input to analyze (type AudioIn)

#### Returns
0 on failure, 1 on success

#### Example

```
/*
 This example reads audio data from an InvenSense ICS-43432 I2S microphone
 breakout board, and prints out the amplitude to the Serial Monitor. The
 Serial Plotter built into the Arduino IDE (Tools -> Serial Plotter) can be
 used to plot the audio amplitude data.

 Circuit:
 * Arduino Zero, MKR Zero or MKR1000 board
 * ICS-43432:
   * GND connected GND
   * 3.3V connected 3.3V (Zero) or VCC (MKR1000, MKR Zero)
   * WS connected to pin 0 (Zero) or pin 3 (MKR1000, MKR Zero)
   * CLK connected to pin 1 (Zero) or pin 2 (MKR1000, MKR Zero)
   * SD connected to pin 9 (Zero) or pin A6 (MKR1000, MKR Zero)

 created 23 November 2016
 by Sandeep Mistry
 */

#include <ArduinoSound.h>

// create an amplitude analyzer to be used with the I2S input
AmplitudeAnalyzer amplitudeAnalyzer;

void setup() {
  // Open serial communications and wait for port to open:
  // A baud rate of 115200 is used instead of 9600 for a faster data rate
  // on non-native USB ports
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  // setup the I2S audio input for 44.1 kHz with 32-bits per sample
  if (!AudioInI2S.begin(44100, 32)) {
    Serial.println("Failed to initialize I2S input!");
    while (1); // do nothing
  }

  // configure the I2S input as the input for the amplitude analyzer
  if (!amplitudeAnalyzer.input(AudioInI2S)) {
    Serial.println("Failed to set amplitude analyzer input!");
    while (1); // do nothing
  }
}

void loop() {
  // check if a new analysis is available
  if (amplitudeAnalyzer.available()) {
    // read the new amplitude
    int amplitude = amplitudeAnalyzer.read();
    //dB relative to full scale
    int dpFS = 20 * log10(abs(amplitude));
    // print out the decibel to the serial monitor
    Serial.println(dpFS);
  }
}
```

### `amplitudeAnalyzer.available()`

#### Description
Query the number of amplitudes available for reading using read()

#### Syntax

```
amplitudeAnalyzer.available();
```

#### Returns
The number of amplitudes available for reading using read()

#### Example

```
/*
 This example reads audio data from an InvenSense ICS-43432 I2S microphone
 breakout board, and prints out the amplitude to the Serial Monitor. The
 Serial Plotter built into the Arduino IDE (Tools -> Serial Plotter) can be
 used to plot the audio amplitude data.

 Circuit:
 * Arduino Zero, MKR Zero or MKR1000 board
 * ICS-43432:
   * GND connected GND
   * 3.3V connected 3.3V (Zero) or VCC (MKR1000, MKR Zero)
   * WS connected to pin 0 (Zero) or pin 3 (MKR1000, MKR Zero)
   * CLK connected to pin 1 (Zero) or pin 2 (MKR1000, MKR Zero)
   * SD connected to pin 9 (Zero) or pin A6 (MKR1000, MKR Zero)

 created 23 November 2016
 by Sandeep Mistry
 */

#include <ArduinoSound.h>

// create an amplitude analyzer to be used with the I2S input
AmplitudeAnalyzer amplitudeAnalyzer;

void setup() {
  // Open serial communications and wait for port to open:
  // A baud rate of 115200 is used instead of 9600 for a faster data rate
  // on non-native USB ports
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  // setup the I2S audio input for 44.1 kHz with 32-bits per sample
  if (!AudioInI2S.begin(44100, 32)) {
    Serial.println("Failed to initialize I2S input!");
    while (1); // do nothing
  }

  // configure the I2S input as the input for the amplitude analyzer
  if (!amplitudeAnalyzer.input(AudioInI2S)) {
    Serial.println("Failed to set amplitude analyzer input!");
    while (1); // do nothing
  }
}

void loop() {
  // check if a new analysis is available
  if (amplitudeAnalyzer.available()) {
    // read the new amplitude
    int amplitude = amplitudeAnalyzer.read();
    //dB relative to full scale
    int dpFS = 20 * log10(abs(amplitude));
    // print out the decibel to the serial monitor
    Serial.println(dpFS);
  }
}
```

### `amplitudeAnalyzer.read()`

#### Description
Read the amplitude value of the current audio block

#### Syntax

```
amplitudeAnalyzer.read();
```

#### Returns
The amplitude value of the current audio block

#### Example

```
/*
 This example reads audio data from an InvenSense ICS-43432 I2S microphone
 breakout board, and prints out the amplitude to the Serial Monitor. The
 Serial Plotter built into the Arduino IDE (Tools -> Serial Plotter) can be
 used to plot the audio amplitude data.

 Circuit:
 * Arduino Zero, MKR Zero or MKR1000 board
 * ICS-43432:
   * GND connected GND
   * 3.3V connected 3.3V (Zero) or VCC (MKR1000, MKR Zero)
   * WS connected to pin 0 (Zero) or pin 3 (MKR1000, MKR Zero)
   * CLK connected to pin 1 (Zero) or pin 2 (MKR1000, MKR Zero)
   * SD connected to pin 9 (Zero) or pin A6 (MKR1000, MKR Zero)

 created 23 November 2016
 by Sandeep Mistry
 */

#include <ArduinoSound.h>

// create an amplitude analyzer to be used with the I2S input
AmplitudeAnalyzer amplitudeAnalyzer;

void setup() {
  // Open serial communications and wait for port to open:
  // A baud rate of 115200 is used instead of 9600 for a faster data rate
  // on non-native USB ports
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  // setup the I2S audio input for 44.1 kHz with 32-bits per sample
  if (!AudioInI2S.begin(44100, 32)) {
    Serial.println("Failed to initialize I2S input!");
    while (1); // do nothing
  }

  // configure the I2S input as the input for the amplitude analyzer
  if (!amplitudeAnalyzer.input(AudioInI2S)) {
    Serial.println("Failed to set amplitude analyzer input!");
    while (1); // do nothing
  }
}

void loop() {
  // check if a new analysis is available
  if (amplitudeAnalyzer.available()) {
    // read the new amplitude
    int amplitude = amplitudeAnalyzer.read();
    //dB relative to full scale
    int dpFS = 20 * log10(abs(amplitude));
    // print out the decibel to the serial monitor
    Serial.println(dpFS);
  }
}
```

### `## AudioInI2S class

AudioInI2S.begin()`

#### Description
Configure the I2S interface for input with the specified sample rate and bits per sample and start capturing audio data

#### Syntax

```
AudioInI2S.begin(sampleRate, bitsPerSample);

```

#### Parameters
sampleRate - sample rate to configure in Hz
bitsPerSample - bits per sample to configure (8, 16, 32)

#### Returns
0 on failure, 1 on success

#### Example

```
/*
 This example reads audio data from an InvenSense ICS-43432 I2S microphone
 breakout board, and prints out the amplitude to the Serial Monitor. The
 Serial Plotter built into the Arduino IDE (Tools -> Serial Plotter) can be
 used to plot the audio amplitude data.

 Circuit:
 * Arduino Zero, MKR Zero or MKR1000 board
 * ICS-43432:
   * GND connected GND
   * 3.3V connected 3.3V (Zero) or VCC (MKR1000, MKR Zero)
   * WS connected to pin 0 (Zero) or pin 3 (MKR1000, MKR Zero)
   * CLK connected to pin 1 (Zero) or pin 2 (MKR1000, MKR Zero)
   * SD connected to pin 9 (Zero) or pin A6 (MKR1000, MKR Zero)

 created 23 November 2016
 by Sandeep Mistry
 */

#include <ArduinoSound.h>

// create an amplitude analyzer to be used with the I2S input
AmplitudeAnalyzer amplitudeAnalyzer;

void setup() {
  // Open serial communications and wait for port to open:
  // A baud rate of 115200 is used instead of 9600 for a faster data rate
  // on non-native USB ports
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  // setup the I2S audio input for 44.1 kHz with 32-bits per sample
  if (!AudioInI2S.begin(44100, 32)) {
    Serial.println("Failed to initialize I2S input!");
    while (1); // do nothing
  }

  // configure the I2S input as the input for the amplitude analyzer
  if (!amplitudeAnalyzer.input(AudioInI2S)) {
    Serial.println("Failed to set amplitude analyzer input!");
    while (1); // do nothing
  }
}

void loop() {
  // check if a new analysis is available
  if (amplitudeAnalyzer.available()) {
    // read the new amplitude
    int amplitude = amplitudeAnalyzer.read();
    //dB relative to full scale
    int dpFS = 20 * log10(abs(amplitude));
    // print out the decibel to the serial monitor
    Serial.println(dpFS);
  }
}
```

### `AudioInI2S.end()`

#### Description
Disables the I2S input interface and stops capturing audio data

#### Syntax

```
AudioInI2S.end();
```

#### Returns
None

#### Example

```
…
  // setup the I2S audio input for 44.1 kHz with 32-bits per sample
  if (!AudioInI2S.begin(44100, 32)) {
    Serial.println("Failed to initialize I2S input!");
    while (1); // do nothing
  }

   // do some processing

  // stop the input source
  AudioInI2S.end();
…
```

### `AudioInI2S.sampleRate()`

#### Description
Query the sample rate of the I2S input

#### Syntax

```
AudioInI2S.sampleRate();
```

#### Returns
Current sample rate of the I2S interface in Hz.

#### Example

```
…
  // setup the I2S audio input for 44.1 kHz with 32-bits per sample
  if (!AudioInI2S.begin(44100, 32)) {
    Serial.println("Failed to initialize I2S input!");
    while (1); // do nothing
  }

  Serial.print(“sample rate = “);
  Serial.println(AudioInI2S.sampleRate()); // prints out the sample rate used in begin(...)
  Serial.print(“bit per sample = “);
  Serial.println(AudioInI2S.bitsPerSample()); // prints out the bits per sample rate used  in begin(...)
  Serial.print(“channels = “);
  Serial.println(AudioInI2S.channels()); // prints out 2, I2S always has 2 channels
…
```

### `AudioInI2S.bitsPerSample()`

#### Description
Query the bits per sample of the I2S input

#### Syntax

```
AudioInI2S.bitsPerSample();
```

#### Returns
Current bits per sample of the I2S interface

#### Example

```
…
  // setup the I2S audio input for 44.1 kHz with 32-bits per sample
  if (!AudioInI2S.begin(44100, 32)) {
    Serial.println("Failed to initialize I2S input!");
    while (1); // do nothing
  }

  Serial.print(“sample rate = “);
  Serial.println(AudioInI2S.sampleRate()); // prints out the sample rate used in begin(...)
  Serial.print(“bit per sample = “);
  Serial.println(AudioInI2S.bitsPerSample()); // prints out the bits per sample rate used  in begin(...)
  Serial.print(“channels = “);
  Serial.println(AudioInI2S.channels()); // prints out 2, I2S always has 2 channels
…
```

### `AudioInI2S.channels()`

#### Description
Query the number of channels of the I2S input

#### Syntax

```
AudioInI2S.channels();
```

#### Returns
Current number of channels of the I2S interface. This is always return 2, as I2S is configured for stereo

#### Example

```
…
  // setup the I2S audio input for 44.1 kHz with 32-bits per sample
  if (!AudioInI2S.begin(44100, 32)) {
    Serial.println("Failed to initialize I2S input!");
    while (1); // do nothing
  }

  Serial.print(“sample rate = “);
  Serial.println(AudioInI2S.sampleRate()); // prints out the sample rate used in begin(...)
  Serial.print(“bit per sample = “);
  Serial.println(AudioInI2S.bitsPerSample()); // prints out the bits per sample rate used  in begin(...)
  Serial.print(“channels = “);
  Serial.println(AudioInI2S.channels()); // prints out 2, I2S always has 2 channels
…
```

## AudioOutI2S Class

### `AudioOutI2S.canPlay()`

#### Description
Check if the specified input can be played out using I2S

#### Syntax

```
AudioOutI2S.canPlay(input);

```

#### Parameters
input: input to check (type AudioIn)

#### Returns
1 in the input can be played using play() or loop(), 0 otherwise

#### Example

```
…
  // check if the I2S output can play the Audio input
  if (!AudioOutI2S.canPlay(audioInput)) {
    Serial.println("unable to play audio input using I2S!");
    while (1); // do nothing
  }

  // start playback
  Serial.println("starting playback");
  AudioOutI2S.play(audioInput);
…
```

### `AudioOutI2S.play()`

#### Description
Play the input using the I2S interface

#### Syntax

```
AudioOutI2S.play(input);

```

#### Parameters
input: input to check (type AudioIn)

#### Returns
1 if input has successfully started playing, 0 on failure

#### Example

```
…
  // check if the I2S output can play the Audio input
  if (!AudioOutI2S.canPlay(audioInput)) {
    Serial.println("unable to play audio input using I2S!");
    while (1); // do nothing
  }

  // start playback
  Serial.println("starting playback");
  AudioOutI2S.play(audioInput);
…
```

### `AudioOutI2S.loop()`

#### Description
Play the input in a loop using the I2S interface

#### Syntax

```
AudioOutI2S.loop(input);

```

#### Parameters
input: input to check (type AudioIn)

#### Returns
1 if input has successfully started playing, 0 on failure

AudioOutI2S.pause()`

#### Description
Pause the current input that is playing out the I2S interface

#### Syntax

```
AudioOutI2S.pause();
```

#### Returns
1 if input is successfully paused, 0 on failure

#### Example

```
…
  // check if the I2S output can play the Audio input
  if (!AudioOutI2S.canPlay(audioInput)) {
    Serial.println("unable to play audio input using I2S!");
    while (1); // do nothing
  }

  // start playback
  Serial.println("starting playback");
  AudioOutI2S.play(audioInput);

  delay(10000); // wait for 10 seconds

  AudioOutI2S.pause(); // pause playback

  delay(10000); // wait for 10 seconds

  AudioOutI2S.resume(); // resume playback
…
```

### `AudioOutI2S.resume()`

#### Description
Resume playback of the input that was paused using pause() out the I2S interface

#### Syntax

```
AudioOutI2S.resume();
```

#### Returns
1 if input is successfully resumed, 0 on failure

#### Example

```
…
  // check if the I2S output can play the Audio input
  if (!AudioOutI2S.canPlay(audioInput)) {
    Serial.println("unable to play audio input using I2S!");
    while (1); // do nothing
  }

  // start playback
  Serial.println("starting playback");
  AudioOutI2S.play(audioInput);

  delay(10000); // wait for 10 seconds

  AudioOutI2S.pause(); // pause playback

  delay(10000); // wait for 10 seconds

  AudioOutI2S.resume(); // resume playback
…
```

### `AudioOutI2S.stop()`

#### Description
Stop playback of the current input that is playing out the I2S interface

#### Syntax

```
AudioOutI2S.stop();
```

#### Returns
1 if input is successfully resumed, 0 on failure

#### Example

```
…
  // check if the I2S output can play the Audio input
  if (!AudioOutI2S.canPlay(audioInput)) {
    Serial.println("unable to play audio input using I2S!");
    while (1); // do nothing
  }

  // start playback
  Serial.println("starting playback");
  AudioOutI2S.play(audioInput);

  delay(10000); // wait for 10 seconds

  AudioOutI2S.stop(); // stop playback
…
```

### `AudioOutI2S.isPlaying()`

#### Description
Query if the input started using play() or loop() is currently playing

#### Syntax

```
AudioOutI2S.isPlaying();
```

#### Returns
1 if input is currently playing, 0 otherwise

#### Example

```
…
  // check if playback is still going on
  if (!AudioOutI2S.isPlaying()) {
    // playback has stopped

    Serial.println("playback stopped");
    while (1); // do nothing
  }
…
```

### `AudioOutI2S.isPaused()`

#### Description
Query if the input started using play() or loop() has been paused using pause()

#### Syntax

```
AudioOutI2S.isPaused();
```

#### Returns
1 if input is currently paused, 0 otherwise

#### Example

```
…
  // check if playback is paused
  if (AudioOutI2S.isPaued()) {
    // playback has paused

    Serial.println("playback paused");
    while (1); // do nothing
  }
…
```

### `AudioOutI2S.volume()`

#### Description
Set the playback volume of the input

#### Syntax

```
AudioOutI2S.volume(level);

```

#### Parameters
level: volume level to set between 0.00 and 100.0 as a percentage

#### Returns
None

#### Example

```
…
  // adjust the playback volume
  AudioOutI2S.volume(5);
…
```

## FFTAnalyzer class

### `FFTAnalyzer`

#### Description
Create an FFT analyzer with the specified length

#### Syntax

```
FFTAnalyzer(length);

```

#### Parameters
length: length to use for FFT in samples. Must be a power of 2 (64, 128, 256, etc)

#### Example

```
/*
 This example reads audio data from an InvenSense ICS-43432 I2S microphone
 breakout board, and prints out the spectrum to the Serial Monitor. The
 Serial Plotter built into the Arduino IDE (Tools -> Serial Plotter) can be
 used to plot the audio amplitude data.

 Circuit:
 * Arduino Zero, MKR Zero or MKR1000 board
 * ICS-43432:
   * GND connected GND
   * 3.3V connected 3.3V (Zero) or VCC (MKR1000, MKR Zero)
   * WS connected to pin 0 (Zero) or pin 3 (MKR1000, MKR Zero)
   * CLK connected to pin 1 (Zero) or pin 2 (MKR1000, MKR Zero)
   * SD connected to pin 9 (Zero) or pin A6 (MKR1000, MKR Zero)

 created 21 November 2016
 by Sandeep Mistry
 */

#include <ArduinoSound.h>

// sample rate for the input
const int sampleRate = 8000;

// size of the FFT to compute
const int fftSize = 128;

// size of the spectrum output, half of FFT size
const int spectrumSize = fftSize / 2;

// array to store spectrum output
int spectrum[spectrumSize];

// create an FFT analyzer to be used with the I2S input
FFTAnalyzer fftAnalyzer(fftSize);

void setup() {
// Open serial communications and wait for port to open:
  // A baud rate of 115200 is used instead of 9600 for a faster data rate
  // on non-native USB ports
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  // setup the I2S audio input for the sample rate with 32-bits per sample
  if (!AudioInI2S.begin(sampleRate, 32)) {
    Serial.println("Failed to initialize I2S input!");
    while (1); // do nothing
  }

  // configure the I2S input as the input for the FFT analyzer
  if (!fftAnalyzer.input(AudioInI2S)) {
    Serial.println("Failed to set FFT analyzer input!");
    while (1); // do nothing
  }
}

void loop() {
  // check if a new analysis is available
  if (fftAnalyzer.available()) {
    // read the new spectrum
    fftAnalyzer.read(spectrum, spectrumSize);

    // print out the spectrum
    for (int i = 0; i < spectrumSize; i++) {
      Serial.print((i * sampleRate) / fftSize); // the starting frequency
      Serial.print("\t"); //
      Serial.println(spectrum[i]); // the spectrum value
    }
  }
}
```

### `FFTAnalyzer.input()`

#### Description
Set the input of the analyzer

#### Syntax

```
fftAnalyzer.input(in);

```

#### Parameters
in: input to analyze (type AudioIn)

#### Returns
0 on failure, 1 on success

#### Example

```
/*
 This example reads audio data from an InvenSense ICS-43432 I2S microphone
 breakout board, and prints out the spectrum to the Serial Monitor. The
 Serial Plotter built into the Arduino IDE (Tools -> Serial Plotter) can be
 used to plot the audio amplitude data.

 Circuit:
 * Arduino Zero, MKR Zero or MKR1000 board
 * ICS-43432:
   * GND connected GND
   * 3.3V connected 3.3V (Zero) or VCC (MKR1000, MKR Zero)
   * WS connected to pin 0 (Zero) or pin 3 (MKR1000, MKR Zero)
   * CLK connected to pin 1 (Zero) or pin 2 (MKR1000, MKR Zero)
   * SD connected to pin 9 (Zero) or pin A6 (MKR1000, MKR Zero)

 created 21 November 2016
 by Sandeep Mistry
 */

#include <ArduinoSound.h>

// sample rate for the input
const int sampleRate = 8000;

// size of the FFT to compute
const int fftSize = 128;

// size of the spectrum output, half of FFT size
const int spectrumSize = fftSize / 2;

// array to store spectrum output
int spectrum[spectrumSize];

// create an FFT analyzer to be used with the I2S input
FFTAnalyzer fftAnalyzer(fftSize);

void setup() {
// Open serial communications and wait for port to open:
  // A baud rate of 115200 is used instead of 9600 for a faster data rate
  // on non-native USB ports
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  // setup the I2S audio input for the sample rate with 32-bits per sample
  if (!AudioInI2S.begin(sampleRate, 32)) {
    Serial.println("Failed to initialize I2S input!");
    while (1); // do nothing
  }

  // configure the I2S input as the input for the FFT analyzer
  if (!fftAnalyzer.input(AudioInI2S)) {
    Serial.println("Failed to set FFT analyzer input!");
    while (1); // do nothing
  }
}

void loop() {
  // check if a new analysis is available
  if (fftAnalyzer.available()) {
    // read the new spectrum
    fftAnalyzer.read(spectrum, spectrumSize);

    // print out the spectrum
    for (int i = 0; i < spectrumSize; i++) {
      Serial.print((i * sampleRate) / fftSize); // the starting frequency
      Serial.print("\t"); //
      Serial.println(spectrum[i]); // the spectrum value
    }
  }
}
```

### `FFTAnalyzer.available()`

#### Description
Query if a spectrum analysis is available for reading using read()

#### Syntax

```
fftAnalyzer.available();
```

#### Returns
1 if a spectrum analysis is available for reading, 0 otherwise.

#### Example

```
/*
 This example reads audio data from an InvenSense ICS-43432 I2S microphone
 breakout board, and prints out the spectrum to the Serial Monitor. The
 Serial Plotter built into the Arduino IDE (Tools -> Serial Plotter) can be
 used to plot the audio amplitude data.

 Circuit:
 * Arduino Zero, MKR Zero or MKR1000 board
 * ICS-43432:
   * GND connected GND
   * 3.3V connected 3.3V (Zero) or VCC (MKR1000, MKR Zero)
   * WS connected to pin 0 (Zero) or pin 3 (MKR1000, MKR Zero)
   * CLK connected to pin 1 (Zero) or pin 2 (MKR1000, MKR Zero)
   * SD connected to pin 9 (Zero) or pin A6 (MKR1000, MKR Zero)

 created 21 November 2016
 by Sandeep Mistry
 */

#include <ArduinoSound.h>

// sample rate for the input
const int sampleRate = 8000;

// size of the FFT to compute
const int fftSize = 128;

// size of the spectrum output, half of FFT size
const int spectrumSize = fftSize / 2;

// array to store spectrum output
int spectrum[spectrumSize];

// create an FFT analyzer to be used with the I2S input
FFTAnalyzer fftAnalyzer(fftSize);

void setup() {
// Open serial communications and wait for port to open:
  // A baud rate of 115200 is used instead of 9600 for a faster data rate
  // on non-native USB ports
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  // setup the I2S audio input for the sample rate with 32-bits per sample
  if (!AudioInI2S.begin(sampleRate, 32)) {
    Serial.println("Failed to initialize I2S input!");
    while (1); // do nothing
  }

  // configure the I2S input as the input for the FFT analyzer
  if (!fftAnalyzer.input(AudioInI2S)) {
    Serial.println("Failed to set FFT analyzer input!");
    while (1); // do nothing
  }
}

void loop() {
  // check if a new analysis is available
  if (fftAnalyzer.available()) {
    // read the new spectrum
    fftAnalyzer.read(spectrum, spectrumSize);

    // print out the spectrum
    for (int i = 0; i < spectrumSize; i++) {
      Serial.print((i * sampleRate) / fftSize); // the starting frequency
      Serial.print("\t"); //
      Serial.println(spectrum[i]); // the spectrum value
    }
  }
}
```

### `FFTAnalyzer.read()`

#### Description
Read the current spectrum analysis

#### Syntax

```
fftAnalyzer.read(spectrum, length);

```

#### Parameters
spectrum: integer array to read spectrum value into
length: array length of spectrum array parameter
#### Returns
The number of spectrum values stored in the spectrum array parameter

#### Example

```
/*
 This example reads audio data from an InvenSense ICS-43432 I2S microphone
 breakout board, and prints out the spectrum to the Serial Monitor. The
 Serial Plotter built into the Arduino IDE (Tools -> Serial Plotter) can be
 used to plot the audio amplitude data.

 Circuit:
 * Arduino Zero, MKR Zero or MKR1000 board
 * ICS-43432:
   * GND connected GND
   * 3.3V connected 3.3V (Zero) or VCC (MKR1000, MKR Zero)
   * WS connected to pin 0 (Zero) or pin 3 (MKR1000, MKR Zero)
   * CLK connected to pin 1 (Zero) or pin 2 (MKR1000, MKR Zero)
   * SD connected to pin 9 (Zero) or pin A6 (MKR1000, MKR Zero)

 created 21 November 2016
 by Sandeep Mistry
 */

#include <ArduinoSound.h>

// sample rate for the input
const int sampleRate = 8000;

// size of the FFT to compute
const int fftSize = 128;

// size of the spectrum output, half of FFT size
const int spectrumSize = fftSize / 2;

// array to store spectrum output
int spectrum[spectrumSize];

// create an FFT analyzer to be used with the I2S input
FFTAnalyzer fftAnalyzer(fftSize);

void setup() {
// Open serial communications and wait for port to open:
  // A baud rate of 115200 is used instead of 9600 for a faster data rate
  // on non-native USB ports
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  // setup the I2S audio input for the sample rate with 32-bits per sample
  if (!AudioInI2S.begin(sampleRate, 32)) {
    Serial.println("Failed to initialize I2S input!");
    while (1); // do nothing
  }

  // configure the I2S input as the input for the FFT analyzer
  if (!fftAnalyzer.input(AudioInI2S)) {
    Serial.println("Failed to set FFT analyzer input!");
    while (1); // do nothing
  }
}

void loop() {
  // check if a new analysis is available
  if (fftAnalyzer.available()) {
    // read the new spectrum
    fftAnalyzer.read(spectrum, spectrumSize);

    // print out the spectrum
    for (int i = 0; i < spectrumSize; i++) {
      Serial.print((i * sampleRate) / fftSize); // the starting frequency
      Serial.print("\t"); //
      Serial.println(spectrum[i]); // the spectrum value
    }
  }
}
```

## SDWave class

### `SDWaveFile`

#### Description

Create an audio in source for a Wave file on an SD card

#### Syntax

```
SDWaveFile(filename);

```

#### Parameters
filename: filename of wave file on the SD card

#### Example

```
/*
 This reads a wave file from an SD card and plays it using the I2S interface to
 a MAX98357 I2S Amp Breakout board.

 Circuit:
 * Arduino Zero, MKR Zero or MKR1000 board
 * SD breakout or shield connected
 * MAX98357:
   * GND connected GND
   * VIN connected 5V
   * LRC connected to pin 0 (Zero) or pin 3 (MKR1000, MKR Zero)
   * BCLK connected to pin 1 (Zero) or pin 2 (MKR1000, MKR Zero)
   * DIN connected to pin 9 (Zero) or pin A6 (MKR1000, MKR Zero)

 created 15 November 2016
 by Sandeep Mistry
 */

#include <SD.h>
#include <ArduinoSound.h>

// filename of wave file to play
const char filename[] = "MUSIC.WAV";

// variable representing the Wave File
SDWaveFile waveFile;

void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  // setup the SD card, depending on your shield of breakout board
  // you may need to pass a pin number in begin for SS
  Serial.print("Initializing SD card...");
  if (!SD.begin()) {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println("initialization done.");

  // create a SDWaveFile
  waveFile = SDWaveFile(filename);

  // check if the WaveFile is valid
  if (!waveFile) {
    Serial.println("wave file is invalid!");
    while (1); // do nothing
  }

  // print out some info. about the wave file
  Serial.print("Bits per sample = ");
  Serial.println(waveFile.bitsPerSample());

  long channels = waveFile.channels();
  Serial.print("Channels = ");
  Serial.println(channels);

  long sampleRate = waveFile.sampleRate();
  Serial.print("Sample rate = ");
  Serial.print(sampleRate);
  Serial.println(" Hz");

  long duration = waveFile.duration();
  Serial.print("Duration = ");
  Serial.print(duration);
  Serial.println(" seconds");

  // adjust the playback volume
  AudioOutI2S.volume(5);

  // check if the I2S output can play the wave file
  if (!AudioOutI2S.canPlay(waveFile)) {
    Serial.println("unable to play wave file using I2S!");
    while (1); // do nothing
  }

  // start playback
  Serial.println("starting playback");
  AudioOutI2S.play(waveFile);
}

void loop() {
  // check if playback is still going on
  if (!AudioOutI2S.isPlaying()) {
    // playback has stopped

    Serial.println("playback stopped");
    while (1); // do nothing
  }
}
```

### `SDWaveFile.bool()`

#### Description
Check if the wave file is valid

#### Syntax

```
If (waveFile) { /* … */ }
```

#### Returns
true if the valid file exists on the SD card and is valid, false otherwise

#### Example

```
/*
 This reads a wave file from an SD card and plays it using the I2S interface to
 a MAX98357 I2S Amp Breakout board.

 Circuit:
 * Arduino Zero, MKR Zero or MKR1000 board
 * SD breakout or shield connected
 * MAX98357:
   * GND connected GND
   * VIN connected 5V
   * LRC connected to pin 0 (Zero) or pin 3 (MKR1000, MKR Zero)
   * BCLK connected to pin 1 (Zero) or pin 2 (MKR1000, MKR Zero)
   * DIN connected to pin 9 (Zero) or pin A6 (MKR1000, MKR Zero)

 created 15 November 2016
 by Sandeep Mistry
 */

#include <SD.h>
#include <ArduinoSound.h>

// filename of wave file to play
const char filename[] = "MUSIC.WAV";

// variable representing the Wave File
SDWaveFile waveFile;

void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  // setup the SD card, depending on your shield of breakout board
  // you may need to pass a pin number in begin for SS
  Serial.print("Initializing SD card...");
  if (!SD.begin()) {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println("initialization done.");

  // create a SDWaveFile
  waveFile = SDWaveFile(filename);

  // check if the WaveFile is valid
  if (!waveFile) {
    Serial.println("wave file is invalid!");
    while (1); // do nothing
  }

  // print out some info. about the wave file
  Serial.print("Bits per sample = ");
  Serial.println(waveFile.bitsPerSample());

  long channels = waveFile.channels();
  Serial.print("Channels = ");
  Serial.println(channels);

  long sampleRate = waveFile.sampleRate();
  Serial.print("Sample rate = ");
  Serial.print(sampleRate);
  Serial.println(" Hz");

  long duration = waveFile.duration();
  Serial.print("Duration = ");
  Serial.print(duration);
  Serial.println(" seconds");

  // adjust the playback volume
  AudioOutI2S.volume(5);

  // check if the I2S output can play the wave file
  if (!AudioOutI2S.canPlay(waveFile)) {
    Serial.println("unable to play wave file using I2S!");
    while (1); // do nothing
  }

  // start playback
  Serial.println("starting playback");
  AudioOutI2S.play(waveFile);
}

void loop() {
  // check if playback is still going on
  if (!AudioOutI2S.isPlaying()) {
    // playback has stopped

    Serial.println("playback stopped");
    while (1); // do nothing
  }
}
```

### `SDWaveFile.sampleRate()`

#### Description
Query the sample rate of the wave file

#### Syntax

```
waveFile.sampleRate()
```

#### Returns
The sample rate of the wave file in Hz.

#### Example

```
/*
 This reads a wave file from an SD card and plays it using the I2S interface to
 a MAX98357 I2S Amp Breakout board.

 Circuit:
 * Arduino Zero, MKR Zero or MKR1000 board
 * SD breakout or shield connected
 * MAX98357:
   * GND connected GND
   * VIN connected 5V
   * LRC connected to pin 0 (Zero) or pin 3 (MKR1000, MKR Zero)
   * BCLK connected to pin 1 (Zero) or pin 2 (MKR1000, MKR Zero)
   * DIN connected to pin 9 (Zero) or pin A6 (MKR1000, MKR Zero)

 created 15 November 2016
 by Sandeep Mistry
 */

#include <SD.h>
#include <ArduinoSound.h>

// filename of wave file to play
const char filename[] = "MUSIC.WAV";

// variable representing the Wave File
SDWaveFile waveFile;

void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  // setup the SD card, depending on your shield of breakout board
  // you may need to pass a pin number in begin for SS
  Serial.print("Initializing SD card...");
  if (!SD.begin()) {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println("initialization done.");

  // create a SDWaveFile
  waveFile = SDWaveFile(filename);

  // check if the WaveFile is valid
  if (!waveFile) {
    Serial.println("wave file is invalid!");
    while (1); // do nothing
  }

  // print out some info. about the wave file
  Serial.print("Bits per sample = ");
  Serial.println(waveFile.bitsPerSample());

  long channels = waveFile.channels();
  Serial.print("Channels = ");
  Serial.println(channels);

  long sampleRate = waveFile.sampleRate();
  Serial.print("Sample rate = ");
  Serial.print(sampleRate);
  Serial.println(" Hz");

  long duration = waveFile.duration();
  Serial.print("Duration = ");
  Serial.print(duration);
  Serial.println(" seconds");

  // adjust the playback volume
  AudioOutI2S.volume(5);

  // check if the I2S output can play the wave file
  if (!AudioOutI2S.canPlay(waveFile)) {
    Serial.println("unable to play wave file using I2S!");
    while (1); // do nothing
  }

  // start playback
  Serial.println("starting playback");
  AudioOutI2S.play(waveFile);
}

void loop() {
  // check if playback is still going on
  if (!AudioOutI2S.isPlaying()) {
    // playback has stopped

    Serial.println("playback stopped");
    while (1); // do nothing
  }
}
```

### `SDWaveFile.bitsPerSample()`

#### Description
Query the bits per sample of the wave file

#### Syntax

```
waveFile.bitsPerSample()
```

#### Returns
The bits per sample of the wave file

#### Example

```
/*
 This reads a wave file from an SD card and plays it using the I2S interface to
 a MAX98357 I2S Amp Breakout board.

 Circuit:
 * Arduino Zero, MKR Zero or MKR1000 board
 * SD breakout or shield connected
 * MAX98357:
   * GND connected GND
   * VIN connected 5V
   * LRC connected to pin 0 (Zero) or pin 3 (MKR1000, MKR Zero)
   * BCLK connected to pin 1 (Zero) or pin 2 (MKR1000, MKR Zero)
   * DIN connected to pin 9 (Zero) or pin A6 (MKR1000, MKR Zero)

 created 15 November 2016
 by Sandeep Mistry
 */

#include <SD.h>
#include <ArduinoSound.h>

// filename of wave file to play
const char filename[] = "MUSIC.WAV";

// variable representing the Wave File
SDWaveFile waveFile;

void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  // setup the SD card, depending on your shield of breakout board
  // you may need to pass a pin number in begin for SS
  Serial.print("Initializing SD card...");
  if (!SD.begin()) {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println("initialization done.");

  // create a SDWaveFile
  waveFile = SDWaveFile(filename);

  // check if the WaveFile is valid
  if (!waveFile) {
    Serial.println("wave file is invalid!");
    while (1); // do nothing
  }

  // print out some info. about the wave file
  Serial.print("Bits per sample = ");
  Serial.println(waveFile.bitsPerSample());

  long channels = waveFile.channels();
  Serial.print("Channels = ");
  Serial.println(channels);

  long sampleRate = waveFile.sampleRate();
  Serial.print("Sample rate = ");
  Serial.print(sampleRate);
  Serial.println(" Hz");

  long duration = waveFile.duration();
  Serial.print("Duration = ");
  Serial.print(duration);
  Serial.println(" seconds");

  // adjust the playback volume
  AudioOutI2S.volume(5);

  // check if the I2S output can play the wave file
  if (!AudioOutI2S.canPlay(waveFile)) {
    Serial.println("unable to play wave file using I2S!");
    while (1); // do nothing
  }

  // start playback
  Serial.println("starting playback");
  AudioOutI2S.play(waveFile);
}

void loop() {
  // check if playback is still going on
  if (!AudioOutI2S.isPlaying()) {
    // playback has stopped

    Serial.println("playback stopped");
    while (1); // do nothing
  }
}
```

### `SDWaveFile.channels()`

#### Description
Query the number of channels in the wave file

#### Syntax

```
waveFile.channels()
```

#### Returns
The number of channels in the wave file

#### Example

```
/*
 This reads a wave file from an SD card and plays it using the I2S interface to
 a MAX98357 I2S Amp Breakout board.

 Circuit:
 * Arduino Zero, MKR Zero or MKR1000 board
 * SD breakout or shield connected
 * MAX98357:
   * GND connected GND
   * VIN connected 5V
   * LRC connected to pin 0 (Zero) or pin 3 (MKR1000, MKR Zero)
   * BCLK connected to pin 1 (Zero) or pin 2 (MKR1000, MKR Zero)
   * DIN connected to pin 9 (Zero) or pin A6 (MKR1000, MKR Zero)

 created 15 November 2016
 by Sandeep Mistry
 */

#include <SD.h>
#include <ArduinoSound.h>

// filename of wave file to play
const char filename[] = "MUSIC.WAV";

// variable representing the Wave File
SDWaveFile waveFile;

void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  // setup the SD card, depending on your shield of breakout board
  // you may need to pass a pin number in begin for SS
  Serial.print("Initializing SD card...");
  if (!SD.begin()) {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println("initialization done.");

  // create a SDWaveFile
  waveFile = SDWaveFile(filename);

  // check if the WaveFile is valid
  if (!waveFile) {
    Serial.println("wave file is invalid!");
    while (1); // do nothing
  }

  // print out some info. about the wave file
  Serial.print("Bits per sample = ");
  Serial.println(waveFile.bitsPerSample());

  long channels = waveFile.channels();
  Serial.print("Channels = ");
  Serial.println(channels);

  long sampleRate = waveFile.sampleRate();
  Serial.print("Sample rate = ");
  Serial.print(sampleRate);
  Serial.println(" Hz");

  long duration = waveFile.duration();
  Serial.print("Duration = ");
  Serial.print(duration);
  Serial.println(" seconds");

  // adjust the playback volume
  AudioOutI2S.volume(5);

  // check if the I2S output can play the wave file
  if (!AudioOutI2S.canPlay(waveFile)) {
    Serial.println("unable to play wave file using I2S!");
    while (1); // do nothing
  }

  // start playback
  Serial.println("starting playback");
  AudioOutI2S.play(waveFile);

void loop() {
}
  if (!AudioOutI2S.isPlaying()) {
    // playback has stopped

    Serial.println("playback stopped");
    while (1); // do nothing
  }
}
```

### `SDWaveFile.frames()`

  // check if playback is still going on
#### Description
Query the number of frames/samples in the wave file

#### Syntax

```
waveFile.frames();
```

#### Returns
The number of frames/samples in the wave file

#### Example

```
…
// create a SDWaveFile
  waveFile = SDWaveFile(filename);

  // check if the WaveFile is valid
  if (!waveFile) {
    Serial.println("wave file is invalid!");
    while (1); // do nothing
  }

  //….

  long frames = waveFile.frames();
  Serial.print("Frames = ");
  Serial.println(frames);
…
```

### `SDWaveFile.duration()`

#### Description
Query the duration of the wave file

#### Syntax

```
waveFile.duration();
```

#### Returns
The duration of the wave file in seconds

#### Example

```
/*
 This reads a wave file from an SD card and plays it using the I2S interface to
 a MAX98357 I2S Amp Breakout board.

 Circuit:
 * Arduino Zero, MKR Zero or MKR1000 board
 * SD breakout or shield connected
 * MAX98357:
   * GND connected GND
   * VIN connected 5V
   * LRC connected to pin 0 (Zero) or pin 3 (MKR1000, MKR Zero)
   * BCLK connected to pin 1 (Zero) or pin 2 (MKR1000, MKR Zero)
   * DIN connected to pin 9 (Zero) or pin A6 (MKR1000, MKR Zero)

 created 15 November 2016
 by Sandeep Mistry
 */

#include <SD.h>
#include <ArduinoSound.h>

// filename of wave file to play
const char filename[] = "MUSIC.WAV";

// variable representing the Wave File
SDWaveFile waveFile;

void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  // setup the SD card, depending on your shield of breakout board
  // you may need to pass a pin number in begin for SS
  Serial.print("Initializing SD card...");
  if (!SD.begin()) {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println("initialization done.");

  // create a SDWaveFile
  waveFile = SDWaveFile(filename);

  // check if the WaveFile is valid
  if (!waveFile) {
    Serial.println("wave file is invalid!");
    while (1); // do nothing
  }

  // print out some info. about the wave file
  Serial.print("Bits per sample = ");
  Serial.println(waveFile.bitsPerSample());

  long channels = waveFile.channels();
  Serial.print("Channels = ");
  Serial.println(channels);

  long sampleRate = waveFile.sampleRate();
  Serial.print("Sample rate = ");
  Serial.print(sampleRate);
  Serial.println(" Hz");

  long duration = waveFile.duration();
  Serial.print("Duration = ");
  Serial.print(duration);
  Serial.println(" seconds");

  // adjust the playback volume
  AudioOutI2S.volume(5);

  // check if the I2S output can play the wave file
  if (!AudioOutI2S.canPlay(waveFile)) {
    Serial.println("unable to play wave file using I2S!");
    while (1); // do nothing
  }

  // start playback
  Serial.println("starting playback");
  AudioOutI2S.play(waveFile);
}

void loop() {
  // check if playback is still going on
  if (!AudioOutI2S.isPlaying()) {
    // playback has stopped

    Serial.println("playback stopped");
    while (1); // do nothing
  }
}
```

### `SDWaveFile.currentTime()`

#### Description
Query the current position of playback of the wave file

#### Syntax

```
waveFile.currentTime();
```

#### Returns
The current playback position of the wave file in seconds

SDWaveFile.cue()`

#### Description
Change the playback position of the wave file

#### Syntax

```
waveFile.cue(time);

```

#### Parameters
time: time is seconds to change playback to

#### Returns
1 on is the playback position was cued successfully, 0 on failure

#### Example

```
…
  // create a SDWaveFile
  waveFile = SDWaveFile(filename);

  // check if the WaveFile is valid
  if (!waveFile) {
    Serial.println("wave file is invalid!");
    while (1); // do nothing
  }

  /// play the file using an audio output ...

  // jump back to 10 second into the wave file
  waveFile.cue(10);
…
```

### `