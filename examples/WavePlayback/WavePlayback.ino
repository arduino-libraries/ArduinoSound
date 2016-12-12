/*
 This reads a wave file from an SD card and plays it using the I2S interface to
 a MAX08357 I2S Amp Breakout board.

 Circuit:
 * Arduino/Genuino Zero, MKRZero or MKR1000 board
 * SD breakout or shield connected
 * MAX08357:
   * GND connected GND
   * VIN connected 5V
   * LRC connected to pin 0 (Zero) or pin 3 (MKR1000, MKRZero)
   * BCLK connected to pin 1 (Zero) or pin 2 (MKR1000, MKRZero)
   * DIN connected to pin 9 (Zero) or pin A6 (MKR1000, MKRZero)

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
