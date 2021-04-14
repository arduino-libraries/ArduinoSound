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
