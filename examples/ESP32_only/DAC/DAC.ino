/*
 This example demostrates usage of builtin DAC and it's usage for playing audio.

 Hardware:
   - Any ESP device
   - Audio amplifier
     - please keep attantion to intended impedance and wattege of speakers
       * Speaker wattege must be less or equal to wattage of amplifier
       * Impedance of speakers must match exactly impedance of amplifier
   - SD card reader + SD card
   - Headphones or speares according to chosen amplifier

 Setup:
   Connect your audio amplifier to ESP.
    - Note:
      * ESP32 has DAC on GPIO pins 25 and 26.
      * ESP32-S2 has DAC on GPIO pins 17 and 18.
   Connect speaker(s) or headpghones.
   Connect SD card reader module:
    - ESP32: MISO 19, MOSI 23, SCK 18, CS 5
   Load audio file in WAV format on SD card and insert card in the connected SD module
   Change test_file value to your audio file name
   If you want to trigger the playback with button press:
    - Connect your button to GPIO 12 and GND
    - Uncomment line #define USE_BUTTON located below this comment block
   Connect ESP to computer and flash this example

 Flashing:
   If upload doesnt start automatically press and hold Boot button and shortly press RST button.
   Flashing should start shortly after releasing both buttons.
   After successfull flashing press shortly RST button

 Usage:
   After flashing and restarting you should hear sound you uploaded with the code.
   If you chose to use button press it and the sound should be played.

 created 13 November 2020
 by Tomas Pilny
 */
#include <ArduinoSound.h>
#include "SD_MMC.h"

//#define USE_BUTTON
#ifdef USE_BUTTON
  const int button_pin = GPIO_NUM_12;
#endif

// filename of wave file to record into and playback
char test_file[] = "/test.wav"; // note: the filename must start with '/' (slash)

// SD card interface
SPIClass sdspi(HSPI);

/**
 * @brief Play wav file from SD card
 * @param filename - file name on SD card starting with '/'
 * @return true  on success <br>
 *         false on failure
 *
 * Function is expecting initialized SD card connection.
 * Function then attempts to open given filename and read file header.
 * Based on file header then initializes I2S and codec chip.
 * After successfull init the function then plays given file.
 * For demostration this function also prints its progress and file header
 */
bool play_wav_file(const char filename[]){
  // Create a SDWaveFile
  SDWaveFile waveFile = SDWaveFile(filename);
  // Check if the WaveFile is valid
  if (!waveFile) {
    Serial.print("Wave file \""); Serial.print(filename); Serial.println("\" is invalid!");
    return false;
  }
  Serial.print("Playing file "); Serial.println(filename);

  // Print out wave file header
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

  if(!AudioOutI2S.beginDAC(waveFile.sampleRate())){// Config I2S for playback with DAC
    Serial.println("ERROR - could not initialize DAC");
    return false;
  }

  // Check if the I2S output can play the wave file
  if (!AudioOutI2S.canPlay(waveFile)) {
    Serial.println("Unable to play wave file using I2S!");
    return false;
  }

  // start playback
  Serial.println("Starting playback");
  AudioOutI2S.play(waveFile);

  while(true){
    if(!AudioOutI2S.isPlaying()){
      Serial.println("Playback stopped");
      AudioOutI2S.stop();
      return true;
    }else{
      AudioOutI2S.transmit(); // send data to audio output
    } // if isPlaying
  } // playback loop
} // play_wav_file()

void setup() {
  //disableCore1WDT();
  // Open serial communications and wait for port to open:
  Serial.begin(115200);

  Serial.print("Initializing SD card...");
  sdspi.begin(18, 19, 23, 5); // Pin configuration
  if (!SD.begin(5, sdspi)) {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println(" SD initialized");

  #ifdef USE_BUTTON
    pinMode(button_pin, INPUT_PULLUP);
  #endif
  Serial.println("Initialization done.");
}

void loop() {
  #ifdef USE_BUTTON
    Serial.println("Press button to play");
    static bool button_current_state, button_previous_state;
    button_previous_state = true;
    while(true){
      button_current_state = digitalRead(button_pin);
      if(button_previous_state == true && button_current_state == false){
        if(!play_wav_file(test_file)){
          Serial.println("Playback Failed");
        }else{
          Serial.println("Press button to play");
        }
      }
      button_previous_state = button_current_state;
    } // infinite loop
  #else
    if(!play_wav_file(test_file)){
      Serial.println("Playback Failed");
    }
    Serial.println("Another playback in 5s");
    delay(5000);
  #endif
}
