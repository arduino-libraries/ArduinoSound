/*
 Example for Espressif LyraT board
 This example records few second using either onboard microphones, or connected microphone
 saves the recording to SD card and then plays it back to line out (connect headphones or speaker)

 Setup:
   Make sure your micro SD card is formated in FAT32
   Put micro SD card in card slot on LyraT board
   Make sure dip switch on board has pins 1 and 2 in ON position and reamining pins in OFF position
   Connect headphones or speaker to PHONEJACK
   If you want to use external microphone connect it to AUX_IN

 Flashing:
   When successfully compiled press and hold Boot button and shortly press RST button.
   Flashing should start shortly after releasing both buttons.
   After successfull flashing press shortly RST button

 Usage:
   After flashing and restarting say something to microphone and listen the playback.
   You can later use the file recorded on SD card in your computer or elswhere.

 created 23 October 2020
 by Tomas Pilny
 */

#include "SD_MMC.h"
#include <ArduinoSound.h>


// filename of wave file to record into and playback
const char filename[] = "/test.wav"; // note: the filename must start with '/' (slash)

// variable representing the Wave File
SDWaveFile waveFile;

// SD card interface
SPIClass sdspi(HSPI);

// Class controlling codec chip on LyraT board
ES8388 *codec_chip;

/**
 * @brief Record audio and save to SD card in WAV format
 *
 * @param   filename        file name on SD card starting with '/'
 *
 * @param   duration        required length of recording in seconds
 *
 * @param   bitsPerSample   number of bits per sample. For example 8,16,32
 *
 * @param   sampleRate      frequency of samples. For example 8000,16000,44100
 *
 * @return:
 *     - true    success
 *     - false   failure
 *
 * Function is expecting initialized SD card connection.
 * Function attempts to create new file on SD card.
 * Based on function parameters then initializes I2S and codec chip.
 * After successfull init the function then records audio in given file.
 */
bool record_wav_file(const char filename[], int duration, int bitsPerSample, long sampleRate, bool use_external_mic){
  bool ret = false;

  if(!codec_chip->inBegin(sampleRate, bitsPerSample, use_external_mic)){ // Config codec for input
    Serial.println("ERROR: Could not initialize codec chip for input");
    return 0; // ERR
  }

  waveFile = SDWaveFile(filename);
  waveFile.purgeTmp();
  ret = waveFile.initWrite(bitsPerSample, sampleRate);
  if(!ret){
    Serial.println("ERROR: Could not initialize tmp file");
    codec_chip->end();
    return ret;
  }
  int bytesToWrite = 0;
  int buffer_size = 256;
  uint8_t data[buffer_size];

  bool finished = false;
  unsigned long startMillis = millis();
  unsigned long timeElapsed = 0;
  int prev_sec = 0; // help variable for printing remaing time
  Serial.print("Recording started (");Serial.print(duration);Serial.println("s)");
  do{
    bytesToWrite = codec_chip->read(data, buffer_size);
    if(timeElapsed >= duration*1000){
      finished = true;
      ret = true;
    }
    if(finished){
      Serial.println("Recording finished - saving file (this may take a while)");
    }
    ret = waveFile.writeData(data, bytesToWrite, finished);
    if(!ret){
      Serial.println("ERROR: While writing");
      codec_chip->end();
      return ret;
    }
    if(prev_sec != timeElapsed/1000){
    Serial.print("Remaining time ");Serial.print(duration-timeElapsed/1000);Serial.println(" s");
      prev_sec = timeElapsed/1000;
    }
    timeElapsed = millis() - startMillis;
  }while(!finished); // write loop
  Serial.println("Recording finished");
  codec_chip->end();
  return ret;
}

/**
 * @brief Play wav file from SD card
 *
 * @param   filename     file name on SD card starting with '/'
 *
 * @return:
 *     - true    success
 *     - false   failure
 *
 * Function is expecting initialized SD card connection.
 * Function then attempts to open given filename and read file header.
 * Based on file header then initializes I2S and codec chip.
 * After successfull init the function then plays given file.
 * For demostration this function also prints its progress and file header
 */
bool play_wav_file(const char filename[]){
  // Create a SDWaveFile
  waveFile = SDWaveFile(filename);

  // Check if the WaveFile is valid
  if (!waveFile) {
    Serial.println("wave file is invalid!");
    return false;
  }

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

  if(!codec_chip->outBegin(waveFile.sampleRate(), waveFile.bitsPerSample())){// Config codec for playback
    Serial.println("ERROR - could not initialize codec chip for output");
    return false;
  }

  // Adjust the playback volume
  Serial.println("set volume...");
  codec_chip->volume(100.0); // Set maximum volume (100%)

  // Check if the I2S output can play the wave file
  if (!codec_chip->canPlay(waveFile)) {
    Serial.println("Unable to play wave file using I2S!");
    return false;
  }

  // start playback
  Serial.println("Starting playback");
  codec_chip->play(waveFile);

  while(true){
    if(!codec_chip->isPlaying()){
      Serial.println("Playback stopped");
      return true;
    }else{
      codec_chip->transmit(); // send data to audio output
    } // if isPlaying
  } // playback loop
  codec_chip->end();
} // play_wav_file()

void setup() {
  disableCore1WDT();
  // Open serial communications and wait for port to open:
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  Serial.print("Initializing SD card...");
  sdspi.begin(14, 2, 15); // Pin configuration
  if (!SD.begin(13, sdspi)) {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println(" SD initialized");

  // Init I2C for codec setup
  Serial.println("Initialize wire for codec chip");
  Wire.begin(GPIO_NUM_18, GPIO_NUM_23);
  codec_chip = new ES8388(GPIO_NUM_21, Wire);

  Serial.println("Initialization done.");
}

void loop() {
  bool use_external_mic = false;
  int recordDuration = 5; // number of seconds to keep recording
  if(!record_wav_file(filename, recordDuration, 24, 32000, use_external_mic)){
    Serial.println("Record failed!");
  }else{ // Recording succeeded - proceed to playback
    if(!play_wav_file(filename)){
      Serial.println("Playback failed!");
    }
  }
  delay(1000);
}
