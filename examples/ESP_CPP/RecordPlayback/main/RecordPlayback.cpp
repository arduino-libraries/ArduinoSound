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
char test_file[] = "/test.wav"; // note: the filename must start with '/' (slash)

// variable representing the Wave File
SDWaveFile waveFile;

// SD card interface
SPIClass sdspi(HSPI);

// Class controlling codec chip on LyraT board
ES8388 *codec_chip;

// Separate I2S read (record) and SD write to different cores
TaskHandle_t Core0Task; // SD write task handle
void Core0TaskCode(void * parameter); // SD write task

const int buffer_size = 512; // Size of array to store I2S samples in Queue
typedef struct buffer {
  uint8_t data[buffer_size];
  bool finished;
} buffer_t;
typedef struct new_file_str {
  int bitsPerSample;
  long sampleRate;
  char *filename;
} new_file_t;
QueueHandle_t xQueue = NULL; // queue for comuniacting between tasks (cores)
#define INCLUDE_vTaskSuspend 1 // enable indefinite wait for queue element
#define INCLUDE_xSemaphoreGetMutexHolde 1 // enable peeking at semaphore holder
SemaphoreHandle_t xSemaphore; // Semaphore to synchronise I2S read and SD write tasks
SemaphoreHandle_t sem_sd_ready; // Semaphore to signal SD thread is ready to receive I2S data
SemaphoreHandle_t sem_sd_saved; // Semaphore to signal SD is done saving file
QueueHandle_t i2s_data_queue = NULL; // queue holding I2S data

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
 *     - 1    success
 *     - 0    failure
 *
 * Function is expecting initialized SD card connection.
 * Function attempts to create new file on SD card.
 * Based on function parameters then initializes I2S and codec chip.
 * After successfull init the function then records audio in given file.
 */
int record_wav_file(char filename[], int duration, int bitsPerSample, long sampleRate, bool use_external_mic){
  int ret = 0; // ERR

  if(!codec_chip->inBegin(sampleRate, bitsPerSample, use_external_mic)){ // Config codec for input
    Serial.println("ERROR: Could not initialize codec chip for input");
    return 0; // ERR
  }
  // Create and fill data structure handed over to SD write task
  new_file_t new_file;
  new_file.filename = filename;
  new_file.bitsPerSample = bitsPerSample;
  new_file.sampleRate = sampleRate;
  if(pdPASS != xQueueSendToBack(xQueue, &new_file, 10)){
    Serial.println("ERROR: starting record task");
    return 0;
  }

  unsigned long startMillis = millis();
  unsigned long timeElapsed = 0;
  int prev_sec = 0; // help variable for printing remaing time
  Serial.print("Recording started (");Serial.print(duration);Serial.println("s)");
  buffer_t buf;

  // wait untill the semaphore is taken by other task - signaling it is ready to write
  while(xSemaphoreTake(sem_sd_ready, 1000) != pdTRUE);
  do{
    codec_chip->read(buf.data, buffer_size);
    buf.finished = false;

    if(timeElapsed >= duration*1000){
      ret = 0; // ERR
      buf.finished = true;
    }
    if(pdPASS != xQueueSendToBack(i2s_data_queue, &buf, 1000)){
      Serial.println("ERROR: sending I2S data to queue");
      codec_chip->end();
      return 0; // ERR
    }
    if(prev_sec != timeElapsed/1000 && (duration-timeElapsed/1000) != 0){
      Serial.print("Remaining time ");Serial.print(duration-timeElapsed/1000);Serial.println(" s");
      prev_sec = timeElapsed/1000;
    }
    timeElapsed = millis() - startMillis;
  }while(!buf.finished); // I2S record loop
  codec_chip->end();

  // wait untill the semaphore is given by other task - signaling it is done saving WAV file to SD card
  while(xSemaphoreTake(sem_sd_saved, 1000) != pdTRUE);
  xSemaphoreGive(sem_sd_ready); // release for another run
  xSemaphoreGive(sem_sd_saved); // release for another run

  Serial.println("Recording saved");
  return ret;
}

// Task running on second core taking care of SD writes
// No need to call anywhere - this is taken care of in setup
void Core0TaskCode(void * parameter){
  uint8_t *SD_write_buffer;
  uint SD_buffer_size = 40960; // 40kB is most optimal for SD write
  uint SD_buffer_pointer;
  while (!Serial) {
    ; // wait for serial port to connect
  }

  do{
    SD_write_buffer = (uint8_t*) malloc(SD_buffer_size * sizeof(uint8_t));
    if(SD_write_buffer == NULL){
      SD_buffer_size /= 2;
      if(SD_buffer_size <= 256){
        Serial.println("ERROR: Unable to allocate buffer for SD write.");
        vTaskDelete(NULL);
      } // trying too small buffer
    } // malloc failed
  }while(SD_write_buffer == NULL);

  new_file_t new_file;
  buffer_t buf;
  while(1){
    if(xSemaphoreTake(sem_sd_ready, 1000) != pdTRUE){
      Serial.println("Error takign semaphore sem_sd_ready");
    }
    if(xSemaphoreTake(sem_sd_saved, 1000) != pdTRUE){
      Serial.println("Error takign semaphore sem_sd_saved");
    }
    // receive filename from queue - wait indefinitely (no timeout)
    if(xQueueReceive(xQueue, &new_file, portMAX_DELAY) == pdPASS){
      SD_buffer_pointer = 0;
      waveFile = SDWaveFile(new_file.filename);
      waveFile.purgeTmp();
      int ret;
      ret = waveFile.initWrite(new_file.bitsPerSample, new_file.sampleRate);
      bool exit = false;
      if(!ret){
        Serial.println("ERROR: Could not initialize tmp file");
        codec_chip->end();
        exit = true;
      }
      xSemaphoreGive(sem_sd_ready);
      while(! exit){
        if(xQueueReceive(i2s_data_queue, &buf, portMAX_DELAY) == pdPASS){
          memcpy(SD_write_buffer + SD_buffer_pointer, buf.data, buffer_size);
          SD_buffer_pointer +=buffer_size;
          if(SD_buffer_pointer == SD_buffer_size || buf.finished){
            ret = waveFile.writeData(SD_write_buffer, SD_buffer_pointer, buf.finished);
            SD_buffer_pointer = 0;
          }
          if(!ret){
            Serial.println("ERROR: While writing");
            codec_chip->end();
            exit = true;
          } // write error ?
          if(buf.finished){
            exit = true;
            Serial.println("Saving done");
            xSemaphoreGive(sem_sd_saved); // Notify other task we finished saving file
          }
        } // queue not empty
      } // main execution loop while(! exit)
    } // Queue received
  } // inifinite loop
} //Core0TaskCode

/**
 * @brief Play wav file from SD card
 *
 * @param   filename     file name on SD card starting with '/'
 *
 * @return:
 *     - 1    success
 *     - 0   failure
 *
 * Function is expecting initialized SD card connection.
 * Function then attempts to open given filename and read file header.
 * Based on file header then initializes I2S and codec chip.
 * After successfull init the function then plays given file.
 * For demostration this function also prints its progress and file header
 */
int play_wav_file(const char filename[]){
  // Create a SDWaveFile
  waveFile = SDWaveFile(filename);

  // Check if the WaveFile is valid
  if (!waveFile) {
    Serial.println("wave file is invalid!");
    return 0; // ERR
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
    return 0; // ERR
  }

  // Adjust the playback volume
  Serial.println("set volume...");
  codec_chip->volume(100.0); // Set maximum volume (100%)

  // Check if the I2S output can play the wave file
  if (!codec_chip->canPlay(waveFile)) {
    Serial.println("Unable to play wave file using I2S!");
    return 0; // ERR
  }

  // start playback
  Serial.println("Starting playback");
  codec_chip->play(waveFile);

  while(true){
    if(!codec_chip->isPlaying()){
      Serial.println("Playback stopped");
      return 1; // OK
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

  xTaskCreatePinnedToCore(
    Core0TaskCode, /* Function to implement the task */
    "Core0Task", /* Name of the task */
    10000,  /* Stack size in words */
    NULL,  /* Task input parameter */
    0,  /* Priority of the task */
    &Core0Task,  /* Task handle. */
    0); /* Core where the task should run */

  xQueue = xQueueCreate(2, sizeof(new_file_t));// create Queue with 2 elemts for new file structure

  // Create and give semaphores used to synchronize I2S recording with SD writting
  sem_sd_ready = xSemaphoreCreateBinary();
  xSemaphoreGive(sem_sd_ready);
  sem_sd_saved = xSemaphoreCreateBinary();
  xSemaphoreGive(sem_sd_saved);

  i2s_data_queue = xQueueCreate(50, sizeof(buffer_t)); // Create Queue for I2S data samples sent to second core
  Serial.println("Initialization done.");
}

void loop() {
  bool use_external_mic = false;
  int recordDuration = 5; // number of seconds to keep recording
  int bitsPerSample = 16;
  int bitrate = 16000;
  if(!record_wav_file(test_file, recordDuration, bitsPerSample, bitrate, use_external_mic)){
    Serial.println("Record failed!");
  }else{ // Recording succeeded - proceed to playback
    if(!play_wav_file(test_file)){
      Serial.println("Playback failed!");
    }
  }
  delay(1000);
}
