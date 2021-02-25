/*
 Example for Espressif LyraT board
 This example records few second using either onboard microphones, or connected microphone
 saves the recording to SD card and then plays it back to line out (connect headphones or speaker)

 Setup:
   Make sure your micro SD card is formatted in FAT32
   Put micro SD card in card slot on LyraT board
   Make sure dip switch on board has pins 1 and 2 in ON position and remaining pins in OFF position
   Connect headphones or speaker to PHONEJACK
   If you want to use external microphone connect it to AUX_IN

 Flashing:
   When successfully compiled press and hold Boot button and shortly press RST button.
   Flashing should start shortly after releasing both buttons.
   After successful flashing press shortly RST button

 Usage:
   After flashing and restarting say something to microphone and listen the playback.
   You can later use the file recorded on SD card in your computer or elsewhere.

 created 23 October 2020
 by Tomas Pilny
 */

#include "SD_MMC.h"
#include <ArduinoSound.h>

// filename of wave file to record into and playback
char test[] = "/test.wav"; // note: the filename must start with '/' (slash)

// SD card interface
SPIClass sdspi(HSPI);

// Class controlling codec chip on LyraT board
ES8388 *codec_chip;

// Separate I2S controll and SD read/write to different cores
TaskHandle_t Core0Task;
void Core0TaskCode(void * parameter);
const int buffer_size = 512;

typedef struct buffer {
  uint8_t data[buffer_size];
  bool finished;
} buffer_t;
QueueHandle_t xQueue = NULL; // queue for comuniacting between tasks (cores)
#define INCLUDE_vTaskSuspend 1 // enable indefinite wait for queue element
EventGroupHandle_t xEventBits;
/* Synchronization flags */
#define SD_TASK_RDY_BIT_0   ( 1 << 0 )
#define SD_TASK_SAVED_BIT_1 ( 1 << 1 )
#define SD_TASK_ERROR_BIT_2 ( 1 << 2 )

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
 *     - true    success
 *     - false   failure
 *
 * Function is expecting initialized SD card connection.
 * Function attempts to create new file on SD card.
 * Based on function parameters then initializes I2S and codec chip.
 * After successfull init the function then records audio in given file.
 */
uint SD_buffer_size = 40960; // 40kB is most optimal for SD write
void Core0TaskCode( void * parameter){
  SDWaveFile *waveFile = (SDWaveFile*)parameter;
  uint8_t *SD_write_buffer;
  uint SD_buffer_pointer;
  int ret = 1;
  do{
    SD_write_buffer = (uint8_t*) malloc(SD_buffer_size * sizeof(uint8_t)+buffer_size); // try to init 40kB + one small buffer size
    if(SD_write_buffer == NULL){
      SD_buffer_size /= 2;
      if(SD_buffer_size <= 256){
        Serial.println("ERROR: Unable to allocate buffer for SD write.");
        xEventGroupSetBits(xEventBits, SD_TASK_ERROR_BIT_2); // Notify other task we have encountered error
        vTaskDelete(NULL); // terminate task
      } // trying too small buffer
    } // malloc failed
  }while(SD_write_buffer == NULL);

  buffer_t buf;
  SD_buffer_pointer = 0;
  bool exit = false;
  xEventGroupSetBits(xEventBits, SD_TASK_RDY_BIT_0); // Notify other task we are ready

  while(! exit){
    if(xQueueReceive(i2s_data_queue, &buf, portMAX_DELAY) == pdPASS){
      memcpy(SD_write_buffer + SD_buffer_pointer, buf.data, buffer_size);
      SD_buffer_pointer += buffer_size;

      if(SD_buffer_pointer >= SD_buffer_size || buf.finished){
        ret = waveFile->writeData(SD_write_buffer, SD_buffer_pointer, buf.finished); // 1 == OK, 0 == ERR
        SD_buffer_pointer = 0;
        if(!ret){
          xEventGroupSetBits(xEventBits, SD_TASK_ERROR_BIT_2); // Notify other task we have encountered error
          Serial.print("ERROR: While writing");
          exit = true;
        } // write error ?
      }
      if(buf.finished){
        exit = true;
        Serial.println("Saving done");
        xEventGroupSetBits(xEventBits, SD_TASK_SAVED_BIT_1); // Notify other task we have finnished saving Wav file
      }
    } // queue not empty
  } // main execution loop while(! exit)
  vTaskDelete(NULL);
} //Core0TaskCode

bool record_wav_file(const char filename[], int duration, int bitsPerSample, long sampleRate, bool use_external_mic){
  SDWaveFile waveFile;
  EventBits_t uxReturn;
  xEventBits = xEventGroupCreate();
  bool ret = false;
  if(!codec_chip->inBegin(sampleRate, bitsPerSample, use_external_mic)){ // Config codec for input
    Serial.println("ERROR: Could not initialize codec chip for input");
    return 0; // ERR
  }
  waveFile = SDWaveFile(filename);
  waveFile.purgeTmp(); // TODO optimize this if card contains lots of files (then it take a lot of time)
  ret = waveFile.initWrite(bitsPerSample, sampleRate);

  if(!ret){
    Serial.println("ERROR: Could not initialize tmp file");
    codec_chip->end();
    return 0; // ERR
  }

  xTaskCreatePinnedToCore(
    Core0TaskCode, /* Function to implement the task */
    "Core0Task", /* Name of the task */
    10000,  /* Stack size in words */
    (void*)(&waveFile),  /* Task input parameter */
    0,  /* Priority of the task */
    &Core0Task,  /* Task handle. */
    0); /* Core where the task should run */

  uxReturn = xEventGroupWaitBits(xEventBits,
                                 SD_TASK_RDY_BIT_0, // wait untill SD task redy to receive
                                 pdFALSE, // Don't clear received bits
                                 pdFALSE, // logacl OR for waiting bits
                                 portMAX_DELAY ); // Wait indefintely
  if((uxReturn & SD_TASK_RDY_BIT_0) != SD_TASK_RDY_BIT_0){
    Serial.println("ERROR: I2S Task reached point that should not happend - waiting untill sd rdy, but after wait flag not set");
    Serial.print("flags = 0x"); Serial.println(xEventGroupGetBits(xEventBits),HEX);
    codec_chip->end();
    return 0; // ERR
  }
  Serial.print("Recording started (");Serial.print(duration);Serial.println("s)");
  buffer_t buf;

  unsigned long startMillis = millis();
  unsigned long timeElapsed = 0;
  int prev_sec = 0; // help variable for printing remaing time
  do{
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    if((xEventGroupGetBits(xEventBits) & SD_TASK_ERROR_BIT_2 ) == SD_TASK_ERROR_BIT_2){
      Serial.println("SD task encountered error - exit main function");
      codec_chip->end();
      return 0; // ERR
    }

    codec_chip->read(buf.data, buffer_size);
    buf.finished = false;

    if(timeElapsed >= duration*1000){
      ret = true;
      buf.finished = true;
    }
    if(pdPASS != xQueueSendToBack(i2s_data_queue, &buf, 10)){
      Serial.println("WARNING: Failed to send I2S data to queue in 10 CLKs;");
      Serial.print  ("         Waiting members = ");Serial.println(uxQueueMessagesWaiting(i2s_data_queue));
      Serial.println("         Try to send again and wait up to 100 CLKs");
      if(pdPASS != xQueueSendToBack(i2s_data_queue, &buf, 100)){
        Serial.println("ERROR: sending I2S data to queue");
        codec_chip->end();
        return 0; // ERR
      }
    }
    if(prev_sec != timeElapsed/1000 && (duration-timeElapsed/1000) != 0){
      Serial.print("Remaining time ");Serial.print(duration-timeElapsed/1000);Serial.println(" s");
      prev_sec = timeElapsed/1000;
    }
    timeElapsed = millis() - startMillis;
  }while(!buf.finished); // write loop
  codec_chip->end(); // TODO uncoment for normal usage

  uxReturn = xEventGroupWaitBits(xEventBits,
                                 SD_TASK_SAVED_BIT_1, // wait untill SD task saves file
                                 pdFALSE, // Don't clear received bits
                                 pdFALSE, // logacl OR for waiting bits
                                 portMAX_DELAY); // Wait indefintely
  if( ( uxReturn & SD_TASK_SAVED_BIT_1 ) != SD_TASK_SAVED_BIT_1 ){
    Serial.println("ERROR: I2S Task reached point that should not happend - waiting untill sd finished, but after flag was not set");
    Serial.print("flags = 0x"); Serial.println(xEventGroupGetBits(xEventBits),HEX);
    return 0; // ERR
  }
  vEventGroupDelete(xEventBits);

  Serial.println("Recording saved");
  return ret;
}

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
  Serial.print("Playing file "); Serial.println(filename);
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
      codec_chip->end();
      return true;
    }else{
      codec_chip->transmit(); // send data to audio output
    } // if isPlaying
  } // playback loop
} // play_wav_file()

void setup() {
  disableCore0WDT();
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

  i2s_data_queue = xQueueCreate(150, sizeof(buffer_t));

  Serial.println("Initialization done.");
}

void loop() {
  bool use_external_mic = false;
  int recordDuration = 5; // number of seconds to keep recording
  if(!record_wav_file(test, recordDuration, 24, 44100, use_external_mic)){
    Serial.println("Record failed!");
  }else{ // Recording succeeded - proceed to playback
    if(!play_wav_file(test)){
      Serial.println("Playback failed!");
    }
  }
  delay(1000);
}
