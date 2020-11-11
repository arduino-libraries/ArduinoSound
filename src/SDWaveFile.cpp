/*
  Copyright (c) 2016 Arduino LLC. All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
*/

#include "SDWaveFile.h"

struct SubChunkHeader {
  uint32_t id;
  uint32_t size;
};

// based on: http://soundfile.sapp.org/doc/WaveFormat/
struct WaveFileHeader {
  uint32_t chunkId;
  uint32_t chunkSize;
  uint32_t format;
  struct {
    struct SubChunkHeader header;
    uint16_t audioFormat;
    uint16_t numChannels;
    uint32_t sampleRate;
    uint32_t byteRate;
    uint16_t blockAlign;
    uint16_t bitsPerSample;
  } subChunk1;
  struct SubChunkHeader subChunk2Header;
} __attribute__((packed));

SDWaveFile::SDWaveFile() : 
  SDWaveFile(NULL)
{
}

SDWaveFile::SDWaveFile(const char* filename) :
  _headerRead(false),
  _isValid(false),
  _isPlaying(false),
  _filename(filename),

  _sampleRate(-1),
  _bitsPerSample(-1),
  _channels(-1),
  _frames(-1),
  _dataOffset(0)
{

}

SDWaveFile::SDWaveFile(const String& filename) :
  SDWaveFile(filename.c_str())
{

}

SDWaveFile::~SDWaveFile() {
}

SDWaveFile::operator bool()
{
  if (!_headerRead) {
    readHeader();
  }

  return _filename && _isValid;
}

long SDWaveFile::sampleRate()
{
  if (!_headerRead) {
    readHeader();
  }

  return _sampleRate;
}

int SDWaveFile::bitsPerSample()
{
  if (!_headerRead) {
    readHeader();
  }

  return _bitsPerSample;
}

int SDWaveFile::channels()
{
  if (!_headerRead) {
    readHeader();
  }

  return _channels;
}

long SDWaveFile::frames()
{
  if (!_headerRead) {
    readHeader();
  }

  return _frames;
}

long SDWaveFile::duration()
{
  if (!_headerRead) {
    readHeader();
  }

  return (_frames / _sampleRate);
}

long SDWaveFile::currentTime()
{
  if (!_isPlaying) {
    return -1;
  }

  uint32_t position = _file.position();

  if (position >= _dataOffset) {
    position -= _dataOffset;
  }

  return (position) / (_blockAlign * _sampleRate);
}

int SDWaveFile::cue(long time)
{
  if (time < 0) {
    return 1;
  }

  long offset = (time * _blockAlign) - _dataOffset;

  if (offset < 0) {
    offset = 0;
  }

  // make sure it's multiple of 512
  offset = (offset / 512) * 512;

  if ((uint32_t)offset > _file.size()) {
    return 1;
  }

  _file.seek(offset);

  return 0;
}

int SDWaveFile::begin()
{
  if (!(*this)) {
    return 0;
  }

  _file = SD.open(_filename);

  _isPlaying = true;

  return 1;
}

int SDWaveFile::read(void* buffer, size_t size)
{
  //Serial.print("SDWaveFile::read: _file =");Serial.println(_file);
  if (!_file){
    //Serial.println("SDWaveFile::read: WARNING - file not opened! Opening...");
    _file = SD.open(_filename);
    if (!_file){
      //Serial.println("SDWaveFile::read: ERROR - file cannot be opened!");
      return -1;
    }
    //Serial.println("Ok opened");
  }

  uint32_t position = _file.position();
  //Serial.print("SDWaveFile::read: cursor position =");Serial.println(position);
  //Serial.print("SDWaveFile::read: up to size =");Serial.println(size);
  int read = _file.read((uint8_t*) buffer, size);
  //Serial.print("SDWaveFile::read: actual read =");Serial.println(read);
  if (read > 0) {
/*
  Serial.println("Read buffer ");

    for(int i = 0; i < read; ++i){
      Serial.print(((uint8_t *)buffer)[i]);Serial.print(" ");
    }
    Serial.println("");
*/
  if (position == 0) {
    // replace the header with 0's
    //Serial.println("SDWaveFile::read: position == 0 => replace the header with 0's");
    memset(buffer, 0x00, _dataOffset);
  }
/*
  Serial.flush();
  Serial.println("Reading buffer ");
  for(int i = 0; i < read; ++i){
    Serial.print(((uint8_t *)buffer)[i]);Serial.print(" ");
  }
  Serial.println("");
*/

     // data here ok
  //if (read) {
    //Serial.println("SDWaveFile::read: samplesRead(buffer, read);");
    samplesRead(buffer, read);
  }
// data here fucked up, but NOT always :-/
/*
   Serial.println("Reading buffer ");
   for(int i = 0; i < read/2; ++i){
     Serial.print(((uint8_t *)buffer)[i]);Serial.print(" ");
   }
   Serial.println("");
*/
  //Serial.print("SDWaveFile::read: return read=");Serial.println(read);Serial.flush();
  return read;
}

int SDWaveFile::reset()
{
  cue(0);

  return 1;
}

void SDWaveFile::end()
{
  _isPlaying = false;

  _file.close();
}

void SDWaveFile::readHeader()
{
  _isValid = false;

  if (_headerRead) { // Already read
    return;
  }

  _file = SD.open(_filename);

  if (!_file) { // Failed to open
    Serial.println("failed to open file");
    return;
  }

  uint32_t fileSize = _file.size();

  Serial.print("fileSize = ");  Serial.println(fileSize);

  if (fileSize < sizeof(struct WaveFileHeader)) {
    _file.close();
    Serial.println("file smaller than header size");
    return;
  }

  struct WaveFileHeader header;
  int headerSize;
  int subChunk2Offset = 0;
  struct SubChunkHeader sch;

  headerSize = sizeof(struct WaveFileHeader) - sizeof(header.subChunk2Header);
  if (_file.read((uint8_t *)&header, headerSize) != headerSize) {
    _file.close();
    Serial.println("err reading header");
    return;
  }

  header.chunkId = __REV(header.chunkId);
  header.format = __REV(header.format);
  header.subChunk1.header.id = __REV(header.subChunk1.header.id);

  if (header.chunkId != 0x52494646) { // "RIFF"
    _file.close();
    Serial.println("not RIFF");
    Serial.print("header.chunkId = "); Serial.println(header.chunkId);
    return;
  }

  if ((fileSize - 8) != header.chunkSize) {
     Serial.println("file size does not match");
     Serial.print("file size - 8 = "); Serial.println(fileSize-8);
     Serial.print("header.chunkSize = "); Serial.println(header.chunkSize);
     _file.close();
    return;
  }

  if (header.format != 0x57415645) { // "WAVE"
     _file.close();
     Serial.println("not WAVE");
     Serial.print("actual value");Serial.println(header.format,HEX);
    return;
  }

  if (header.subChunk1.header.id != 0x666d7420) { // "fmt "
    _file.close();
    Serial.println("not fmt");
    return;
  }

  // header.size==16 for PCM
  // audioFormat==1 is PCM == Linear quantization; other than 1 indicate some form of compression
  if (header.subChunk1.header.size != 16 || header.subChunk1.audioFormat != 1) {
    _file.close();
    Serial.println("not PCM or not liear qant.");
    return;
  }

  while (_file.available()) {
    if (_file.read((uint8_t *)&(sch), sizeof(sch)) != sizeof(sch)) {
      _file.close();
      Serial.println("foo");
      return;
    }

    sch.id = __REV(sch.id);

    if (sch.id == 0x64617461) {
      // found the data section
      header.subChunk2Header.id = sch.id;
      header.subChunk2Header.size = sch.size;
      break;
    }

    // skip this header section
    _file.seek(_file.position() + sch.size);
    subChunk2Offset += (sizeof(sch) + sch.size);
  }

  if (header.subChunk2Header.id != 0x64617461) { // "data"
    // no data section found
    _file.close();
    Serial.println("not data");
    Serial.print("actual value");Serial.println(header.subChunk2Header.id,HEX);
    return;
  }

  _dataOffset = sizeof(struct WaveFileHeader) + subChunk2Offset;
  _file.close();

  _headerRead = true;

  _channels = header.subChunk1.numChannels;
  _sampleRate = header.subChunk1.sampleRate;
  _bitsPerSample = header.subChunk1.bitsPerSample;
  _blockAlign = header.subChunk1.blockAlign;
  _frames = header.subChunk2Header.size / _blockAlign;

  _isValid = true;
}

int SDWaveFile::initWrite(int bitsPerSample, long sampleRate){
  Serial.println("SDWaveFile::initWrite");
  _dataSize = 0;
  // Try to create temporary file. If it fails exit with failure
  int max_tmp_tries = 5;
  do{
    _tmp_filename = "/tmp_" + String(millis() % 1000) + String(max_tmp_tries); // generate tmp file name
    --max_tmp_tries;
    Serial.print("Check file \"");Serial.print(_tmp_filename);Serial.println("\"");
  }while(SD.exists(_tmp_filename) && max_tmp_tries > 0);
  if(SD.exists(_tmp_filename) && max_tmp_tries <= 0){ // if file exists and all tries exceeded
    Serial.println("Tries exceeded, return with error");
    return 0; // failed
  }
  Serial.println("Open file");
  _file = SD.open(_tmp_filename, FILE_WRITE);
  Serial.print("after open _file = "); Serial.println(_file);
  if(_file == false){
    return 0; // ERR
  }

  _channels = 2;
  _sampleRate = sampleRate;
  _bitsPerSample = bitsPerSample;
  return 1; // OK
}

int SDWaveFile::writeData(void* data, size_t bytesToWrite, bool finished){
  uint32_t written = 0;
  //Serial.print("write to pos "); Serial.println(_file.position());
  // TODO __REV samples
  /*
  if(_bitsPerSample == 8){
    for(int i = 0; i < bytesToWrite; ++i){
      written += _file.write(__REV(((uint8_t *)data)[i]));
    }
  }else if(_bitsPerSample == 16){
    uint8_t retype[2];
    for(int i = 0; i < bytesToWrite; i+=2){
      retype[0] = __REV(((uint8_t *)data)[i+1]);
      retype[1] = __REV(((uint8_t *)data)[i]);
      written += _file.write(retype,2);
    }
  }else{
    written = _file.write((uint8_t *)data,bytesToWrite);
  }
  */
  /*
  Serial.print("Writing ");  Serial.println(bytesToWrite);
  for(int i = 0; i < bytesToWrite; ++i){
    Serial.print(((uint8_t *)data)[i]);Serial.print(" ");
  }
  Serial.println("");
  */

  written = _file.write((uint8_t *)data,bytesToWrite);
  _dataSize += written;
  if(bytesToWrite != written){
    Serial.println("ERROR: bytesToWrite != written");
    return 0; // ERR
  }
  //Serial.print("Written ");Serial.print(_dataSize);Serial.println(" Bytes so far..");
  if(finished){ // write header and move data from tmp file
    return finishWavWrite(_dataSize);
  }
  return 1; // OK
}

int SDWaveFile::finishWavWrite(uint32_t numOfBytes){
  Serial.println("SDWaveFile::finishWavWrite");
  struct WaveFileHeader header;
  header.chunkId = __REV(0x52494646); // "RIFF"
  //header.chunkId = 0x52494646; // "RIFF"
  header.chunkSize = 36 + numOfBytes; // 36 + SubChunk2Size
  header.format = __REV(0x57415645); // "WAVE"
  //header.format = 0x57415645; // "WAVE"

  header.subChunk1.header.id = __REV(0x666d7420); // "fmt "
  //header.subChunk1.header.id = 0x666d7420; // "fmt "
  header.subChunk1.header.size = 16; // PCM
  header.subChunk1.audioFormat = 1; // PCM Linear quantization; other than 1 indicate some form of compression
  header.subChunk1.numChannels = _channels;
  header.subChunk1.sampleRate = _sampleRate;
  header.subChunk1.byteRate = _sampleRate * _channels * _bitsPerSample/8;
  header.subChunk1.blockAlign = _channels * _bitsPerSample/8;
  header.subChunk1.bitsPerSample = _bitsPerSample;

  header.subChunk2Header.id = __REV(0x64617461); // "data"
  //header.subChunk2Header.id = 0x64617461; // "data"
  header.subChunk2Header.size = numOfBytes;

  _file.close(); // close tmp file with only data
  // Remove previous file and write header
  SD.remove(_filename);
  _file = SD.open(_filename, FILE_WRITE);
  _file.write((uint8_t *)&header, sizeof(header)/sizeof(uint8_t));
  _file.close();
  Serial.println("Header written");

  // Only one file can be opened - create buffer for transfer
  int buffer_bytes = 1024;
  uint8_t buffer[buffer_bytes];
  uint32_t bytesMoved = 0; // At the end this should be equal to numOfBytes
  uint32_t read, written; // How many Bytes were actually read and written
  uint32_t offset = 0; // Offset for reading tmp file

  // only for debug
  /*
  _file = SD.open(_tmp_filename, FILE_READ);
  Serial.print("tmp file size = "); Serial.println(_file.size());
  _file.close();
*/

  do{
    // load buffer from tmp file
    _file = SD.open(_tmp_filename, FILE_READ);
    _file.seek(offset);
    //Serial.print("read from "); Serial.println(_file.position());
    read = _file.read(buffer, buffer_bytes);
    //Serial.print("read ");Serial.println(read);
    _file.close();
/*
    Serial.print("Moving buffer; read=");  Serial.println(read);
    for(int i = 0; i < read; ++i){
      Serial.print(buffer[i]);Serial.print(" ");
    }
    Serial.println("");
*/
    //move buffer to actual file
    _file = SD.open(_filename, FILE_WRITE); // Should open in append mode, but does not!!!
    _file.seek(offset+44); // Because append mode doesn't work move to (offset + header_length)
    //Serial.print("write to "); Serial.println(_file.position());
    written = _file.write(buffer, read);
    //Serial.print("written "); Serial.println(written);
    _file.close();
    if(read != written){
      SD.remove(_tmp_filename); // remove tmp file
      Serial.println("read and written mismatch! Return with error");
      return 0; // ERROR
    }
    bytesMoved += written;
    offset += read;
    //Serial.print("bytesMoved "); Serial.print(bytesMoved); Serial.print(" of total numOfBytes "); Serial.println(numOfBytes);
  }while(bytesMoved < numOfBytes);

  SD.remove(_tmp_filename); // remove tmp file
  Serial.println("finishWavWrite() Done");
  return 1; // OK
}

// TODO void purgeTmp();
void SDWaveFile::purgeTmp(){
  File root = SD.open("/");
  char filename_buff[32];
  char sub_name[6];
  while (true) {
    File entry =  root.openNextFile();
    if (! entry) {
      // no more files
      break;
    }
    int len = strlen(entry.name());
    /*
    Serial.println("");
    Serial.print("Check file "); Serial.println(entry.name());
    Serial.print("Filename len = "); Serial.println(len);
    */
    memset(filename_buff,0,32);
    memcpy(filename_buff, entry.name(),(len > 32) ? 32 : len);
    sub_name[5] = 0;
    memcpy(sub_name, entry.name(), 5);
    /*
    Serial.print("filename_buff "); Serial.println(String(filename_buff));
    Serial.print("sub_name "); Serial.println(sub_name);
    */
    entry.close();
    if(strcmp(sub_name, "/TMP_") == 0){ // If filename starts with "tmp_" delete it
      //Serial.print("Deleting tmp file "); Serial.println(entry.name());
      SD.remove(String(filename_buff));
    }
  }
}
