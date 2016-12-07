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

// based on: http://soundfile.sapp.org/doc/WaveFormat/
struct WaveFileHeader {
  uint32_t chunkId;
  uint32_t chunkSize;
  uint32_t format;
  struct {
    uint32_t id;
    uint32_t size;
    uint16_t audioFormat;
    uint16_t numChannels;
    uint32_t sampleRate;
    uint32_t byteRate;
    uint16_t blockAlign;
    uint16_t bitsPerSample;
  } subChunk1;
  struct {
    uint32_t id;
    uint32_t size;
  } subChunk2;
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
  _frames(-1)
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

  if (position >= sizeof(struct WaveFileHeader)) {
    position -= sizeof(struct WaveFileHeader);
  }

  return (position) / (_blockAlign * _sampleRate);
}

int SDWaveFile::cue(long time)
{
  if (time < 0) {
    return 1;
  }

  long offset = (time * _blockAlign) - sizeof(struct WaveFileHeader);

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
  uint32_t position = _file.position();
  int read = _file.read(buffer, size);

  if (position == 0) {
    // replace the header with 0's
    memset(buffer, 0x00, sizeof(struct WaveFileHeader));
  }

  if (read) {
    samplesRead(buffer, read);
  }

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

  if (_headerRead) {
    return;
  }

  _file = SD.open(_filename);

  if (!_file) {
    return;
  }

  uint32_t fileSize = _file.size();

  if (fileSize < sizeof(struct WaveFileHeader)) {
    _file.close();
    return;
  }

  struct WaveFileHeader header;

  if (_file.read(&header, sizeof(header)) != sizeof(header)) {
    _file.close();
    return;
  }
  _file.close();

  _headerRead = true;

  header.chunkId = __REV(header.chunkId);
  header.format = __REV(header.format);
  header.subChunk1.id = __REV(header.subChunk1.id);
  header.subChunk2.id = __REV(header.subChunk2.id);

  if (header.chunkId != 0x52494646) { // "RIFF"
    return;
  }

  if ((fileSize - 8) != header.chunkSize) {
    return;
  }

  if (header.format != 0x57415645) { // "WAVE"
    return;
  }

  if (header.subChunk1.id != 0x666d7420) { // "fmt "
    return;
  }

  if (header.subChunk1.size != 16 || header.subChunk1.audioFormat != 1) {
    // not PCM
    return;
  }

  if (header.subChunk2.id != 0x64617461) { // "data"
    return;
  }

  _channels = header.subChunk1.numChannels;
  _sampleRate = header.subChunk1.sampleRate;
  _bitsPerSample = header.subChunk1.bitsPerSample;
  _blockAlign = header.subChunk1.blockAlign;
  _frames = header.subChunk2.size / _blockAlign;

  _isValid = true;
}
