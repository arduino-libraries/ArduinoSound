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

#ifndef _SD_WAVE_FILE_INCLUDED
#define _SD_WAVE_FILE_INCLUDED

#include <Arduino.h>
#include <SD.h>

#include "SoundFile.h"

class SDWaveFile : public SoundFile
{
public:
  SDWaveFile();
  SDWaveFile(const char *filename);
  SDWaveFile(const String& filename);

  virtual ~SDWaveFile();

  operator bool();

  // from AudioIn
  virtual long sampleRate();
  virtual int bitsPerSample();
  virtual int channels();

  // from SoundFile
  virtual long frames();
  virtual long duration();
  virtual long currentTime();

  virtual int cue(long time);

protected:
  virtual int begin();
  virtual int read(void* buffer, size_t size);
  virtual int reset();
  virtual void end();

private:
  void readHeader();

private:
  bool _headerRead;
  bool _isValid;
  bool _isPlaying;

  File _file;
  String _filename;

  long _sampleRate;
  int _bitsPerSample;
  int _channels;
  long _frames;
  int _blockAlign;
};

#endif
