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

#ifndef _AUDIO_OUT_I2S_INCLUDED
#define _AUDIO_OUT_I2S_INCLUDED

#include "AudioOut.h"

class AudioOutI2SClass : public AudioOut
{
public:
  AudioOutI2SClass();
  virtual ~AudioOutI2SClass();

  virtual int canPlay(AudioIn& input);
  virtual int play(AudioIn& input);
  virtual int loop(AudioIn& input);

  virtual int pause();
  virtual int resume();
  virtual int stop();

  virtual int isPlaying();
  virtual int isPaused();

private:
  int startPlayback(AudioIn& input, bool loop);

  void onTransmit();

  static void onI2STransmit();

private:
  AudioIn* _input;
  bool _loop;
  bool _paused;
};

extern AudioOutI2SClass AudioOutI2S;

#endif
