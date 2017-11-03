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

#include <I2S.h>

#include "AudioOutI2S.h"

AudioOutI2SClass::AudioOutI2SClass() :
  _input(NULL),
  _loop(false),
  _paused(false)
{
}

AudioOutI2SClass::~AudioOutI2SClass()
{

}

int AudioOutI2SClass::canPlay(AudioIn& input)
{
  int bitsPerSample = input.bitsPerSample();
  int channels = input.channels();

  if (bitsPerSample != 8 && bitsPerSample != 16 && bitsPerSample != 32) {
    return 0;
  }

  if (channels != 1 && channels != 2) {
    return 0;
  }

  return 1;
}

int AudioOutI2SClass::play(AudioIn& input)
{
  return startPlayback(input, false);
}

int AudioOutI2SClass::loop(AudioIn& input)
{
  return startPlayback(input, true);
}

int AudioOutI2SClass::pause()
{
  if (!isPlaying()) {
    return 0;
  }

  _paused = true;

  return 1;
}

int AudioOutI2SClass::resume()
{
  if (!_paused) {
    return 0;
  }

  _paused = false;

  // play some silence to get things going
  uint8_t silence[512];
  memset(silence, 0x00, sizeof(silence));

  I2S.write(silence, sizeof(silence));
  I2S.write(silence, sizeof(silence));

  return 1;
}

int AudioOutI2SClass::stop()
{
  if (!_input) {
    return 0;    
  }

  endInput(_input);
  _input = NULL;

  I2S.end();

  return 1;
}

int AudioOutI2SClass::isPlaying()
{
  return !_paused && (_input != NULL);
}

int AudioOutI2SClass::isPaused()
{
  return _paused;
}

int AudioOutI2SClass::startPlayback(AudioIn& input, bool loop)
{
  if (_input) {
    stop();
  }

  I2S.onTransmit(AudioOutI2SClass::onI2STransmit);

  if (!I2S.begin(I2S_PHILIPS_MODE, input.sampleRate(), input.bitsPerSample())) {
    return 0;
  }

  if (!beginInput(&input)) {
    I2S.end();
    return 0;
  }

  _input = &input;
  _loop = loop;

  uint8_t silence[512];
  memset(silence, 0x00, sizeof(silence));

  I2S.write(silence, sizeof(silence));
  I2S.write(silence, sizeof(silence));

  return 1;
}

void AudioOutI2SClass::onTransmit()
{
  if (!_input || _paused) {
    return;
  }

  int channels = _input->channels();

  uint8_t data[512];
  size_t length = sizeof(data);

  if (channels == 1) {
    length /= 2;
  }

  int n = readInput(_input, data, length);

  if (n == 0) {
    if (!_loop) {
      // non-looped playback, we are done
      stop();
      return;
    }

    // reset the input
    if (!resetInput(_input)) {
      I2S.end();
      return;
    }

    // read the input (again)
    n = readInput(_input, data, length);
  }

  if (channels == 1) {
    monoToStereo(data, n, _input->bitsPerSample());

    n *= 2;
  }

  I2S.write(data, n);
}

void AudioOutI2SClass::onI2STransmit()
{
  AudioOutI2S.onTransmit();
}

AudioOutI2SClass AudioOutI2S;
