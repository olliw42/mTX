/*
 * Copyright (C) OpenTX
 *
 * Based on code named
 *   th9x - http://code.google.com/p/th9x
 *   er9x - http://code.google.com/p/er9x
 *   gruvin9x - http://code.google.com/p/gruvin9x
 *
 * License GPLv2: http://www.gnu.org/licenses/gpl-2.0.html
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef _TOUCH_H_
#define _TOUCH_H_

enum TouchEvent
{
  TE_NONE,
  TE_DOWN,
  TE_UP,
  TE_SLIDE,
  TE_SLIDE_END
};

//OW
enum TouchEventExtended
{
  TE_EXT_NONE,
  TE_TAP,
  TE_WIPE_UP,
  TE_WIPE_DOWN,
  TE_WIPE_LEFT,
  TE_WIPE_RIGHT,
};
//OWEND

struct TouchState
{
  unsigned char event;
  short x;
  short y;
  short startX;
  short startY;
  short deltaX;
  short deltaY;
  short lastDeltaX;
  short lastDeltaY;
//OW
  unsigned char extEvent;
  short _deltaX;
  short _deltaY;
  tmr10ms_t _last;
//OWEND
};

constexpr uint8_t SLIDE_RANGE = 6;

extern TouchState touchState;

#endif // _TOUCH_H_