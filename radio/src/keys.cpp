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

#include "opentx.h"

#define KEY_LONG_DELAY              32  // long key press minimum duration (x10ms), must be less than KEY_REPEAT_DELAY
#define KEY_REPEAT_DELAY            40  // press longer than this enables repeat (but does not fire it yet)
#define KEY_REPEAT_TRIGGER          48  // repeat trigger, used in combination with m_state to produce decreasing times between repeat events
#define KEY_REPEAT_PAUSE_DELAY      64

#ifdef SIMU
  #define FILTERBITS                1   // defines how many bits are used for debounce
#else
  #define FILTERBITS                4   // defines how many bits are used for debounce
#endif

#if !defined(HARDWARE_TRIMS)
  #define  ROTARY_FILTERBITS        1   // when make the rotary as the key, reduce the hold time
#endif

#define KSTATE_OFF                  0
#define KSTATE_RPTDELAY             95
#define KSTATE_START                97
#define KSTATE_PAUSE                98
#define KSTATE_KILLED               99

#if !defined(HARDWARE_TRIMS)
uint8_t g_trimEditMode = EDIT_TRIM_DISABLED;
#endif

event_t s_evt;
struct InactivityData inactivity = {0};
Key keys[NUM_KEYS];
//OW
uint16_t s_evt_lockmask = 0;
bool s_evt_locktmo = false;
tmr10ms_t s_evt_lock_tlast = 0;
#define EVENT_LOCKMASK_ALLOWED   ((1<<KEY_ENTER)|(1<<KEY_MODEL)|(1<<KEY_EXIT)|(1<<KEY_TELEM)|(1<<KEY_RADIO))

bool eventIsLocked(event_t evt)
{
  //for testing s_evt_lockmask = (1<<2) | (1<<3) | (1<<4) | (1<<5) | (1<<6);
  uint16_t key = EVT_KEY_MASK(evt);
  if (key >= TRM_BASE) return false;
  if (s_evt_lockmask & (1<<key)) return true;
  return false;
}

void lockKeys(uint16_t mask)
{
  s_evt_lockmask = (mask & EVENT_LOCKMASK_ALLOWED);
  s_evt_lock_tlast = get_tmr10ms();
}

void unlockKeys(void)
{
  s_evt_lockmask = 0;
}

void checkEventLockTmo(void)
{
  if (!s_evt_lockmask) return;
  tmr10ms_t now = get_tmr10ms();
  if ((now - s_evt_lock_tlast) > 50) s_evt_lockmask = 0;  //500 ms
}
//OWEND

event_t getEvent(bool trim)
{
  event_t evt = s_evt;
//OW
  if (eventIsLocked(evt)) return 0;
//OWEND
  int8_t k = EVT_KEY_MASK(s_evt) - TRM_BASE;
  bool trim_evt = (k>=0 && k<TRM_LAST-TRM_BASE+1);

  if (trim == trim_evt) {
    s_evt = 0;
    return evt;
  }
  else {
    return 0;
  }
}

void Key::input(bool val)
{
  // store new value in the bits that hold the key state history (used for debounce)
  uint8_t t_vals = m_vals ;
#if !defined(HARDWARE_TRIMS) && !defined(BOOT)
  uint8_t filterBits = 0;
#endif
  t_vals <<= 1 ;
  if (val) t_vals |= 1;
  m_vals = t_vals ;

  m_cnt++;

  if (m_state && m_vals == 0) {
    // key is released
    if (m_state != KSTATE_KILLED) {
      // TRACE("key %d BREAK", key());
      putEvent(EVT_KEY_BREAK(key()));
    }
    m_state = KSTATE_OFF;
    m_cnt = 0;
    return;
  }

  switch (m_state) {
    case KSTATE_OFF:
#if !defined(HARDWARE_TRIMS) && !defined(BOOT)
      if (g_trimEditMode != EDIT_TRIM_DISABLED) {
        filterBits = ROTARY_FILTERBITS;
      }
      else {
        filterBits = FILTERBITS;
      }
      if (m_vals == ((1<<filterBits)-1)) {
#else
      if (m_vals == ((1<<FILTERBITS)-1)) {
#endif
        m_state = KSTATE_START;
        m_cnt = 0;
      }
      break;

    case KSTATE_START:
      // TRACE("key %d FIRST", key());
      putEvent(EVT_KEY_FIRST(key()));
      inactivity.counter = 0;
      m_state = KSTATE_RPTDELAY;
      m_cnt = 0;
      break;

    case KSTATE_RPTDELAY: // gruvin: delay state before first key repeat
#if !defined(HARDWARE_TRIMS)
      if (g_trimEditMode != EDIT_TRIM_DISABLED) {
  #if defined(SIMU)
        if (m_cnt == KEY_LONG_DELAY) {
          m_state = 16;
          m_cnt = 0;
        }
        break;
  #else
        if (m_cnt == 6) {
          m_state = 2;
          m_cnt = 0;
        }
        break;
  #endif
      }
      else {
        if (m_cnt == KEY_LONG_DELAY) {
          // generate long key press
          //TRACE("key %d LONG", key());
          putEvent(EVT_KEY_LONG(key()));
        }
        if (m_cnt == KEY_REPEAT_DELAY) {
          m_state = 16;
          m_cnt = 0;
        }
        break;
      }
#else
      if (m_cnt == KEY_LONG_DELAY) {
        // generate long key press
        // TRACE("key %d LONG", key());
        putEvent(EVT_KEY_LONG(key()));
      }
      if (m_cnt == KEY_REPEAT_DELAY) {
        m_state = 16;
        m_cnt = 0;
      }
      break;
#endif
    case 16:
    case 8:
    case 4:
    case 2:
      if (m_cnt >= KEY_REPEAT_TRIGGER) { //3 6 12 24 48 pulses in every 480ms
        m_state >>= 1;
        m_cnt = 0;
      }
      // no break
    case 1:
      if ((m_cnt & (m_state-1)) == 0) {
        // this produces repeat events that at first repeat slowly and then increase in speed
        // TRACE("key %d REPEAT", key());
        if (!IS_SHIFT_KEY(key()))
          putEvent(EVT_KEY_REPT(key()));
      }
      break;

    case KSTATE_PAUSE: //pause repeat events
      if (m_cnt >= KEY_REPEAT_PAUSE_DELAY) {
        m_state = 8;
        m_cnt = 0;
      }
      break;

    case KSTATE_KILLED: //killed
      break;
  }
}

void Key::pauseEvents()
{
  m_state = KSTATE_PAUSE;
  m_cnt = 0;
}

void Key::killEvents()
{
  // TRACE("key %d killed", key());
  m_state = KSTATE_KILLED;
}


uint8_t Key::key() const
{
  return (this - keys);
}

// Introduce a slight delay in the key repeat sequence
void pauseEvents(event_t event)
{
  event = EVT_KEY_MASK(event);
  if (event < (int)DIM(keys)) keys[event].pauseEvents();
}

// Disables any further event generation (BREAK and REPEAT) for this key, until the key is released
void killEvents(event_t event)
{
  event = EVT_KEY_MASK(event);
  if (event < (int)DIM(keys)) {
    keys[event].killEvents();
  }
}

void killAllEvents()
{
  for (uint8_t key = 0; key < DIM(keys); key++) {
    keys[key].killEvents();
  }
}

bool waitKeysReleased()
{
#if defined(PCBSKY9X)
  RTOS_WAIT_MS(200); // 200ms
#endif

  // loop until all keys are up
#if !defined(BOOT)
  tmr10ms_t start = get_tmr10ms();
#endif

  while (keyDown()) {
    WDG_RESET();

#if !defined(BOOT)
    if ((get_tmr10ms() - start) >= 300) {  // wait no more than 3 seconds
      //timeout expired, at least one key stuck
      return false;
    }
#endif
  }

  memclear(keys, sizeof(keys));
  putEvent(0);
  return true;
}
