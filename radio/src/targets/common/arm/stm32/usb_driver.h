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

#ifndef OPENTX_USB_DRIVER_H
#define OPENTX_USB_DRIVER_H

#include <stdbool.h>

// USB driver
enum usbMode {
  USB_UNSELECTED_MODE,
  USB_JOYSTICK_MODE,
#if defined(RADIO_FAMILY_TBS)
  USB_AGENT_MODE,
  USB_CHARGING_MODE,
#endif
  USB_MASS_STORAGE_MODE,
#if defined(DEBUG)
  USB_SERIAL_MODE,
  USB_MAX_MODE=USB_SERIAL_MODE
#else
  USB_TELEMETRY_MIRROR_MODE,                  // Todo : increase EEprom storage to allow more mode
//OW comment: I've increase RadioData UsbMode to 3 bits = 8 modes
//  USB_MAX_MODE=USB_TELEMETRY_MIRROR_MODE
  USB_MAVLINK_MODE,
#if !defined(TELEMETRY_MAVLINK_USB_SERIAL)
  USB_MAX_MODE=USB_TELEMETRY_MIRROR_MODE
#else
  USB_MAX_MODE=USB_MAVLINK_MODE
#endif
//OWEND
#endif
};

int usbPlugged();
void usbInit();
void usbStart();
void usbStop();
bool usbStarted();
int getSelectedUsbMode();
void setSelectedUsbMode(int mode);

void usbSerialPutc(uint8_t c);

#if defined(RADIO_FAMILY_TBS)
void agentHandler();
#endif

// Used in view_statistics.cpp
#if defined(DEBUG) && !defined(BOOT)
  extern uint16_t usbWraps;
  extern uint16_t charsWritten;
  extern volatile uint32_t APP_Rx_ptr_in;
  extern volatile uint32_t APP_Rx_ptr_out;
#endif

#endif
