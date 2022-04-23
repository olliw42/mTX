/*
 * Copyright (C) OpenTX
 *
 * Based on code named
 *   cleanflight - https://github.com/cleanflight
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

#ifndef _GPS_H_
#define _GPS_H_

#include <inttypes.h>

struct gpsdata_t
{
  int32_t longitude;              // degrees * 1.000.000
  int32_t latitude;               // degrees * 1.000.000
  uint8_t fix;
  uint8_t numSat;
  uint32_t packetCount;
  uint32_t errorCount;
  uint16_t altitude;              // altitude in 0.1m //OW NOTE: incorrect, is in m! lua description also incorrect!
  uint16_t speed;                 // speed in 0.1m/s
  uint16_t groundCourse;          // degrees * 10
  uint16_t hdop;
};

extern gpsdata_t gpsData;
void gpsWakeup();

void gpsSendFrame(const char * frame);

//OW
// extension to OpenTx's gpsdata_t structure
typedef struct {
  uint8_t fix;          // 0: no fix, 2: 2D fix, 3: 3D fix
  uint16_t vdop;        // scaling 1e-2, 100 = 1.0
  int32_t lat_1e7;      // in 1e7 deg, 1e7 = 1°
  int32_t lon_1e7;      // in 1e7 deg, 1e7 = 1°
  int32_t alt_mm;       // in mm, 1000 = 1 m, is above sea level in m
  int32_t speed_mms;    // in mm/s, 1000 = 1 m/s
  uint16_t cog_cdeg;    // in cdeg, 100 = 1°
  int32_t velN_mms;     // in mm/s
  int32_t velE_mms;     // in mm/s
  int32_t velD_mms;     // in mm/s
  // auxiliary
  bool has_pos_fix;
} gpsdata2_t;

extern tmr10ms_t gps_msg_received_tlast;
extern gpsdata2_t gpsData2;
void gpsClear(void);
//OWEND

#endif // _GPS_H_
