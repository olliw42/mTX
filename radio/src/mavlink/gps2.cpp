//*******************************************************
// GPS Library
// Copyright (c) OlliW, OlliW42, www.olliw.eu
// Licence GPL2 or (at your option) GPL3
// https://www.gnu.org/licenses/gpl-2.0.en.html
// https://www.gnu.org/licenses/gpl-3.0.en.html
//*******************************************************

#include "opentx.h"


#define GPS_TIMEOUT_10MS   500 // 5 sec

uint32_t gps_msg_received_tlast = 0;

gpsdata2_t gpsData2 = {0};

void gps_parse_nextchar(char c);
void gps_reset(void);
void gps_do(void);


//-------------------------------------------------------
// OpenTx Gps Interface
//-------------------------------------------------------
// this is the interface used by OpenTx, so we recreate it here to fool OpenTx
// gpsSendByte(), gpsGetByte() are defined in gps_driver.cpp
// gps.cpp

gpsdata_t gpsData = {0};

char hex(uint8_t b)
{
  return b > 9 ? b + 'A' - 10 : b + '0';
}

// used by cli
void gpsSendFrame(const char * frame)
{
  // send given frame, add checksum and CRLF
  uint8_t parity = 0;
  TRACE_NOCRLF("gps> %s", frame);
  while (*frame) {
    if (*frame != '$') parity ^= *frame;
    gpsSendByte(*frame);
    ++frame;
  }
  gpsSendByte('*');
  gpsSendByte(hex(parity >> 4));
  gpsSendByte(hex(parity & 0x0F));
  gpsSendByte('\r');
  gpsSendByte('\n');
  TRACE("*%02x", parity);
}

// called in main
void gpsWakeup()
{
  bool enabled = false;
#if defined(SERIAL_GPS)
#if defined(AUX_SERIAL)
  if (auxSerialMode == UART_MODE_GPS) enabled = true;
#endif
#if defined(AUX2_SERIAL)
  if (aux2SerialMode == UART_MODE_GPS) enabled = true;
#endif
#endif
  if (!enabled) { gps_msg_received_tlast = 0; return; }

  uint8_t byte;
  while (gpsGetByte(&byte)) {
    gps_parse_nextchar(byte); // gpsNewData(byte);
  }
  gps_do();
  uint32_t tnow = get_tmr10ms();
  if ((tnow - gps_msg_received_tlast) > GPS_TIMEOUT_10MS) gpsClear();
}

void gpsClear(void)
{
  gps_msg_received_tlast = 0;
  memset(&gpsData, 0, sizeof(gpsdata_t));
  memset(&gpsData2, 0, sizeof(gpsdata2_t));
  gps_reset();
}


//-------------------------------------------------------
// New GPS Driver
//-------------------------------------------------------

typedef enum {
  GPS_PARSER_STATE_IDLE = 0,

  GPS_PARSER_STATE_NMEA_RECEIVING,
  GPS_PARSER_STATE_NMEA_RECEIVEERROR,
  GPS_PARSER_STATE_NMEA_ERROR,
  GPS_PARSER_STATE_NMEA_UPDATED,

  GPS_PARSER_STATE_UBX_RECEIVING,
  GPS_PARSER_STATE_UBX_RECEIVEERROR,
  GPS_PARSER_STATE_UBX_ERROR,
  GPS_PARSER_STATE_UBX_UPDATED
} GPSPARSERSTATEENUM;


typedef enum {
  GPS_INIT_STATE_IDLE = 0,
  GPS_INIT_STATE_START,
  // disable NMEA messages
  GPS_INIT_STATE_DISABLE_TXT,
  GPS_INIT_STATE_DISABLE_GSA,
  GPS_INIT_STATE_DISABLE_GSV,
  GPS_INIT_STATE_DISABLE_GLL,
  GPS_INIT_STATE_DISABLE_VTG,
  GPS_INIT_STATE_DISABLE_RMC,
  GPS_INIT_STATE_DISABLE_GGA,
  GPS_INIT_STATE_DISABLE_NMEA,
  // do UBX stuff
  GPS_INIT_STATE_UBX_ENABLE_DOP1,
  GPS_INIT_STATE_UBX_ENABLE_SOL1,
  GPS_INIT_STATE_UBX_ENABLE_PVT1,
  GPS_INIT_STATE_UBX_RATE,
} GPSINITSTATEENUM;


typedef struct {
  uint8_t StartSign; // is not really needed, but keep it
  uint8_t TalkerID;
  char TalkerIDstr[2];
  uint8_t SentenceID;
  char SentenceIDstr[3];
  uint8_t Checksum1;
  uint8_t Checksum2;

  uint8_t state;
  uint8_t cnt; // general purpose counter
  uint8_t datafield_cnt;
  uint8_t* datafield[32];
  uint8_t crc;
} tNmeaMsg;


typedef struct {
  uint8_t StartSign1; // is not really needed, but keep it
  uint8_t StartSign2; // is not really needed, but keep it
  uint8_t ClassID;
  uint8_t MessageID;
  uint16_t Length;
  uint8_t CheckSumA;
  uint8_t CheckSumB;

  uint8_t state;
  uint16_t payload_cnt;
  uint8_t crcA;
  uint8_t crcB;
} tUbxMsg;


typedef struct {
  uint8_t state;
  uint8_t buf[256]; // this contains the parsed data
  union {           // this holds msg type specific status data
    tNmeaMsg nmea_msg;
    tUbxMsg ubx_msg;
  };

  uint8_t initialized;
  uint8_t init_state;
  uint32_t init_timefornextstep_10ms;
} tGps;

tGps _gps = {0};


#define GPS_PACKED(__Declaration__) __Declaration__ __attribute__((packed))

#include "gps-nmea.h"
#include "gps-ubx.h"


void gps_hal_putc(char c)
{
  gpsSendByte(c);
};


void gps_reset(void)
{
  memset(&_gps, 0, sizeof(tGps));
}


void gps_parse_nextchar(char c)
{
  if (_gps.state == GPS_PARSER_STATE_IDLE) {
      ubx_parser_nextchar(&_gps, c);
      nmea_parser_nextchar(&_gps, c);
  } else
  if (_gps.state == GPS_PARSER_STATE_UBX_RECEIVING) {
      ubx_parser_nextchar(&_gps, c);
  } else
  if (_gps.state == GPS_PARSER_STATE_NMEA_RECEIVING) {
      nmea_parser_nextchar(&_gps, c);
  }

  if ((_gps.state != GPS_PARSER_STATE_NMEA_RECEIVING) && (_gps.state != GPS_PARSER_STATE_UBX_RECEIVING)) {
      _gps.state = GPS_PARSER_STATE_IDLE;
  }
}


// is called when a new valid sentence is received
uint8_t nmea_digestmsg(tGps* gps)
{
  if (gps->nmea_msg.TalkerID == NMEA_TALKER_UNDEFIND) return 0;

  gpsData.packetCount++;

  if (!_gps.initialized && !_gps.init_state) {
      if (gps->nmea_msg.SentenceID == NMEA_SENTENCE_GGA) { // wait for all initial TXT sentences having passed
          _gps.init_state = GPS_INIT_STATE_START;
      }
  }

  return 1;
}


uint8_t ubx_digestmsg(tGps* gps)
{
  gpsData.packetCount++;

  if (gps->ubx_msg.ClassID != UBX_CLASS_NAV) return 0;

  if (gps->ubx_msg.MessageID == UBX_NAV_PVT) {
      if (gps->ubx_msg.Length != UBLOX_UBXNAVPVT_LEN) return 0; // should never happen, but play it safe

      gps_msg_received_tlast = get_tmr10ms(); // mark it as received, and reset timeout

      tUbxNavPvtPacket pvt;
      memcpy(&pvt, gps->buf, sizeof(tUbxNavPvtPacket));

      // give OpenTx what it wants
      int32_t latitude = pvt.lat / 10; // OpenTx wants it in 1e6
      int32_t longitude = pvt.lon / 10; // OpenTx wants it in 1e6
      int32_t altitude = pvt.hMSL / 1000; // OpenTx wants it in m // NOTE: the lua description is incorrect!

      //gpsData.fix = (pvt.fixType >= UBX_NAV_PVT_FIXTYPE_2DFIX && pvt.fixType <= UBX_NAV_PVT_FIXTYPE_3DFIX) ? 1 : 0;
      gpsData.fix = (pvt.flags & UBX_NAV_PVT_FLAGS_GNSSFIXOK) ? 1 : 0;
      gpsData.numSat = pvt.numSV;

      if (gpsData.fix) {
          __disable_irq(); // do the atomic update of lat/lon
          gpsData.latitude = latitude;
          gpsData.longitude = longitude;
          gpsData.altitude = altitude;
          __enable_irq();
      }

      gpsData.speed = pvt.gSpeed / 10; // OpenTx wants it in cm/s
      gpsData.groundCourse = pvt.headMot / 10000; // OpenTx wants it in ddeg

      // that's what we want in addition
      gpsData2.fix = 0;
      if (pvt.flags & UBX_NAV_PVT_FLAGS_GNSSFIXOK) { // flags bitfield says that fix is OK
          switch (pvt.fixType) {
          case UBX_NAV_PVT_FIXTYPE_2DFIX:
              gpsData2.fix = 2;
              break;
          case UBX_NAV_PVT_FIXTYPE_3DFIX:
          case UBX_NAV_PVT_FIXTYPE_GNSSDEADRECKONING:
              gpsData2.fix = 3;
              break;
          }
      }
      gpsData2.has_pos_fix = ((gpsData2.fix >= 3) && (gpsData.numSat >= 8) && (gpsData.hdop < 150));

      gpsData2.lat_1e7 = pvt.lat;
      gpsData2.lon_1e7 = pvt.lon;
      gpsData2.alt_mm = pvt.hMSL;
      gpsData2.speed_mms = pvt.gSpeed;
      gpsData2.cog_cdeg = pvt.headMot / 1000;

      gpsData2.velN_mms = pvt.velN;
      gpsData2.velE_mms = pvt.velE;
      gpsData2.velD_mms = pvt.velD;

#if defined(RTCLOCK) && 0
      // set RTC clock if needed
      if (g_eeGeneral.adjustRTC && gpsData.fix) {
          uint8_t year = pvt.year; //TODO: check if all these are correct !!!!!
          uint8_t mon = pvt.month;
          uint8_t day = pvt.day;
          uint8_t sec = pvt.sec;
          uint8_t min = pvt.min;
          uint8_t hour = pvt.hour;
          rtcAdjust(year+2000, mon, day, hour, min, sec);
      }
#endif
      return 1;
  }

  if (gps->ubx_msg.MessageID == UBX_NAV_DOP) {
      if (gps->ubx_msg.Length != UBLOX_UBXNAVDOP_LEN) return 0; // should never happen, but play it safe

      tUbxNavDopPacket dop;
      memcpy(&dop, gps->buf, sizeof(tUbxNavDopPacket));

      // give OpenTx what it wants
      gpsData.hdop = dop.hDOP;

      // that's what we want in addition
      gpsData2.vdop = dop.vDOP;

      return 1;
  }

  return 0;
};


void gps_do(void)
{
  if (_gps.initialized) return;

  uint32_t tnow = get_tmr10ms();
  gps_msg_received_tlast = tnow; // we mark it as received, this prevents reset

  if (_gps.init_state == GPS_INIT_STATE_IDLE) return;

  if (_gps.init_state == GPS_INIT_STATE_START) {
      _gps.state = GPS_PARSER_STATE_IDLE; // reset parser
      _gps.init_timefornextstep_10ms = tnow + 25; // wait a bit, to not interrupt the current NMEA stream
      _gps.init_state = GPS_INIT_STATE_DISABLE_NMEA;
  }

  if (tnow < _gps.init_timefornextstep_10ms) return;

  switch (_gps.init_state) {
  case GPS_INIT_STATE_DISABLE_NMEA:
      nmea_send_pubx_cnfg();
      _gps.init_timefornextstep_10ms = tnow + 50;
      _gps.init_state = GPS_INIT_STATE_UBX_ENABLE_DOP1;
      break;
  case GPS_INIT_STATE_UBX_ENABLE_DOP1:
      ubx_send_cnfmsg_dop1();
      _gps.init_timefornextstep_10ms = tnow + 50;
      _gps.init_state = GPS_INIT_STATE_UBX_ENABLE_PVT1;
      break;
  case GPS_INIT_STATE_UBX_ENABLE_PVT1:
      ubx_send_cnfmsg_pvt1();
      _gps.init_timefornextstep_10ms = tnow + 50;
      _gps.init_state = GPS_INIT_STATE_UBX_RATE;
      break;
  case GPS_INIT_STATE_UBX_RATE:
      // PVT: 8 + 92 = 100 bytes
      // DOP: 8 + 18 = 26 bytes
      // 5 Hz with PVT & DOP = 5 x 126 bytes/s = 630 bytes/2 = 66% of bandwidth @ 9600bps
      switch (g_model.mavlinkSendPosition) {
      case 1: ubx_send_cnfrate(1000); break; // 1 Hz
      case 2: ubx_send_cnfrate(500); break; // 2 Hz
      case 3: ubx_send_cnfrate(333); break; // 3 Hz
      case 4: ubx_send_cnfrate(250); break; // 4 Hz
      case 5: ubx_send_cnfrate(200); break; // 5 Hz
      default: ubx_send_cnfrate(1000); break; // 1 Hz
      }
      _gps.initialized = 1;
      break;
  }
};


