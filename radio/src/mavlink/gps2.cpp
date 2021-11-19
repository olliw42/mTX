/*
 * The MAVLink for OpenTx project
 * Copyright (c) www.olliw.eu, OlliW, OlliW42
 */

#include "opentx.h"


tmr10ms_t gps_msg_received_tlast = 0;

gpsdata2_t gpsData2 = {0};

void gps_parse_nextchar(char c);
void gps_reset(void);


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
  uint8_t byte;
  while (gpsGetByte(&byte)) {
    gps_parse_nextchar(byte); // gpsNewData(byte);
  }
  tmr10ms_t tnow = get_tmr10ms();
  if ((tnow - gps_msg_received_tlast) > 500) gpsClear(); // timeout of 5 sec
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

#define GPS_LOCATION_VALID_QUALITY  1
#define GPS_LOCATION_VALID_HDOP     150 //1.5
#define GPS_LOCATION_VALID_SAT      8

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
  uint8_t init_step;
  uint16_t init_mask;
  tmr10ms_t init_timefornextstep_10ms;
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
  uint8_t res = 0;

  if (gps->nmea_msg.SentenceID == NMEA_SENTENCE_GGA) {
      tNmeaGGA sentence;
      if (!nmea_parse_GGA(&sentence, &(gps->nmea_msg) )) return 0;

      int32_t latitude = sentence.lat_1e7 / 10; // OpenTx wants it in 1e6
      int32_t longitude = sentence.lon_1e7 / 10; // OpenTx wants it in 1e6
      int32_t altitude = sentence.alt_cm / 100; // OpenTx wants it in m // NOTE: the lua description is incorrect!

      // this is GPGGA, so we do the update
      gpsData.fix = (sentence.fix) ? 1 : 0;
      gpsData.numSat = sentence.sat;
      gpsData.hdop = sentence.hdop;
      if (gpsData.fix) {
        __disable_irq(); // do the atomic update of lat/lon
        gpsData.latitude = latitude;
        gpsData.longitude = longitude;
        gpsData.altitude = altitude;
        __enable_irq();

        gpsData2.lat_1e7 = sentence.lat_1e7;
        gpsData2.lon_1e7 = sentence.lon_1e7;
        gpsData2.alt_cm = sentence.alt_cm;
      }

      gps_msg_received_tlast = get_tmr10ms();
      res = 1;
  }
  if (gps->nmea_msg.SentenceID == NMEA_SENTENCE_RMC) {
      tNmeaRMC sentence;
      if (!nmea_parse_RMC(&sentence, &(gps->nmea_msg) )) return 0;

      gpsData.speed = sentence.speed_cms; // OpenTx wants it in cms
      gpsData.groundCourse = sentence.cog_cdeg / 10; // OpenTx wants it in ddeg

      res = 1;
#if defined(RTCLOCK) && 0
      // set RTC clock if needed
      if (g_eeGeneral.adjustRTC && sentence.fix) {
          div_t qr = div(sentence.date, 100);
          uint8_t year = qr.rem;
          qr = div(qr.quot, 100);
          uint8_t mon = qr.rem;
          uint8_t day = qr.quot;
          qr = div(sentence.time, 100);
          uint8_t sec = qr.rem;
          qr = div(qr.quot, 100);
          uint8_t min = qr.rem;
          uint8_t hour = qr.quot;
          rtcAdjust(year+2000, mon, day, hour, min, sec);
      }
#endif
  }

  if (_gps.initialized) return res;

  // turn off frames, and initialize rate (do this only once a second)
  static tmr10ms_t gps_cmd_send_tlast = 0;

  tmr10ms_t tnow = get_tmr10ms();
  if ((tnow - gps_cmd_send_tlast) > 100) {
      switch (gps->nmea_msg.SentenceID) {
      case NMEA_SENTENCE_VTG: nmea_send_pubx_disable("VTG"); gps_cmd_send_tlast = tnow; break;
      case NMEA_SENTENCE_GSA: nmea_send_pubx_disable("GSA"); gps_cmd_send_tlast = tnow; break;
      case NMEA_SENTENCE_GSV: nmea_send_pubx_disable("GSV"); gps_cmd_send_tlast = tnow; break;
      case NMEA_SENTENCE_GLL: nmea_send_pubx_disable("GLL"); gps_cmd_send_tlast = tnow; break;
      case NMEA_SENTENCE_TXT: nmea_send_pubx_disable("TXT"); gps_cmd_send_tlast = tnow; break;
      default:
          if ((tnow - gps_cmd_send_tlast) > 200) {
              //ubx_send_cnfrate(250); // 4 Hz output rate, more is not reasonable at 9600bps
              ubx_send_cnfrate(333); // 4 Hz works, but play it safe and go with 3 Hz
              _gps.initialized = 1;
          }
      }
  }

  return res;
};


uint8_t ubx_digestmsg(tGps* gps)
{
  return 0;
};


