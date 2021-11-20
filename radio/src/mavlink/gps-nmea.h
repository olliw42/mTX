/*
 * The MAVLink for OpenTx project
 * Copyright (c) www.olliw.eu, OlliW, OlliW42
 */

#ifndef GPS_NMEA_H
#define GPS_NMEA_H


void gps_hal_putc(char c);

// returns 1 if NMEA message could be successfully parsed, or if error occurred
uint8_t nmea_digestmsg(tGps* gps);


//-------------------------------------------------------
// NMEA
//-------------------------------------------------------
// this is only a parser of NMEA sentences
// we don't need to parse sentences themselves since we switch of NMEA and enable UBX

#define NMEA_STARTSIGN              '$'
#define NMEA_DATAFIELD_SEPARATOR    ','
#define NMEA_CRC_SEPARATOR          '*'


typedef enum {
  NMEA_TALKER_GN = 0,
  NMEA_TALKER_GP,
  NMEA_TALKER_GL,
  NMEA_TALKER_GA,
  NMEA_TALKER_GB,
  NMEA_TALKER_UNDEFIND
} NMEATALKERENUM;


typedef enum {
  NMEA_SENTENCE_RMC = 0,
  NMEA_SENTENCE_VTG,
  NMEA_SENTENCE_GGA,
  NMEA_SENTENCE_GSA,
  NMEA_SENTENCE_GSV,
  NMEA_SENTENCE_GLL,
  NMEA_SENTENCE_TXT,
  NMEA_SENTENCE_UNDEFINED
} NMEASENTENCEENUM;


typedef enum {
  NMEA_STATE_STX = 0, // does never occur
  NMEA_STATE_DATAFIELD,
  NMEA_STATE_DATAFIELD_SEPERATOR,
  NMEA_STATE_CRC_SEPERATOR,
  NMEA_STATE_CRC1,
  NMEA_STATE_CRC2,
  NMEA_STATE_CR,
  NMEA_STATE_LF
} NMEASTATEENUM;


/*
typedef struct {
  uint8_t StartSign; // is currently not really needed, but keep it
  uint8_t TalkerID;
  uint8_t SentenceID;
  uint8_t Checksum1;
  uint8_t Checksum2;

  uint8_t state;
  uint8_t cnt; // general purpose counter
  uint8_t datafield_cnt;
  uint8_t* datafield[32];
  uint8_t crc;
} tNmeaMsg;
*/


void nmea_send(char* s)
{
uint8_t parity = 0;

  while (*s) {
    if (*s != '$') parity ^= *s;
    gps_hal_putc(*s);
    ++s;
  }
  gps_hal_putc('*');
  gps_hal_putc(hex(parity >> 4));
  gps_hal_putc(hex(parity & 0x0F));
  gps_hal_putc('\r');
  gps_hal_putc('\n');
}


//-------------------------------------------------------
// NMEA parser helpers
//-------------------------------------------------------

uint8_t nmea_parse_talkerid(char* s)
{
  if (s[0] == 'G') {
      if (s[1] == 'N') return NMEA_TALKER_GN;
      if (s[1] == 'P') return NMEA_TALKER_GP;
      if (s[1] == 'L') return NMEA_TALKER_GL;
      if (s[1] == 'A') return NMEA_TALKER_GA;
      if (s[1] == 'B') return NMEA_TALKER_GB;
  }
  return NMEA_TALKER_UNDEFIND;
}


uint8_t nmea_parse_sentenceid(char* data)
{
char* s = data;
uint8_t pos = strlen(s);

  if (pos == 5)
      s += 2;
  else if (pos == 4)
      s += 1;
  else
      return NMEA_SENTENCE_UNDEFINED; // invalid length

  if (!strcmp(s,"GGA")) return NMEA_SENTENCE_GGA;
  if (!strcmp(s,"RMC")) return NMEA_SENTENCE_RMC;
  if (!strcmp(s,"VTG")) return NMEA_SENTENCE_VTG;
  if (!strcmp(s,"GSA")) return NMEA_SENTENCE_GSA;
  if (!strcmp(s,"GSV")) return NMEA_SENTENCE_GSV;
  if (!strcmp(s,"GLL")) return NMEA_SENTENCE_GLL;
  if (!strcmp(s,"TXT")) return NMEA_SENTENCE_TXT;

  return NMEA_SENTENCE_UNDEFINED;
}


uint8_t hexchar_to_byte(char c)
{
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  if (c >= '0' && c <= '9') return c - '0';
  return 0;
}


uint8_t nmea_crccheck(tNmeaMsg* msg)
{
uint8_t c;

  c = (hexchar_to_byte(msg->Checksum1) << 4) + hexchar_to_byte(msg->Checksum2);
  if (c == msg->crc) return 1;
  return 0;
}


//-------------------------------------------------------
// NMEA parser
//-------------------------------------------------------

// reset the parser
void nmea_parser_reset(tGps* gps)
{
  gps->state = GPS_PARSER_STATE_IDLE;
}


// returns 0: further chars expected, 1: last char received, or error
uint8_t nmea_parser_nextchar(tGps* gps, uint8_t c)
{
  tNmeaMsg* msg = &gps->nmea_msg; // this is just to keep code a bit cleaner

  if (c == NMEA_STARTSIGN) {
      msg->StartSign = c; // store it, is not really needed
      msg->cnt = 0; // counts the position in buf
      msg->datafield_cnt = 0; // counts the number of datafields
      msg->datafield[gps->nmea_msg.datafield_cnt++] = gps->buf;
      msg->crc = 0;
      msg->state = NMEA_STATE_DATAFIELD;

      gps->buf[0] = '\0'; // should not matter, but let's play it safe
      gps->state = GPS_PARSER_STATE_NMEA_RECEIVING;
      return 0;
  }

  if (gps->state != GPS_PARSER_STATE_NMEA_RECEIVING) return 1;

  switch (msg->state) {
  case NMEA_STATE_DATAFIELD:
      if (c == NMEA_DATAFIELD_SEPARATOR) { // end of datafield detected
          msg->crc ^= c;
          gps->buf[msg->cnt++] = '\0';
          msg->datafield[msg->datafield_cnt++] = gps->buf + msg->cnt;
      } else
      if (c == NMEA_CRC_SEPARATOR) { // end of datafield and begin of crc field detected
          gps->buf[msg->cnt++] = '\0';
          msg->state = NMEA_STATE_CRC1;
      } else {
          msg->crc ^= c;
          gps->buf[msg->cnt++] = c;
      }
      return 0;
  case NMEA_STATE_CRC1:
      msg->Checksum1 = c;
      msg->state++;
      return 0;
  case NMEA_STATE_CRC2:
      msg->Checksum2 = c;
      msg->state++;
      return 0;
  case NMEA_STATE_CR:
      if (c != 13) {
          gps->state = GPS_PARSER_STATE_NMEA_RECEIVEERROR;
          return 1;
      }
      msg->state++;
      return 0;
  case NMEA_STATE_LF:
      if (c != 10) {
          gps->state = GPS_PARSER_STATE_NMEA_RECEIVEERROR;
          return 1;
      }
      if (nmea_crccheck(msg)) {
          msg->TalkerID = nmea_parse_talkerid((char*)gps->buf);
          strncpy(msg->TalkerIDstr, (char*)gps->buf, 2);
          msg->SentenceID = nmea_parse_sentenceid((char*)gps->buf);
          strncpy(msg->SentenceIDstr, (char*)(gps->buf + 2), 3);
          if (nmea_digestmsg(gps)) {
              gps->state = GPS_PARSER_STATE_NMEA_UPDATED;
          } else {
              gps->state = GPS_PARSER_STATE_NMEA_ERROR;
          }
          return 1;
      }
      gps->state = GPS_PARSER_STATE_NMEA_RECEIVEERROR;
      return 1;
  }
  // no change in gps status
  return 1;
}


//-------------------------------------------------------
// NMEA high level send functions
//-------------------------------------------------------

#define NMEA_PUBX40         "$PUBX,40,???,0,0,0,0"
#define NMEA_PUBX40_LEN     20


void nmea_send_pubx_disable(const char* sentence)
{
char payload[NMEA_PUBX40_LEN+1];

  strcpy(payload, NMEA_PUBX40);
  payload[9]  = sentence[0];
  payload[10] = sentence[1];
  payload[11] = sentence[2];
  nmea_send(payload);
}


#define NMEA_PUBX41         "$PUBX,41,1,0007,0001,9600,0"
#define NMEA_PUBX41_LEN     27


void nmea_send_pubx_cnfg(void)
{
char payload[NMEA_PUBX41_LEN+1];

  strcpy(payload, NMEA_PUBX41);
  nmea_send(payload);
}


//-------------------------------------------------------
// Auxiliary Helpers
//-------------------------------------------------------

// converts the first digit in a string to an integer
uint8_t nmea_todigit(char* data)
{
  return *data - '0';
}


// converts a string of digits to an integer, assumes it has no '.'
uint32_t nmea_tonumeric(char* data)
{
uint32_t v = 0;

  char* c = data;
  while (*c != '\0') {
      v = v*10 + (*c - '0');
      c++;
  }
  return v;
}


// converts a string of digits, which may include a '.', to an integer, whereby multiplying it by 100
// value may be negative
int32_t nmea_tonumericwfraction(char* data, uint32_t fraction)
{
int32_t v = 0;

  // find the '.', if present
  char* point = data;
  while ((*point != '.') && (*point != '\0')) point++;

  // convert the digits before the '.'
  char* c = data;
  if (*c == '-') c++; // skip a leading '-' sign
  while (c < point) {
      v = v * 10 + (*c - '0');
      c++;
  }

  // multiply by 100 to shift
  v *= fraction;

  // convert the digits after the '.'
  if (*point == '.'){ // this is to prevent that point+1 is outside of the string
    char* c = point + 1;
    while (*c != '\0') {
        fraction /= 10;
        v += (*c - '0') * fraction;
        c++;
    }
  }

  if (*c == '-') return -v;
  return v;
}


static inline int32_t nmea_tonumeric100(char* data)
{
  return nmea_tonumericwfraction(data, 100);
}


static inline int32_t nmea_tonumeric1(char* data)
{
  return nmea_tonumericwfraction(data, 1);
}


// the data is in format dddmm.mmmmm or degrees, minute, fractional minutes
// shall be converted to degrees*10^7, the range is thus +-180.0000000°, which can be accommodated by a int32
// the resolution is then 1.1 cm, this might not be sufficient for RTK!
uint32_t nmea_todegrees(char* data)
{
char* c;

  // find the '.', if present
  char* point = data;
  while ((*point != '.') && (*point != '\0')) point++;

  // get the degrees
  uint32_t degrees = 0;
  c = data;
  while (c < point - 2) {
      degrees = degrees*10 + (*c - '0');
      c++;
  }

  // get the minutes, including the fraction
  // see nmea_tonumericwfraction()
  uint32_t minutes = 0;
  c = point - 2;
  while (c < point) {
      minutes = minutes*10 + (*c - '0');
      c++;
  }

  minutes *= (s32)10000000;

  uint32_t fraction = (s32)10000000;
  if (*point == '.') {
    char* c = point + 1;
    while (*c != '\0') {
        fraction /= 10;
        minutes += (*c - '0') * fraction;
        c++;
    }
  }

  // we now have the pieces, so let's put them together
  return (s32)10000000 * degrees + minutes / 60;
}


//-------------------------------------------------------
// NMEA RMC message
//-------------------------------------------------------
// ATTENTION: it seems there can be different version of this sentence
// with different number of datafields
// http://www.nmea.de/nmea0183datensaetze.html#rmc (and others) says 11 fields
// https://gpsd.gitlab.io/gpsd/NMEA.html says 11 for old, 12 for 2.3 and 13 for 4.1
// F9Z datasheets says 16 fields
// M8 datasheets says 13 fields
// the M8 I'm using gives me however 12 fields

typedef enum {
  NMEA_RMC_TIME         = 0x0001,
  NMEA_RMC_STATUS       = 0x0002,
  NMEA_RMC_LATITUDE     = 0x0004,
  NMEA_RMC_LONGITUDE    = 0x0008,
  NMEA_RMC_SPEED        = 0x0010,
  NMEA_RMC_COURSEOVERGROUND = 0x0020,
  NMEA_RMC_DATE         = 0x0040,
  NMEA_RMC_POSMODE      = 0x0080,
} NMEARMCMASKENUM;


typedef struct {
  uint32_t time_utc;
  uint32_t date; // in BCD format yymmday
  int32_t lat_1e7; // in 1e7 deg, 1e7 = 1°
  int32_t lon_1e7; // in 1e7 deg, 1e7 = 1°
  int32_t speed_cms; // in cm/s, 100 = 1 m/s
  uint16_t cog_cdeg; // in cdeg, 100 = 1°
  uint8_t status;
  uint8_t fix;
  uint16_t mask; // this tracks which fields are valid
} tNmeaRMC;


uint8_t nmea_parse_RMC(tNmeaRMC* sentence, tNmeaMsg* msg)
{
  memset(sentence, 0, sizeof(tNmeaRMC)); // invalidate all fields

  if (msg->datafield_cnt < 10) return 0; // 11 fields minimum

  char** datafield = (char**)msg->datafield; // this is just to keep code cleaner

  if (*datafield[1] != '\0') {
      sentence->mask |= NMEA_RMC_TIME;
      sentence->time_utc = nmea_tonumeric1(datafield[1]); // that's the format OpenTx wants, so do it
  }

  if (*datafield[2] != '\0') {
      sentence->mask |= NMEA_RMC_STATUS;
      sentence->status = (*datafield[2] == 'V') ? 1 : 0;
  }

  if ((*datafield[3] != '\0') && (*datafield[4] != '\0')) {
      sentence->mask |= NMEA_RMC_LATITUDE;
      sentence->lat_1e7 = nmea_todegrees(datafield[3]);
      if (*datafield[4] == 'S') sentence->lat_1e7 = -sentence->lat_1e7;
  }

  if ((*datafield[5] != '\0') && (*datafield[6] != '\0')) {
      sentence->mask |= NMEA_RMC_LONGITUDE;
      sentence->lon_1e7 = nmea_todegrees(datafield[5]);
      if (*datafield[6] == 'W') sentence->lon_1e7 = -sentence->lon_1e7;
  }

  if (*datafield[7] != '\0') {
      sentence->mask |= NMEA_RMC_SPEED;
      sentence->speed_cms = (nmea_tonumeric100(datafield[7]) * 5144) / 10000;
  }

  if (*datafield[8] != '\0') {
      sentence->mask |= NMEA_RMC_COURSEOVERGROUND;
      sentence->cog_cdeg = nmea_tonumeric100(datafield[8]);
  }

  if (*datafield[9] != '\0') {
      sentence->mask |= NMEA_RMC_DATE;
      sentence->date = nmea_tonumeric1(datafield[9]); // that's the format OpenTx wants, so do it
  }

  if (*datafield[12] != '\0') {
      sentence->mask |= NMEA_RMC_POSMODE;
      switch (*datafield[12]) {
      case 'N': sentence->fix = 0; break;
      case 'A': sentence->fix = 1; break;
      case 'D': sentence->fix = 2; break;
      case 'R': sentence->fix = 4; break;
      case 'F': sentence->fix = 5; break;
      case 'E': sentence->fix = 6; break;
      default: sentence->fix = 0;
      }
  }

  return 1;
}


//-------------------------------------------------------
// NMEA GGA message
//-------------------------------------------------------

typedef enum {
  NMEA_GGA_TIME         = 0x0001,
  NMEA_GGA_LATITUDE     = 0x0002,
  NMEA_GGA_LONGITUDE    = 0x0004,
  NMEA_GGA_QUALITY      = 0x0008,
  NMEA_GGA_SAT          = 0x0010,
  NMEA_GGA_HDOP         = 0x0020,
  NMEA_GGA_ALTITUDE     = 0x0040
} NMEAGGAMASKENUM;


typedef struct {
  uint32_t time_utc;
  int32_t lat_1e7; // in 1e7 deg, 1e7 = 1°
  int32_t lon_1e7; // in 1e7 deg, 1e7 = 1°
  int32_t alt_cm; // in cm, 100 = 1 m, is above sea level in m
  uint8_t fix;
  uint8_t sat;
  uint16_t hdop; // 100 = 1.0
  uint16_t mask; // this tracks which fields are valid
} tNmeaGGA;


// returns 1 if NMEA sentence could be successfully parsed
// datafield is an array of pointers to the datafields in the sentence
uint8_t nmea_parse_GGA(tNmeaGGA* sentence, tNmeaMsg* msg)
{
  memset(sentence, 0, sizeof(tNmeaGGA)); // invalidate all fields

  if (msg->datafield_cnt < 15) return 0; // it has 16 datafields

  char** datafield = (char**)msg->datafield; // this is just to keep code cleaner

  if (*datafield[1] != '\0') {
      sentence->mask |= NMEA_GGA_TIME;
      sentence->time_utc = nmea_tonumeric1(datafield[1]); // that's the format OpenTx wants, so do it
  }

  if ((*datafield[2] != '\0') && (*datafield[3] != '\0')) {
      sentence->mask |= NMEA_GGA_LATITUDE;
      sentence->lat_1e7 = nmea_todegrees(datafield[2]);
      if (*datafield[3] == 'S') sentence->lat_1e7 = -sentence->lat_1e7;
  }

  if ((*datafield[4] != '\0') && (*datafield[5] != '\0')) {
      sentence->mask |= NMEA_GGA_LONGITUDE;
      sentence->lon_1e7 = nmea_todegrees(datafield[4]);
      if (*datafield[5] == 'W') sentence->lon_1e7 = -sentence->lon_1e7;
  }

  if (*datafield[6] != '\0') {
      sentence->mask |= NMEA_GGA_QUALITY;
      sentence->fix = nmea_todigit(datafield[6]);
  }

  if (*datafield[7] != '\0') {
      sentence->mask |= NMEA_GGA_SAT;
      sentence->sat = nmea_tonumeric(datafield[7]);
  }

  if (*datafield[8] != '\0') {
      sentence->mask |= NMEA_GGA_HDOP;
      sentence->hdop = nmea_tonumeric100(datafield[8]);
  }

  if (*datafield[9] != '\0') { // we could/should test also for *datafield[10] == 'M', but this seems to be fixed
      sentence->mask |= NMEA_GGA_ALTITUDE;
      sentence->alt_cm = nmea_tonumeric100(datafield[9]);
  }

  return 1;
}



#endif // GPS_NMEA_H



