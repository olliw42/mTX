//*******************************************************
// GPS Library
// Copyright (c) OlliW, OlliW42, www.olliw.eu
// Licence GPL2 or (at your option) GPL3
// https://www.gnu.org/licenses/gpl-2.0.en.html
// https://www.gnu.org/licenses/gpl-3.0.en.html
//*******************************************************

#ifndef GPS_UBX_H
#define GPS_UBX_H


void gps_hal_putc(char c);

// returns 1 if NMEA message could be successfully parsed, or if error occurred
uint8_t ubx_digestmsg(tGps* gps);


//-------------------------------------------------------
// UBX
//-------------------------------------------------------
// ubx frame structure
// stx1         = 1 byte
// stx2         = 1 byte
// class id     = 1 byte
// message id   = 1 byte
// payload len  = 2 bytes
// payload
// crc          = 2 bytes
// => header len = 6 bytes
// => frame len = 8 + payload len


#define UBX_STARTSIGN1            0xB5
#define UBX_STARTSIGN2            0x62


typedef enum {
  UBX_STATE_STX1 = 0, // does never occur
  UBX_STATE_STX2,
  UBX_STATE_CLASSID,
  UBX_STATE_MESSAGEID,
  UBX_STATE_LENGTH1,
  UBX_STATE_LENGTH2,
  UBX_STATE_PAYLOAD,
  UBX_STATE_CRCA,
  UBX_STATE_CRCB,
} UBXESTATEENUM;

/*
typedef struct {
  uint8_t StartSign1;
  uint8_t StartSign2;
  uint8_t ClassID;
  uint8_t MessageID;
  uint16_t Length;
  uint8_t CheckSumA;
  uint8_t CheckSumB;

  uint16_t state;
  uint16_t payload_cnt;
  uint8_t crcA;
  uint8_t crcB;
} tUbxMsg;
*/


static inline void _ubx_putc_wcrc(char c, uint8_t* crcA, uint8_t* crcB)
{
  gps_hal_putc(c);
  (*crcA) += c;
  (*crcB) += *crcA;
}


void ubx_send(uint8_t classid, uint8_t msgid, uint8_t* payload, uint16_t len)
{
uint8_t crcA = 0, crcB = 0;

  gps_hal_putc(UBX_STARTSIGN1);
  gps_hal_putc(UBX_STARTSIGN2);

  _ubx_putc_wcrc(classid, &crcA, &crcB);
  _ubx_putc_wcrc(msgid, &crcA, &crcB);
  _ubx_putc_wcrc(len, &crcA, &crcB);
  _ubx_putc_wcrc(len >> 8, &crcA, &crcB);
  for(uint16_t n = 0; n < len; n++) {
      _ubx_putc_wcrc(payload[n], &crcA, &crcB);
  }

  gps_hal_putc(crcA);
  gps_hal_putc(crcB);
}


//-------------------------------------------------------
// UBX parser
//-------------------------------------------------------

void ubx_parser_reset(tGps* gps)
{
  gps->state = GPS_PARSER_STATE_IDLE;
}


// return 0: further chars expected, 1: last char received, or error
uint16_t ubx_parser_nextchar(tGps* gps, uint8_t c)
{
  tUbxMsg* msg = &gps->ubx_msg; // this is just to keep code a bit cleaner

  if (c == UBX_STARTSIGN1) {
      msg->StartSign1 = c; // store it, is not really needed
      msg->payload_cnt = 0;
      msg->crcA = 0;
      msg->crcB = 0;
      msg->state = UBX_STATE_STX2;

      gps->buf[0] = '\0'; // should not matter, but let's play it safe
      gps->state = GPS_PARSER_STATE_UBX_RECEIVING;
      return 0;
  }

  if (gps->state != GPS_PARSER_STATE_UBX_RECEIVING) return 1;

  switch (msg->state) {
  case UBX_STATE_STX2:
      if (c != UBX_STARTSIGN2) {
          gps->state = GPS_PARSER_STATE_UBX_RECEIVEERROR;
          return 1;
      }
      msg->StartSign2 = c; // store it, is not really needed
      msg->state++;
      return 0;
  case UBX_STATE_CLASSID:
      msg->ClassID = c;
      msg->crcA = c;
      msg->crcB = msg->crcA;
      msg->state++;
      return 0;
  case UBX_STATE_MESSAGEID:
      msg->MessageID = c;
      msg->crcA += c;
      msg->crcB += msg->crcA;
      msg->state++;
      return 0;
  case UBX_STATE_LENGTH1:
      msg->Length = c;
      msg->crcA += c;
      msg->crcB += msg->crcA;
      msg->state++;
      return 0;
  case UBX_STATE_LENGTH2:
      msg->Length += ((uint16_t)c << 8);
      // we want to add a protection to avoid buffer overflow
      msg->crcA += c;
      msg->crcB += msg->crcA;
      msg->state++;
      msg->payload_cnt = 0; // not needed, was done already, let's play it safe ;)
      return 0;
  case UBX_STATE_PAYLOAD:
      gps->buf[msg->payload_cnt++] = c;
      msg->crcA += c;
      msg->crcB += msg->crcA;
      if (msg->payload_cnt >= msg->Length) msg->state++;
      return 0;
  case UBX_STATE_CRCA:
      msg->CheckSumA = c;
      msg->state++;
      return 0;
  case UBX_STATE_CRCB:
      msg->CheckSumB = c;
      // complete ubx packet received
      if ((msg->crcA == msg->CheckSumA) && (msg->crcB == msg->CheckSumB)) {
        if (ubx_digestmsg(gps)) {
            gps->state = GPS_PARSER_STATE_UBX_UPDATED;
        } else {
            gps->state = GPS_PARSER_STATE_UBX_ERROR;
        }
        return 1;
      }
      gps->state = GPS_PARSER_STATE_UBX_RECEIVEERROR;
      return 1;
  }

  //no change in gps status
  return 1;
}


//-------------------------------------------------------
// UBX message structures
//-------------------------------------------------------

typedef enum {
  UBX_CLASS_NAV = 0x01,
  UBX_CLASS_ACK = 0x05,
  UBX_CLASS_CFG = 0x06,
} UBXCLASSENUM;


typedef enum {
  UBX_ACK_NACK = 0x00,
  UBX_ACK_ACK = 0x01,
} UBXCLASSACKMESSAGEENUM;


typedef enum {
  UBX_CNF_PRT = 0x00,
  UBX_CNF_MSG = 0x01,
  UBX_CNF_RATE = 0x08,
} UBXCLASSCNFMESSAGEENUM;


typedef enum { // this is in the sequence of how m8 is sending them
  UBX_NAV_SOL = 0x06,
  UBX_NAV_PVT = 0x07,
  UBX_NAV_SAT = 0x35,
  UBX_NAV_DOP = 0x04,
  UBX_NAV_TIMEGPS = 0x20,
} UBXCLASSNAVMESSAGEENUM;


//-------------------------------------------------------
// UBX NAV DOP 0x04

GPS_PACKED(
typedef struct {
  uint32_t iTOW;        // ms
  uint16_t gDOP;        // scaling 1e-2
  uint16_t pDOP;        // scaling 1e-2
  uint16_t tDOP;        // scaling 1e-2
  uint16_t vDOP;        // scaling 1e-2
  uint16_t hDOP;        // scaling 1e-2
  uint16_t nDOP;        // scaling 1e-2
  uint16_t eDOP;        // scaling 1e-2
}) tUbxNavDopPacket;

#define UBLOX_UBXNAVDOP_LEN  18


//-------------------------------------------------------
// UBX NAV SOL 0x06

typedef enum {
  UBX_NAV_SOL_FLAGS_GNSSFIXOK = (1<<0),
  UBX_NAV_SOL_FLAGS_DIFFSOLN = (1<<1),
  UBX_NAV_SOL_FLAGS_WKNSET = (1<<2),
  UBX_NAV_SOL_FLAGS_TOWSET = (1<<3),
} UBXNAVSOLFLAGSENUM;

GPS_PACKED(
typedef struct {
  uint32_t iTOW;        // ms
  int32_t  fTOW;        // ns, precise gps time = (iTOW * 1e-3) + (fTOW * 1e-9)
  int16_t  week;        // week, GPS week number of navigation epoche
  uint8_t  gpsFix;
  uint8_t  flags;       // fix status flag
  int32_t  ecefXYZ[3];  // cm
  uint32_t pAcc;        // cm
  int32_t  ecefVXYZ[3]; // cm/s
  uint32_t sAcc;        // cm/s
  uint16_t pDOP;        // scaling 1e-2
  uint8_t  reserved1[1];
  uint8_t  numSV;
  uint8_t  reserved2[4];
}) tUbxNavSolPacket;

#define UBLOX_UBXNAVSOL_LEN  52


//-------------------------------------------------------
// UBX NAV PVT 0x07

typedef enum {
  UBX_NAV_PVT_VALID_DATE = (1<<0),
  UBX_NAV_PVT_VALID_TIME = (1<<1),
  UBX_NAV_PVT_VALID_FULLYRESOLVED = (1<<2),
  UBX_NAV_PVT_VALID_MAG = (1<<3),
} UBXNAVPVTVALIDENUM;

typedef enum {
  UBX_NAV_PVT_FIXTYPE_NOFIX = 0,
  UBX_NAV_PVT_FIXTYPE_DEADRECKONINGONLY = 1,
  UBX_NAV_PVT_FIXTYPE_2DFIX = 2,
  UBX_NAV_PVT_FIXTYPE_3DFIX = 3,
  UBX_NAV_PVT_FIXTYPE_GNSSDEADRECKONING = 4,
  UBX_NAV_PVT_FIXTYPE_TIMEONLYFIX = 5,
} UBXNAVPVTFIXTYPEENUM;

typedef enum {
  UBX_NAV_PVT_FLAGS_GNSSFIXOK = (1<<0),
  UBX_NAV_PVT_FLAGS_DIFFSOLN = (1<<1),
  UBX_NAV_PVT_FLAGS_HEADVEHVALID = (1<<5),
} UBXNAVPVTFLAGSENUM;

typedef enum {
  UBX_NAV_PVT_FLAGS2_CONFIRMAVAIL = (1<<5),
  UBX_NAV_PVT_FLAGS2_CONFIRMDATE = (1<<6),
  UBX_NAV_PVT_FLAGS2_CONFIRMTIME = (1<<7),
} UBXNAVPVTFLAGS2ENUM;

GPS_PACKED(
typedef struct {
  uint32_t iTOW;        // ms
  uint16_t year;
  uint8_t  month;       // m, 1...12
  uint8_t  day;         // d, 1...31
  uint8_t  hour;        // h, 0...23
  uint8_t  min;         // min, 0...59
  uint8_t  sec;         // s, 0...60
  uint8_t  valid;       // time validity flag
  uint32_t tAcc;        // ns
  int32_t  nano;        // ns
  uint8_t  fixType;
  uint8_t  flags;       // fix status flag
  uint8_t  flags2;
  uint8_t  numSV;
  int32_t  lon;         // deg, scaling 1e-7
  int32_t  lat;         // deg, scaling ie-7
  int32_t  height;      // mm
  int32_t  hMSL;        // mm
  uint32_t hAcc;        // mm
  uint32_t vAcc;        // mm
  int32_t  velN;        // mm/s
  int32_t  velE;        // mm/s
  int32_t  velD;        // mm/s
  int32_t  gSpeed;      // mm/s
  int32_t  headMot;     // deg, scaling 1e-5
  uint32_t sAcc;        // mm/s
  uint32_t headAcc;     // deg, scaling 1e-5
  uint16_t pDOP;        // scaling 1e-2
  uint8_t  reserved1[6];
  uint32_t headVeh;     // deg, scaling 1e-5
  uint8_t  reserved2[4];
}) tUbxNavPvtPacket;

#define UBLOX_UBXNAVPVT_LEN  92


//-------------------------------------------------------
// UBX NAV TIMEGPS 0x20

GPS_PACKED(
typedef struct {
  uint32_t iTOW;        // ms
  int32_t  fTOW;        // ns, precise gps time = (iTOW * 1e-3) + (fTOW * 1e-9)
  int16_t  week;        // week, GPS week number of navigation epoche
  int8_t   leapS;       // s, leap seconds
  uint8_t  valid;       // time validity flag
  uint32_t tAcc;        // ns
}) tUbxNavTimeGpsPacket;

#define UBLOX_UBXNAVTIMEGPS_LEN  16


//-------------------------------------------------------
// UBX NAV SAT 0x35

//this packet has variable length, depending on the number of satellites reported
// we thus capture ONLY the heading info
GPS_PACKED(
typedef struct {
  uint32_t iTOW;        // ms
  uint8_t  version;     // should be 1
  uint8_t  numSVs;      // number of SVs which are either known to be visible or currently tracked by the receiver
  uint8_t  reserved[2];
}) tUbxNavSatPacket;

#define UBLOX_UBXNAVSAT_MIN_LEN  8


//-------------------------------------------------------
// UBX high level send functions
//-------------------------------------------------------

#define UBLOX_UBXCNFMSG_DOP1      "\x01\x04\x01"
#define UBLOX_UBXCNFMSG_PVT1      "\x01\x07\x01"
#define UBLOX_UBXCNFMSG_SAT1      "\x01\x35\x01"
#define UBLOX_UBXCNFMSG_SOL1      "\x01\x06\x01"
#define UBLOX_UBXCNFMSG_TIMEGPS1  "\x01\x20\x01"
#define UBLOX_UBXCNFMSG_LEN       3

#define UBLOX_UBXCNFRATE_1HZ      "\xE8\x03\x01\x00\x00\x00" // 1Hz
#define UBLOX_UBXCNFRATE_5HZ      "\xC8\x00\x01\x00\x00\x00" // 5Hz
#define UBLOX_UBXCNFRATE          "??\x01\x00\x00\x00"
#define UBLOX_UBXCNFRATE_LEN      6

// for some reason this should come after the CNF-MSG packets
// @4:  0x000008D0 = 8bit, no parity, 1 stop bit
// @8:  0x0000E100 = 57600 bps
// @12: 0x0007 = inUbx UBX protocol, inNmea NMEA protocol, inRtcm
// @14: 0x0001 = outUbx UBX protocol
#define UBLOX_UBXCNFPRT_DEFAULT   "\x01\x00\x00\x00\xD0\x08\x00\x00\x00\xE1\x00\x00\x07\x00\x01\x00\x00\x00\x00\x00"
#define UBLOX_UBXCNFPRT_LEN       20


static inline void ubx_send_cnfmsg_pvt1(void)
{
  ubx_send(UBX_CLASS_CFG, UBX_CNF_MSG, (uint8_t*)UBLOX_UBXCNFMSG_PVT1, UBLOX_UBXCNFMSG_LEN);
}


static inline void ubx_send_cnfmsg_dop1(void)
{
  ubx_send(UBX_CLASS_CFG, UBX_CNF_MSG, (uint8_t*)UBLOX_UBXCNFMSG_DOP1, UBLOX_UBXCNFMSG_LEN);
}


static inline void ubx_send_cnfmsg_sol1(void)
{
  ubx_send(UBX_CLASS_CFG, UBX_CNF_MSG, (uint8_t*)UBLOX_UBXCNFMSG_SOL1, UBLOX_UBXCNFMSG_LEN);
}


static inline void ubx_send_cnfmsg_timegps1(void)
{
  ubx_send(UBX_CLASS_CFG, UBX_CNF_MSG, (uint8_t*)UBLOX_UBXCNFMSG_TIMEGPS1, UBLOX_UBXCNFMSG_LEN);
}


static inline void ubx_send_cnfmsg_sat1(void)
{
  ubx_send(UBX_CLASS_CFG, UBX_CNF_MSG, (uint8_t*)UBLOX_UBXCNFMSG_SAT1, UBLOX_UBXCNFMSG_LEN);
}


static inline void ubx_send_cnfrate(uint16_t rate_ms)
{
  uint8_t payload[UBLOX_UBXCNFRATE_LEN+1] = UBLOX_UBXCNFRATE_5HZ;
  payload[0] = rate_ms;
  payload[1] = rate_ms >> 8;
  ubx_send(UBX_CLASS_CFG, UBX_CNF_RATE, payload, UBLOX_UBXCNFRATE_LEN);
}


static inline void ubx_send_cnfprt_default(void)
{
   ubx_send(UBX_CLASS_CFG, UBX_CNF_PRT, (uint8_t*)UBLOX_UBXCNFPRT_DEFAULT, UBLOX_UBXCNFPRT_LEN);
}


#endif // GPS_UBX_H
