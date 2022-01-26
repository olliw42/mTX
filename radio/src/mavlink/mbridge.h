//*******************************************************
// MavTelem Library
// Copyright (c) OlliW, OlliW42, www.olliw.eu
// LGPL3
// https://www.gnu.org/licenses/lgpl-3.0.en.html
//*******************************************************

#pragma once
#ifndef MBRIDGE_H
#define MBRIDGE_H

// -- MBridge definitions --

#define MBRIDGE_STX1                                'O'
#define MBRIDGE_STX2                                'W'

#define MBRIDGE_R2M_SERIALPACKET_PAYLOAD_LEN_MAX    17

#define MBRIDGE_CHANNELPACKET_SIZE                  22

#define MBRIDGE_M2R_COMMANDPACKET_PAYLOAD_LEN       12
#define MBRIDGE_R2M_COMMANDPACKET_PAYLOAD_LEN       22

typedef enum {
  MBRIDGE_SERIALPACKET_STX    = 0x00,
  MBRIDGE_CHANNELPACKET_STX   = 0xFF,
  MBRIDGE_COMMANDPACKET_STX   = 0xA0, // 0b101x
  MBRIDGE_COMMANDPACKET_MASK  = 0xE0, // 0b111x
} MBRIDGE_PACKET_STX_ENUM;


// -- packets as exchanged over MBridge
// note that TX means here = received from module !
// these defines are largely copied over from mbridge class from mLRS, in order to not have too much confusion

#define MBRIDGE_PACKED(__Declaration__) __Declaration__ __attribute__((packed))

typedef enum {
  MBRIDGE_CMD_TX_LINK_STATS = 0x02,
} MBRIDGE_CMD_TX_ENUM;


MBRIDGE_PACKED(
typedef struct
{
  int8_t rssi;
  uint8_t LQ;
  int8_t snr; // invalid = INT8_MAX
  int8_t rssi2; // in case of 2nd antenna, invalid = INT8_MAX

  int8_t receiver_rssi;
  uint8_t receiver_LQ;
  int8_t receiver_snr; // invalid = INT8_MAX
  int8_t receiver_rssi2; // in case of 2nd antenna, invalid = INT8_MAX

  uint8_t ant_no : 1; // 0: antenna 1, 1: antenna 2
  uint8_t receiver_ant_no : 1; // 0: antenna 1, 1: antenna 2
  uint8_t spare_bits : 6;

  uint8_t LQ_frames_received;
  uint8_t LQ_received;
  uint8_t LQ_valid_received;
}) tMBridgeLinkStats;


// -- MBridge class --

class MBridge
{
  public:
    MBridge() { } // constructor

    void read_in();
    void send_serialpacket(void);
    void send_channelpacket(void);

    struct LinkStats { // may not be exactly what is send in packet
      int8_t rssi;
      uint8_t LQ;
      int8_t snr;
      int8_t rssi2;
      uint8_t ant_no;
      int8_t receiver_rssi;
      uint8_t receiver_LQ;
      int8_t receiver_snr;
      int8_t receiver_rssi2;
      uint8_t receiver_ant_no;
      // only momentarily for debug
      uint8_t LQ_frames_received;
      uint8_t LQ_received;
      uint8_t LQ_valid_received;
    };
    struct LinkStats link_stats;

  private:
    bool cmd_get(uint8_t* cmd, uint8_t* payload, uint8_t* len);

    void get_channels(uint8_t* _payload, uint8_t* len);

    void set_linkstats(tMBridgeLinkStats* ls);
};

extern MBridge mBridge;


#endif // MBRIDGE_H
