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

#define MBRIDGE_R2M_SERIAL_PAYLOAD_LEN_MAX          24

#define MBRIDGE_CHANNELPACKET_SIZE                  23

#define MBRIDGE_M2R_COMMAND_PAYLOAD_LEN_MAX         24


typedef enum {
  MBRIDGE_CHANNELPACKET_STX   = 0xFF, // marker which indicates a channel packet
  MBRIDGE_COMMANDPACKET_STX   = 0xA0, // 0b101x marker which indicates a command packet
  MBRIDGE_COMMANDPACKET_MASK  = 0xE0, // 0b111x
} MBRIDGE_PACKET_STX_ENUM;


// -- packets as exchanged over MBridge
// note that TX means here = received from module !
// these defines are largely copied over from mbridge class from mLRS, in order to not have too much confusion

#define MBRIDGE_PACKED(__Declaration__) __Declaration__ __attribute__((packed))


typedef enum {
  MBRIDGE_CMD_TX_LINK_STATS = 0x02,
} MBRIDGE_CMD_ENUM;


#define MBRIDGE_CMD_TX_LINK_STATS_LEN  22


MBRIDGE_PACKED(
typedef struct
{
  // transmitter side of things
  uint8_t LQ;
  int8_t rssi_instantaneous;
  int8_t snr_instantaneous; // invalid = INT8_MAX
  int8_t rssi2_instantaneous; // invalid = INT8_MAX

  int8_t rssi_filtered;
  int8_t snr_filtered;
  int8_t rssi2_filtered;

  // receiver side of things
  uint8_t receiver_LQ;
  uint8_t receiver_LQ_serial;
  int8_t receiver_rssi_instantaneous;
  int8_t receiver_snr_instantaneous; // invalid = INT8_MAX
  int8_t receiver_rssi2_instantaneous; // invalid = INT8_MAX

  int8_t receiver_rssi_filtered;
  int8_t receiver_snr_filtered;
  int8_t receiver_rssi2_filtered;

  // both
  uint8_t ant_no : 1;
  uint8_t receiver_ant_no : 1;
  uint8_t spare_bits : 6;

  // further stats acquired on transmitter side
  uint8_t LQ_fresh_serial_packets_transmitted;
  uint8_t bytes_per_sec_transmitted;

  uint8_t LQ_valid_received; // number of completely valid packets received per sec
  uint8_t LQ_fresh_serial_packets_received;
  uint8_t bytes_per_sec_received;

  uint8_t LQ_received; // number of packets received per sec, not practically relevant
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
      uint8_t LQ;
      int8_t rssi_instantaneous;
      int8_t snr_instantaneous; // invalid = INT8_MAX
      int8_t rssi2_instantaneous; // invalid = INT8_MAX
      uint8_t ant_no;
      int8_t rssi_filtered;
      int8_t snr_filtered;
      int8_t rssi2_filtered;

      uint8_t receiver_LQ;
      uint8_t receiver_LQ_serial;
      int8_t receiver_rssi_instantaneous;
      int8_t receiver_snr_instantaneous; // invalid = INT8_MAX
      int8_t receiver_rssi2_instantaneous; // invalid = INT8_MAX
      uint8_t receiver_ant_no;
      int8_t receiver_rssi_filtered;
      int8_t receiver_snr_filtered;
      int8_t receiver_rssi2_filtered;

      uint8_t LQ_fresh_serial_packets_transmitted;
      uint8_t bytes_per_sec_transmitted;
      uint8_t LQ_valid_received; // number of completely valid packets received per sec
      uint8_t LQ_fresh_serial_packets_received;
      uint8_t bytes_per_sec_received;

      uint8_t LQ_received; // number of packets received per sec, not practically relevant
    };
    struct LinkStats link_stats;

  private:
    uint8_t cmd_payload_len(uint8_t cmd);
    bool cmd_get(uint8_t* cmd, uint8_t* payload, uint8_t* len);

    void get_channels(uint8_t* _payload, uint8_t* len);

    void set_linkstats(tMBridgeLinkStats* ls);
};

extern MBridge mBridge;


#endif // MBRIDGE_H
