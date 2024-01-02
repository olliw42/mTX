//*******************************************************
// mTX: MAVLink for OpenTx Project
// Copyright (c) OlliW, OlliW42, www.olliw.eu
// Licence GPL2 or (at your option) GPL3
// https://www.gnu.org/licenses/gpl-2.0.en.html
// https://www.gnu.org/licenses/gpl-3.0.en.html
// MavTelem Library
//*******************************************************

#pragma once
#ifndef MBRIDGE_H
#define MBRIDGE_H

// -- MBridge definitions --
// format:
// radio->module:  stx1 stx2 len/cmd payload
// module->radio:  0/cmd payload

#define MBRIDGE_STX1                          'O'
#define MBRIDGE_STX2                          'W'

#define MBRIDGE_R2M_SERIAL_PAYLOAD_LEN_MAX    24 // up to 24 bytes payload when received from transmitter
#define MBRIDGE_M2R_SERIAL_PAYLOAD_LEN_MAX    24 // up to 24 bytes payload when send from module to transmitter

#define MBRIDGE_CHANNELPACKET_SIZE            23 // 23 bytes payload, only received from transmitter

#define MBRIDGE_R2M_COMMAND_PAYLOAD_LEN_MAX   24 // 24 bytes payload
#define MBRIDGE_M2R_COMMAND_PAYLOAD_LEN_MAX   24 // 24 bytes payload

#define MBRIDGE_R2M_COMMAND_FRAME_LEN_MAX     25 // cmd byte + 24 bytes payload
#define MBRIDGE_M2R_COMMAND_FRAME_LEN_MAX     25 // cmd byte + 24 bytes payload


typedef enum {
  MBRIDGE_CHANNELPACKET_STX   = 0xFF, // marker which indicates a channel packet
  MBRIDGE_COMMANDPACKET_STX   = 0xA0, // 0b101x marker which indicates a command packet
  MBRIDGE_COMMANDPACKET_MASK  = 0xE0, // 0b111x
} MBRIDGE_PACKET_STX_ENUM;


typedef enum {
    MBRIDGE_CMD_TX_LINK_STATS         = 2,
    MBRIDGE_CMD_REQUEST_INFO          = 3, // len = 0
    MBRIDGE_CMD_DEVICE_ITEM_TX        = 4,
    MBRIDGE_CMD_DEVICE_ITEM_RX        = 5,
    MBRIDGE_CMD_PARAM_REQUEST_LIST    = 6, // len = 0
    MBRIDGE_CMD_PARAM_ITEM            = 7,
    MBRIDGE_CMD_PARAM_ITEM2           = 8,
    MBRIDGE_CMD_PARAM_ITEM3           = 9,
    MBRIDGE_CMD_REQUEST_CMD           = 10,
    MBRIDGE_CMD_INFO                  = 11,
    MBRIDGE_CMD_PARAM_SET             = 12,
    MBRIDGE_CMD_PARAM_STORE           = 13, // len = 0
    MBRIDGE_CMD_BIND_START            = 14, // len = 0
    MBRIDGE_CMD_BIND_STOP             = 15, // len = 0
    MBRIDGE_CMD_MODELID_SET           = 16,
} MBRIDGE_CMD_ENUM;

#define MBRIDGE_CMD_TX_LINK_STATS_LEN         22
#define MBRIDGE_CMD_DEVICE_ITEM_LEN           24
#define MBRIDGE_CMD_PARAM_ITEM_LEN            24
#define MBRIDGE_CMD_REQUEST_CMD_LEN           18
#define MBRIDGE_CMD_INFO_LEN                  24
#define MBRIDGE_CMD_PARAM_SET_LEN             7
#define MBRIDGE_CMD_MODELID_SET_LEN           3


// -- packets as exchanged over MBridge
// note that TX means here = received from module !
// these defines are largely copied over from mbridge class from mLRS, in order to not have too much confusion

#define MBRIDGE_PACKED(__Declaration__) __Declaration__ __attribute__((packed))


MBRIDGE_PACKED(
typedef struct
{
  // transmitter side of things
  uint8_t LQ;
  int8_t rssi1_instantaneous; // invalid = INT8_MAX
  int8_t rssi2_instantaneous; // invalid = INT8_MAX
  int8_t snr_instantaneous;

  int8_t rssi1_filtered;
  int8_t rssi2_filtered;
  int8_t snr_filtered;

  // receiver side of things
  uint8_t receiver_LQ;
  uint8_t receiver_LQ_serial;
  int8_t receiver_rssi_instantaneous;

  int8_t receiver_rssi_filtered;

  // both
  uint8_t receive_antenna : 1;
  uint8_t transmit_antenna : 1;
  uint8_t receiver_receive_antenna : 1;
  uint8_t receiver_transmit_antenna : 1;
  uint8_t diversity : 1;
  uint8_t receiver_diversity : 1;
  uint8_t rx1_valid : 1;
  uint8_t rx2_valid : 1;

  // further stats acquired on transmitter side
  uint8_t LQ_fresh_serial_packets_transmitted;
  uint8_t bytes_per_sec_transmitted;
  uint8_t LQ_valid_received;  // number of completely valid packets received per sec
  uint8_t LQ_fresh_serial_packets_received;
  uint8_t bytes_per_sec_received;

  uint8_t LQ_received; // number of packets received per sec, not practically relevant

  uint8_t fhss_curr_i;
  uint8_t fhss_cnt;

  uint8_t vehicle_state : 2; // 0 = disarmed, 1 = armed 2 = flying, 3 = invalid/unknown
  uint8_t spare : 6;

  uint8_t link_state_connected : 1;
  uint8_t link_state_binding : 1;
  uint8_t spare2 : 6;
}) tMBridgeLinkStats;


typedef enum {
  MBRIDGE_PARAM_TYPE_UINT8 = 0,
  MBRIDGE_PARAM_TYPE_INT8,
  MBRIDGE_PARAM_TYPE_UINT16,
  MBRIDGE_PARAM_TYPE_INT16,
  MBRIDGE_PARAM_TYPE_LIST,
  MBRIDGE_PARAM_TYPE_STR6,
} MBRIDGE_PARAM_TYPE_ENUM;


// -- MBridge class --

class MBridge
{
  public:
    MBridge(); // constructor

    void triggerSendModelId(uint8_t new_model_id) { _send_model_id = true; _model_id = new_model_id; }
    void triggerSendBindStart(void) { _send_bind = true; _send_bind_start = true; }
    void triggerSendBindStop(void) { _send_bind = true; _send_bind_start = false; }

    void read_packet();
    void send_serialpacket(void);
    void send_channelpacket(void);
    bool send_cmdpacket(void);
    bool linkstats_updated(void);

    bool send_bindpacket(void);
    bool send_modelidpacket(void);

    #define MBRIDGE_RSSI_LIST_LEN  32

    struct LinkStats { // this is not exactly what is send in the mBridge LinkStats packet, see tMBridgeLinkStats
      uint8_t LQ;
      int8_t rssi1_instantaneous; // invalid = INT8_MAX
      int8_t rssi2_instantaneous; // invalid = INT8_MAX
      int8_t snr_instantaneous; // invalid = INT8_MAX
      uint8_t receive_antenna;
      uint8_t transmit_antenna;
      uint8_t diversity;
      int8_t rssi2_filtered;
      int8_t rssi1_filtered;
      int8_t snr_filtered;

      uint8_t receiver_LQ;
      uint8_t receiver_LQ_serial;
      int8_t receiver_rssi_instantaneous;
      int8_t receiver_snr_instantaneous; // invalid = INT8_MAX
      uint8_t receiver_receive_antenna;
      uint8_t receiver_transmit_antenna;
      uint8_t receiver_diversity;
      int8_t receiver_rssi_filtered;
      int8_t receiver_snr_filtered;

      uint8_t LQ_fresh_serial_packets_transmitted;
      uint8_t bytes_per_sec_transmitted;
      uint8_t LQ_valid_received; // number of completely valid packets received per sec
      uint8_t LQ_fresh_serial_packets_received;
      uint8_t bytes_per_sec_received;

      uint8_t LQ_received; // number of packets received per sec, not practically relevant

      uint8_t rx1_valid;
      uint8_t rx2_valid;
      uint8_t fhss_curr_i;
      uint8_t fhss_cnt;

      int8_t rssi1_list[MBRIDGE_RSSI_LIST_LEN];
      int8_t rssi2_list[MBRIDGE_RSSI_LIST_LEN];
      int8_t receiver_rssi_list[MBRIDGE_RSSI_LIST_LEN];
      uint8_t receiver_antenna_list[MBRIDGE_RSSI_LIST_LEN];
    };
    struct LinkStats link_stats;

    struct CmdPacket {
      uint8_t cmd;
      uint8_t len; // helper field, to keep len
      uint8_t payload[MBRIDGE_R2M_COMMAND_PAYLOAD_LEN_MAX]; // used for both rx & tx, payload sizes are equal
    };
    Fifo<struct CmdPacket, 16> tx_cmd_fifo;
    Fifo<struct CmdPacket, 64> rx_cmd_fifo;

  private:
    bool _link_stats_updated;

    uint8_t cmd_payload_len(uint8_t cmd);
    bool cmd_get(uint8_t* cmd, uint8_t* payload, uint8_t* len);

    void get_channels(uint8_t* _payload, uint8_t* len);

    void set_linkstats(tMBridgeLinkStats* ls);

    void _send_cmdpacket(struct CmdPacket* pkt);

  public:
    bool _send_model_id;
    uint8_t _model_id;
    bool _send_bind;
    uint32_t _send_bind_tsend_10ms;
    bool _send_bind_start;
};

extern MBridge mBridge;


#endif // MBRIDGE_H
