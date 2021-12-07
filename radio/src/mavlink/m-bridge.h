/*
 * The MAVLink for OpenTx project
 * (c) www.olliw.eu, OlliW, OlliW42
 */

#pragma once
#ifndef MBRIDGE_H
#define MBRIDGE_H

// -- MBridge definitions --

#define MBRIDGE_SERIALPACKET_TX_PAYLOAD_SIZE_MAX    17

#define MBRIDGE_CHANNELPACKET_SIZE  22
#define MBRIDGE_CHANNELPACKET_STX   0xFF

#define MBRIDGE_COMMANDPACKET_RX_PAYLOAD_SIZE       12
#define MBRIDGE_COMMANDPACKET_TX_PAYLOAD_SIZE       22
#define COMMANDPACKET_STX           0xA0
#define COMMANDPACKET_STX_MASK      0xE0

// -- MBridge interface --
// implemented in mavlink_telem_interface.cpp

uint32_t mBridge_cmd_available(void);
bool mBridge_cmd_get(uint8_t* cmd, uint8_t* payload, uint8_t* len);

// -- MBridge class --

class MBridge
{
  public:
    MBridge() { } // constructor

    void wakeup();

    void get_channels(uint8_t* _payload, uint8_t* len);

    struct Stats {
      int8_t rssi;
      uint8_t LQ;
      int8_t rx_rssi;
      uint8_t rx_LQ;
      uint8_t LQ_transmitted;
      uint8_t LQ_received;
      uint8_t LQ_valid_received;
    };
    struct Stats stats;

};

extern MBridge mBridge;


#endif // MBRIDGE_H
