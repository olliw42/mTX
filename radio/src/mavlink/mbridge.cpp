//*******************************************************
// MavTelem Library
// Copyright (c) OlliW, OlliW42, www.olliw.eu
// LGPL3
// https://www.gnu.org/licenses/lgpl-3.0.en.html
//*******************************************************

#include "opentx.h"


extern Fifo<uint8_t, 1024> mavlinkTelemExternalTxFifo;
MAVLINK_RAM_SECTION Fifo<uint8_t, 32> mBridgeTxFifo_frame;
MAVLINK_RAM_SECTION Fifo<uint8_t, 256> mBridgeRxFifo_cmd;


MBridge mBridge;


uint8_t MBridge::cmd_payload_len(uint8_t cmd)
{
  switch (cmd) {
  case MBRIDGE_CMD_TX_LINK_STATS: return MBRIDGE_CMD_TX_LINK_STATS_LEN;
  }
  return 0;
}


bool MBridge::cmd_get(uint8_t* cmd, uint8_t* payload, uint8_t* len)
{
  *len = 0;
  uint8_t c = 0;
  uint8_t state = 0;
  uint8_t payload_len = 0;
  while (mBridgeRxFifo_cmd.pop(c)) {
    switch (state) {
    case 0:
      if (c == MBRIDGE_STX1) state = 1;
      break;
    case 1:
      if (c == MBRIDGE_STX2) state = 2;
      break;
    case 2:
      *cmd = c & (~MBRIDGE_COMMANDPACKET_MASK);
      payload_len = cmd_payload_len(*cmd);
      state = 3;
      break;
    case 3:
      payload[*len] = c;
      (*len)++;
      if (*len >= payload_len) return true; // end of packet reached
      break;
    }
    if (*len >= MBRIDGE_M2R_COMMAND_PAYLOAD_LEN_MAX) return false; // should not happen, but play it safe
  }
  return false;
}


void MBridge::read_in()
{
uint8_t cmd;
uint8_t payload[MBRIDGE_M2R_COMMAND_PAYLOAD_LEN_MAX];
uint8_t len;

  if (!mBridgeRxFifo_cmd.size()) return; // nothing received

  if (!cmd_get(&cmd, payload, &len)) return; // received frame doesn't match

  switch (cmd) {
  case MBRIDGE_CMD_TX_LINK_STATS:
    set_linkstats((tMBridgeLinkStats*)payload);
    break;
  }
}


void MBridge::set_linkstats(tMBridgeLinkStats* ls)
{
  link_stats.LQ = ls->LQ;
  link_stats.rssi_instantaneous = ls->rssi_instantaneous;
  link_stats.snr_instantaneous = ls->snr_instantaneous;
  link_stats.rssi2_instantaneous = ls->rssi2_instantaneous;
  link_stats.ant_no = ls->ant_no;
  link_stats.rssi_filtered = ls->rssi_filtered;
  link_stats.snr_filtered = ls->snr_filtered;
  link_stats.rssi2_filtered = ls->rssi2_filtered;

  link_stats.receiver_LQ = ls->receiver_LQ;
  link_stats.receiver_LQ_serial = ls->receiver_LQ_serial;
  link_stats.receiver_rssi_instantaneous = ls->receiver_rssi_instantaneous;
  link_stats.receiver_snr_instantaneous = ls->receiver_snr_instantaneous;
  link_stats.receiver_rssi2_instantaneous = ls->receiver_rssi2_instantaneous;
  link_stats.receiver_ant_no = ls->receiver_ant_no;
  link_stats.receiver_rssi_filtered = ls->receiver_rssi_filtered;
  link_stats.receiver_snr_filtered = ls->receiver_snr_filtered;
  link_stats.receiver_rssi2_filtered = ls->receiver_rssi2_filtered;

  link_stats.LQ_fresh_serial_packets_transmitted = ls->LQ_fresh_serial_packets_transmitted;
  link_stats.bytes_per_sec_transmitted = ls->bytes_per_sec_transmitted;
  link_stats.LQ_valid_received = ls->LQ_valid_received;
  link_stats.LQ_fresh_serial_packets_received = ls->LQ_fresh_serial_packets_received;
  link_stats.bytes_per_sec_received = ls->bytes_per_sec_received;

  link_stats.LQ_received = ls->LQ_received;
}


void MBridge::send_serialpacket(void)
{
  uint32_t count = mavlinkTelemExternalTxFifo.size();
  if (count > MBRIDGE_R2M_SERIAL_PAYLOAD_LEN_MAX) count = MBRIDGE_R2M_SERIAL_PAYLOAD_LEN_MAX;

  // always send header, this synchronizes slave
  mBridgeTxFifo_frame.push(MBRIDGE_STX1);
  mBridgeTxFifo_frame.push(MBRIDGE_STX2);
  mBridgeTxFifo_frame.push((uint8_t)count);

  // send payload
  for (uint16_t i = 0; i < count; i++) {
    uint8_t c = '\0';
    mavlinkTelemExternalTxFifo.pop(c);
    mBridgeTxFifo_frame.push(c);
  }
}


void MBridge::send_channelpacket(void)
{
 uint8_t payload[MBRIDGE_CHANNELPACKET_SIZE];
 uint8_t len;

  get_channels(payload, &len);

  // always send header, this synchronizes slave
  mBridgeTxFifo_frame.push(MBRIDGE_STX1);
  mBridgeTxFifo_frame.push(MBRIDGE_STX2);
  mBridgeTxFifo_frame.push((uint8_t)MBRIDGE_CHANNELPACKET_STX); // marker for channel packet

  // send payload
  for (uint16_t i = 0; i < MBRIDGE_CHANNELPACKET_SIZE; i++) {
    mBridgeTxFifo_frame.push(payload[i]);
  }
}


/* not yet used
void MBridge::send_cmdpacket(uint8_t cmd, uint8_t* payload, uint8_t len)
{
  // always send header, this synchronizes slave
  mBridgeTxFifo_frame.push('O');
  mBridgeTxFifo_frame.push('W');
  mBridgeTxFifo_frame.push((uint8_t)MBRIDGE_COMMANDPACKET_STX + (cmd &~ MBRIDGE_COMMANDPACKET_STX_MASK));

  // send payload
  for (uint16_t i = 0; i < MBRIDGE_R2M_COMMAND_PAYLOAD_LEN; i++) {
    mBridgeTxFifo_frame.push((i < len) ? payload[i] : 0);
  }
}
*/


typedef union {
  uint8_t c[MBRIDGE_CHANNELPACKET_SIZE]; // 176 + 8 = 184 bits = 23 bytes
  struct {
    uint16_t channel0  : 11; // 16 channels a 11 bits per channel = 176 bits
    uint16_t channel1  : 11;
    uint16_t channel2  : 11;
    uint16_t channel3  : 11;
    uint16_t channel4  : 11;
    uint16_t channel5  : 11;
    uint16_t channel6  : 11;
    uint16_t channel7  : 11;
    uint16_t channel8  : 11;
    uint16_t channel9  : 11;
    uint16_t channel10 : 11;
    uint16_t channel11 : 11;
    uint16_t channel12 : 11;
    uint16_t channel13 : 11;
    uint16_t channel14 : 11;
    uint16_t channel15 : 11;
    uint8_t channel16 : 1; // 2 channels a 1 bit per channel = 2 bits
    uint8_t channel17 : 1;
  } __attribute__ ((__packed__));
} tMBridgeChannelBuffer;


uint16_t CH11BIT(uint8_t i)
{
  int16_t v = channelOutputs[i] + 2*PPM_CH_CENTER(i) - 2*PPM_CENTER + 1024;
  if (v < 0) return 0;
  if (v > 2047) return 2047;
  return v;
}


void MBridge::get_channels(uint8_t* _payload, uint8_t* len)
{
  tMBridgeChannelBuffer* payload = (tMBridgeChannelBuffer*)_payload;
  #define CH1BIT(i)  (uint16_t)( (channelOutputs[i] + 2*PPM_CH_CENTER(i) - 2*PPM_CENTER + 1024) >= 1536 ? 1 : 0 )

  payload->channel0  = CH11BIT(0); // 11 bits, 0 .. 1024 .. 2047
  payload->channel1  = CH11BIT(1);
  payload->channel2  = CH11BIT(2);
  payload->channel3  = CH11BIT(3);
  payload->channel4  = CH11BIT(4);
  payload->channel5  = CH11BIT(5);
  payload->channel6  = CH11BIT(6);
  payload->channel7  = CH11BIT(7);
  payload->channel8  = CH11BIT(8);
  payload->channel9  = CH11BIT(9);
  payload->channel10 = CH11BIT(10);
  payload->channel11 = CH11BIT(11);
  payload->channel12 = CH11BIT(12);
  payload->channel13 = CH11BIT(13);
  payload->channel14 = CH11BIT(14);
  payload->channel15 = CH11BIT(15);
  payload->channel16 = CH1BIT(16); // 1 bit, 0..1
  payload->channel17 = CH1BIT(17);

  *len = MBRIDGE_CHANNELPACKET_SIZE;
}



