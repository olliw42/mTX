//*******************************************************
// MavTelem Library
// Copyright (c) OlliW, OlliW42, www.olliw.eu
// LGPL3
// https://www.gnu.org/licenses/lgpl-3.0.en.html
//*******************************************************

#include "opentx.h"


extern Fifo<uint8_t, 4096> mavlinkTelemExternalTxFifo; // MissionPlanner is rude
MAVLINK_RAM_SECTION Fifo<uint8_t, 256> mBridgeTxFifo_frame; // is filled every time with just one frame
MAVLINK_RAM_SECTION Fifo<uint8_t, 256> mBridgeRxFifo_cmd; // is cleared every time so can just hold one frame


MBridge mBridge;


MBridge::MBridge()
{
  link_stats = {0};

  for (uint8_t i = 0; i < MBRIDGE_RSSI_LIST_LEN; i++) {
    link_stats.rssi1_list[i] = 127;
    link_stats.rssi2_list[i] = 127;
    link_stats.receiver_rssi_list[i] = 127;
    link_stats.receiver_antenna_list[i] = 127;
  }

  _link_stats_updated = false;

  tx_cmd_fifo.clear();
  rx_cmd_fifo.clear();
}


uint8_t MBridge::cmd_payload_len(uint8_t cmd)
{
  switch (cmd) {
  case MBRIDGE_CMD_TX_LINK_STATS: return MBRIDGE_CMD_TX_LINK_STATS_LEN;
  case MBRIDGE_CMD_DEVICE_REQUEST_ITEM: return 0;
  case MBRIDGE_CMD_DEVICE_ITEM_TX: return MBRIDGE_CMD_DEVICE_ITEM_LEN;
  case MBRIDGE_CMD_DEVICE_ITEM_RX: return MBRIDGE_CMD_DEVICE_ITEM_LEN;
  case MBRIDGE_CMD_PARAM_REQUEST_LIST: return 0;
  case MBRIDGE_CMD_PARAM_ITEM: return MBRIDGE_CMD_PARAM_ITEM_LEN;
  case MBRIDGE_CMD_PARAM_ITEM2: return MBRIDGE_CMD_PARAM_ITEM_LEN;
  case MBRIDGE_CMD_PARAM_ITEM3: return MBRIDGE_CMD_PARAM_ITEM_LEN;
  case MBRIDGE_CMD_REQUEST_CMD: return MBRIDGE_CMD_REQUEST_CMD_LEN;
  case MBRIDGE_CMD_INFO: return MBRIDGE_CMD_INFO_LEN;
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


// called in mavlinkTelemExternal_wakeup()
void MBridge::read_packet()
{
struct CmdPacket pkt;

  if (!mBridgeRxFifo_cmd.size()) return; // nothing received

  if (!cmd_get(&pkt.cmd, pkt.payload, &pkt.len)) return; // received data doesn't (yet) match

  switch (pkt.cmd) {
  case MBRIDGE_CMD_TX_LINK_STATS:
    set_linkstats((tMBridgeLinkStats*)pkt.payload);
    break;
  default:
    rx_cmd_fifo.push(pkt);
    break;
  }
}


// called in mavlinkTelemExternal_wakeup()
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


// called in mavlinkTelemExternal_wakeup()
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


// called in mavlinkTelemExternal_wakeup()
bool MBridge::send_cmdpacket(void)
{
  struct MBridge::CmdPacket pkt;
  if (!tx_cmd_fifo.pop(pkt)) return false;

  pkt.cmd &=~  MBRIDGE_COMMANDPACKET_MASK;

  // always send header, this synchronizes slave
  mBridgeTxFifo_frame.push(MBRIDGE_STX1);
  mBridgeTxFifo_frame.push(MBRIDGE_STX2);
  mBridgeTxFifo_frame.push((uint8_t)MBRIDGE_COMMANDPACKET_STX + pkt.cmd);

  // send payload
  uint8_t payload_len = cmd_payload_len(pkt.cmd);
  for (uint16_t i = 0; i < payload_len; i++) {
    mBridgeTxFifo_frame.push((i < pkt.len) ? pkt.payload[i] : 0);
  }

  return true;
}


void MBridge::set_linkstats(tMBridgeLinkStats* ls)
{
  link_stats.LQ = ls->LQ;
  link_stats.rssi1_instantaneous = (ls->receive_antenna == 0) ? ls->rssi1_instantaneous : -127;
  link_stats.rssi2_instantaneous = (ls->receive_antenna == 1) ? ls->rssi2_instantaneous : -127;
  link_stats.snr_instantaneous = ls->snr_instantaneous;
  link_stats.receive_antenna = ls->receive_antenna;
  link_stats.transmit_antenna = ls->transmit_antenna;
  link_stats.diversity = ls->diversity;
  link_stats.rssi1_filtered = ls->rssi1_filtered;
  link_stats.rssi2_filtered = ls->rssi2_filtered;
  link_stats.snr_filtered = ls->snr_filtered;

  link_stats.receiver_LQ = ls->receiver_LQ;
  link_stats.receiver_LQ_serial = ls->receiver_LQ_serial;
  link_stats.receiver_rssi_instantaneous = ls->receiver_rssi_instantaneous;
  link_stats.receiver_receive_antenna = ls->receiver_receive_antenna;
  link_stats.receiver_transmit_antenna = ls->receiver_transmit_antenna;
  link_stats.receiver_diversity = ls->receiver_diversity;
  link_stats.receiver_rssi_filtered = ls->receiver_rssi_filtered;

  link_stats.LQ_fresh_serial_packets_transmitted = ls->LQ_fresh_serial_packets_transmitted;
  link_stats.bytes_per_sec_transmitted = ls->bytes_per_sec_transmitted;
  link_stats.LQ_valid_received = ls->LQ_valid_received;
  link_stats.LQ_fresh_serial_packets_received = ls->LQ_fresh_serial_packets_received;
  link_stats.bytes_per_sec_received = ls->bytes_per_sec_received;

  link_stats.LQ_received = ls->LQ_received;

  link_stats.rx1_valid = ls->rx1_valid;
  link_stats.rx2_valid = ls->rx2_valid;
  link_stats.fhss_curr_i = ls->fhss_curr_i;
  link_stats.fhss_cnt = ls->fhss_cnt;

  link_stats.rssi1_list[ls->fhss_curr_i] = (ls->rx1_valid) ? ls->rssi1_instantaneous : 127;
  link_stats.rssi2_list[ls->fhss_curr_i] = (ls->rx2_valid) ? ls->rssi2_instantaneous : 127;
  link_stats.receiver_rssi_list[ls->fhss_curr_i] = ls->receiver_rssi_instantaneous;
  link_stats.receiver_antenna_list[ls->fhss_curr_i] = ls->receiver_receive_antenna;

  _link_stats_updated = true;
}


bool MBridge::linkstats_updated(void)
{
   if (_link_stats_updated) {
     _link_stats_updated = false;
     return true;
   }
   return false;
}


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


// mBridge: 1...1024...2047 for +-120% range
uint16_t CH11BIT(uint8_t i)
{
  int32_t v = channelOutputs[i] + 2*PPM_CH_CENTER(i) - 2*PPM_CENTER;
  v = (v * 5) / 6 + 1024; // scale +-1023 for 100% down to +-852 for 100%
  if (v < 1) return 1;
  if (v > 2047) return 2047;
  return v;
}


void MBridge::get_channels(uint8_t* _payload, uint8_t* len)
{
  tMBridgeChannelBuffer* payload = (tMBridgeChannelBuffer*)_payload;
  #define CH1BIT(i)  (uint16_t)( (channelOutputs[i] + 2*PPM_CH_CENTER(i) - 2*PPM_CENTER + 1024) >= 1536 ? 1 : 0 ) // > 50%

  payload->channel0  = CH11BIT(0); // 11 bits, 1 .. 1024 .. 2047
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



