/*
 * The MAVLink for OpenTx project
 * (c) www.olliw.eu, OlliW, OlliW42
 */

#include "opentx.h"


extern Fifo<uint8_t, 1024> mavlinkTelemExternalTxFifo;
MAVLINK_RAM_SECTION Fifo<uint8_t, 32> mBridgeTxFifo_frame;
MAVLINK_RAM_SECTION Fifo<uint8_t, 256> mBridgeRxFifo_cmd;

MBridge mBridge;

uint32_t MBridge::cmd_available(void)
{
  return mBridgeRxFifo_cmd.size();
}

bool MBridge::cmd_get(uint8_t* cmd, uint8_t* payload, uint8_t* len)
{
  *len = 0;
  uint8_t c = 0;
  uint8_t state = 0;
  while (mBridgeRxFifo_cmd.pop(c)) {
    switch (state) {
    case 0:
      if (c == 'O') state = 1;
      break;
    case 1:
      if (c == 'W') state = 2;
      break;
    case 2:
      *cmd = c & 0x1F;
      state = 3;
      break;
    case 3:
      payload[*len] = c;
      (*len)++;
      break;
    }
    if (*len >= MBRIDGE_COMMANDPACKET_RX_PAYLOAD_SIZE) break; // end of packet reached
  }
  return (state < 3) ? false : true;
}

void MBridge::wakeup()
{

  if (cmd_available()) {
    uint8_t cmd;
    uint8_t payload[32];
    uint8_t len;
    bool res = cmd_get(&cmd, payload, &len);

    if (res && (cmd == 0x01) && (len >= 7)) {
      stats.rssi = -(payload[0]);
      stats.LQ = payload[1];
      stats.rx_rssi = -(payload[2]);
      stats.rx_LQ = payload[3];
      stats.LQ_transmitted = payload[4];
      stats.LQ_received = payload[5];
      stats.LQ_valid_received = payload[6];
    }
  }

}

void MBridge::send_serialpacket(void)
{
  uint32_t count = mavlinkTelemExternalTxFifo.size();
  if (count > MBRIDGE_SERIALPACKET_TX_PAYLOAD_SIZE_MAX) count = MBRIDGE_SERIALPACKET_TX_PAYLOAD_SIZE_MAX;

  // always send header, this synchronizes slave
  mBridgeTxFifo_frame.push('O');
  mBridgeTxFifo_frame.push('W');
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
  mBridgeTxFifo_frame.push('O');
  mBridgeTxFifo_frame.push('W');
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
  mBridgeTxFifo_frame.push((uint8_t)COMMANDPACKET_STX + (cmd &~ COMMANDPACKET_STX_MASK));

  // send payload
  for (uint16_t i = 0; i < MBRIDGE_COMMANDPACKET_TX_PAYLOAD_SIZE; i++) {
    mBridgeTxFifo_frame.push((i < len) ? payload[i] : 0);
  }
}
*/

typedef union {
  uint8_t c[MBRIDGE_CHANNELPACKET_SIZE]; // 154 + 20 + 2 = 176 bits = 22 bytes
  struct {
    uint16_t channel0  : 11; // 14 channels a 11 bits per channel = 154 bits
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
    uint16_t channel14 : 10; // 2 channels a 10 bits per channel = 20 bits
    uint16_t channel15 : 10;
    uint16_t channel16 : 1; // 2 channels a 1 bit per channel = 2 bits
    uint16_t channel17 : 1;
  } __attribute__ ((__packed__));
} tMBridgeChannelBuffer;

uint16_t CH11BIT(uint8_t i)
{
  int16_t v = channelOutputs[i] + 2*PPM_CH_CENTER(i) - 2*PPM_CENTER + 1024;
  if (v < 0) return 0;
  if (v > 2047) return 2047;
  return v;
}

uint16_t CH10BIT(uint8_t i)
{
  int16_t v = (channelOutputs[i] + 2*PPM_CH_CENTER(i) - 2*PPM_CENTER + 1024) / 2;
  if (v < 0) return 0;
  if (v > 1023) return 1023;
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
  payload->channel14 = CH10BIT(14); // 10 bits, 0 .. 512 .. 1023
  payload->channel15 = CH10BIT(15);
  payload->channel16 = CH1BIT(16); // 1 bit, 0..1
  payload->channel17 = CH1BIT(17);

  *len = MBRIDGE_CHANNELPACKET_SIZE;
}



