/*
 * The MAVLink for OpenTx project
 * (c) www.olliw.eu, OlliW, OlliW42
 */

#include "opentx.h"

MBridge mBridge;

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

void MBridge::wakeup()
{

  if (mBridge_cmd_available()) {
    uint8_t cmd;
    uint8_t payload[32];
    uint8_t len;
    bool res = mBridge_cmd_get(&cmd, payload, &len);

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

