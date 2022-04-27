//*******************************************************
// The MAVLink for OpenTx project
// Copyright (c) OlliW, OlliW42, www.olliw.eu
// LGPL3
// https://www.gnu.org/licenses/lgpl-3.0.en.html
//*******************************************************

#include "opentx.h"
#include "stamp.h"

// -- CoOS RTOS mavlink task handlers --

RTOS_TASK_HANDLE mavlinkTaskId;
RTOS_DEFINE_STACK(mavlinkStack, MAVLINK_STACK_SIZE);

struct MavlinkTaskStat {
  uint16_t start = 0;
  uint16_t run = 0;
  uint16_t max = 0;
  uint16_t loop = 0;
};
struct MavlinkTaskStat mavlinkTaskStat;

uint16_t mavlinkTaskRunTime(void)
{
  return mavlinkTaskStat.run/2;
}

uint16_t mavlinkTaskRunTimeMax(void)
{
  return mavlinkTaskStat.max/2;
}

uint16_t mavlinkTaskLoop(void)
{
  return mavlinkTaskStat.loop/2;
}

TASK_FUNCTION(mavlinkTask)
{
  while (true) {
    uint16_t start_last = mavlinkTaskStat.start;
    mavlinkTaskStat.start = getTmr2MHz();

    mavlinkTelem.wakeup();

    mavlinkTaskStat.run = getTmr2MHz() - mavlinkTaskStat.start;
    if (mavlinkTaskStat.run > mavlinkTaskStat.max) mavlinkTaskStat.max = mavlinkTaskStat.run;
    mavlinkTaskStat.loop = (mavlinkTaskStat.start - start_last);

    RTOS_WAIT_TICKS(2);
  }
}

void mavlinkStart()
{
  RTOS_CREATE_TASK(mavlinkTaskId, mavlinkTask, "mavlink", mavlinkStack, MAVLINK_STACK_SIZE, MAVLINK_TASK_PRIO);
}

// -- EXTERNAL BAY SERIAL handlers --
// we essentially redo everything from scratch
// it is a bit of a tricky thing since we use the telemetry uart and not the module uart
// TxFifo & RxFifo forms the 'serial' interface
// TxFifo_frame is filled from TxFifo every 2ms, and is what is pushed out in Tx ISR
// RXFifo is continuously filled in Rx ISR
// if as command is detected, it is filled into RxFifo_cmd, prepended with 'OW'

MAVLINK_RAM_SECTION Fifo<uint8_t, 4096> mavlinkMBridgeTxFifo; // MissionPlanner is rude
MAVLINK_RAM_SECTION Fifo<uint8_t, 4096> mavlinkMBridgeRxFifo;

void extmoduleMBridgeStop(void)
{
  USART_ITConfig(TELEMETRY_USART, USART_IT_RXNE, DISABLE);
  USART_ITConfig(TELEMETRY_USART, USART_IT_TXE, DISABLE);
  NVIC_DisableIRQ(TELEMETRY_USART_IRQn);

  NVIC_DisableIRQ(TELEMETRY_EXTI_IRQn);
  NVIC_DisableIRQ(TELEMETRY_TIMER_IRQn);
  DMA_ITConfig(TELEMETRY_DMA_Stream_TX, DMA_IT_TC, DISABLE);
  NVIC_DisableIRQ(TELEMETRY_DMA_TX_Stream_IRQ);

  USART_DeInit(TELEMETRY_USART);
  DMA_DeInit(TELEMETRY_DMA_Stream_TX);

  //EXTERNAL_MODULE_OFF();
}

void extmoduleMBridgeStart(void)
{
  //EXTERNAL_MODULE_ON();

  // we don't want or need all this
  NVIC_DisableIRQ(TELEMETRY_EXTI_IRQn);
  NVIC_DisableIRQ(TELEMETRY_TIMER_IRQn);
  NVIC_DisableIRQ(TELEMETRY_DMA_TX_Stream_IRQ);

  DMA_ITConfig(TELEMETRY_DMA_Stream_TX, DMA_IT_TC, DISABLE);
  DMA_Cmd(TELEMETRY_DMA_Stream_TX, DISABLE);
  USART_DMACmd(TELEMETRY_USART, USART_DMAReq_Tx, DISABLE);
  DMA_DeInit(TELEMETRY_DMA_Stream_TX);

  EXTI_InitTypeDef EXTI_InitStructure;
  EXTI_StructInit(&EXTI_InitStructure);
  EXTI_InitStructure.EXTI_Line = TELEMETRY_EXTI_LINE;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = TELEMETRY_EXTI_TRIGGER;
  EXTI_InitStructure.EXTI_LineCmd = DISABLE;
  EXTI_Init(&EXTI_InitStructure);

  // is it called already? through telemetryInit() -> telemetryPortInit(FRSKY_SPORT_BAUDRATE) -> telemetryInitDirPin()
  GPIO_PinAFConfig(TELEMETRY_GPIO, TELEMETRY_GPIO_PinSource_RX, TELEMETRY_GPIO_AF);
  GPIO_PinAFConfig(TELEMETRY_GPIO, TELEMETRY_GPIO_PinSource_TX, TELEMETRY_GPIO_AF);

  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = TELEMETRY_TX_GPIO_PIN | TELEMETRY_RX_GPIO_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(TELEMETRY_GPIO, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin   = TELEMETRY_DIR_GPIO_PIN;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(TELEMETRY_DIR_GPIO, &GPIO_InitStructure);
  GPIO_ResetBits(TELEMETRY_DIR_GPIO, TELEMETRY_DIR_GPIO_PIN);

  // init uart itself
  USART_InitTypeDef USART_InitStructure;
  USART_InitStructure.USART_BaudRate = 400000;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
  USART_Init(TELEMETRY_USART, &USART_InitStructure);

  USART_Cmd(TELEMETRY_USART, ENABLE);
  USART_ITConfig(TELEMETRY_USART, USART_IT_RXNE, ENABLE);
  USART_ITConfig(TELEMETRY_USART, USART_IT_TXE, DISABLE);
  NVIC_SetPriority(TELEMETRY_USART_IRQn, 5); // everything else seems to be 6,7,8,10,11 some are 0
  NVIC_EnableIRQ(TELEMETRY_USART_IRQn);
}

// this must be called regularly, at 2 ms
// 115200 bps = 86 us per byte => 12 bytes per ms = 24 bytes per 2 ms
// 3+24 bytes @ 400000 bps = 0.675 ms, 24 bytes @ 400000 bps = 0.6 ms => 1.275 ms
// => enough time for a tx and rx packet in a 2 ms slot
// however, the slots are not precisely fixed to 2 ms, can be shorter
// so design for lower data rate, we send at most 16 bytes per slot
// 16 bytes per slot = 8000 bytes/s = effectively 80000 bps, should be way enough
// 3+16 bytes @ 400000 bps = 0.475 ms, 16 bytes @ 400000 bps = 0.4 ms, => 0.875 ms

void mavlinkTelemExternal_wakeup(void)
{
  static uint8_t slot_counter = 0;

  mBridge.read_packet();

  // we do it at the beginning, so it gives few cycles before TX is enabled
  TELEMETRY_DIR_GPIO->BSRRL = TELEMETRY_DIR_GPIO_PIN; // enable output
  TELEMETRY_USART->CR1 &= ~USART_CR1_RE; // turn off receiver

  // every 10th slot we send a channel packet
  if (slot_counter == 0) {
    mBridge.send_channelpacket();
  } else
  if ((slot_counter == 1) || (slot_counter == 5)) {
    if (!mBridge.send_cmdpacket()) mBridge.send_serialpacket(); // if we have a cmd packet, send it, else send a serial packet
  } else {
    mBridge.send_serialpacket();
  }

  USART_ITConfig(TELEMETRY_USART, USART_IT_TXE, ENABLE); // enable TX interrupt, starts sending

  slot_counter++;
  if (slot_counter >= 10) slot_counter = 0;

  // we received a linkstats cmd packet
  if (mBridge.linkstats_updated()) {
    mavlinkTelem.mbridgestats.is_receiving_linkstats = MAVLINK_TELEM_RADIO_RECEIVING_TIMEOUT;
    uint8_t rssi = 0;
    int8_t rssi_i8 = mBridge.link_stats.receiver_rssi_instantaneous;
    // convert -127 .. 0 to 0 ... 254
    if (rssi_i8 == 127) {
      rssi = UINT8_MAX;
    }
    else if (rssi_i8 > -50) {
      rssi = 254;
    }
    else if (rssi_i8 < -120) {
      rssi = 0;
    } else {
      int32_t r = (int32_t)rssi_i8 - (-120);
      constexpr int32_t m = (int32_t)(-50) - (-120);
      rssi = (r * 254 + m/2) / m;
    }
    mavlinkTelem.mbridgestats.receiver_rssi = rssi;

    if (g_model.mavlinkRssiScale > 0) {
      if (g_model.mavlinkRssiScale < 255) { //if not full range, respect  UINT8_MAX
        if (rssi == UINT8_MAX) rssi = 0;
      }
      if (rssi > g_model.mavlinkRssiScale) rssi = g_model.mavlinkRssiScale; //constrain
      rssi = (uint8_t)( ((uint16_t)rssi * 100) / g_model.mavlinkRssiScale); //scale to 0..100
    }
    else { //mavlink default
      if (rssi == UINT8_MAX) rssi = 0;
    }
    mavlinkTelem.mbridgestats.receiver_rssi_scaled = rssi;

    mavlinkTelem.mbridgestats.receiver_LQ = mBridge.link_stats.receiver_LQ;
  }
}

uint32_t mavlinkTelemExternalAvailable(void)
{
  return mavlinkMBridgeRxFifo.size();
}

uint8_t mavlinkTelemExternalGetc(uint8_t* c)
{
  return mavlinkMBridgeRxFifo.pop(*c);
}

bool mavlinkTelemExternalHasSpace(uint16_t count)
{
  return mavlinkMBridgeTxFifo.hasSpace(count);
}

bool mavlinkTelemExternalPutBuf(const uint8_t *buf, const uint16_t count)
{
  if (!mavlinkMBridgeTxFifo.hasSpace(count)) return false;
  for (uint16_t i = 0; i < count; i++) mavlinkMBridgeTxFifo.push(buf[i]);
  return true;
}

// -- AUX1, AUX2 handlers --

uint32_t _cvtBaudrate(uint16_t baud)
{
  switch (baud) {
    case 0: return 57600;
    case 1: return 115200;
    case 2: return 38400;
    case 3: return 19200;
  }
  return 57600;
}

uint32_t mavlinkTelemAuxBaudrate(void)
{
  return _cvtBaudrate(g_eeGeneral.mavlinkBaudrate);
}

uint32_t mavlinkTelemAux2Baudrate(void)
{
  return _cvtBaudrate(g_eeGeneral.mavlinkBaudrate2);
}

#if defined(TELEMETRY_MAVLINK_USB_SERIAL)
MAVLINK_RAM_SECTION Fifo<uint8_t, 4096> mavlinkTelemUsbRxFifo; // MissionPlanner is rude
#endif

#if defined(AUX_SERIAL)

uint32_t mavlinkTelem1Available(void)
{
  if (!mavlinkTelem.serial1_enabled) return 0;
  if (mavlinkTelem.serial1_isexternal) return mavlinkMBridgeRxFifo.size();

//  if (auxSerialMode != UART_MODE_MAVLINK) return 0;
  return auxSerialRxFifo.size();
}

// call only after check with mavlinkTelem2Available()
uint8_t mavlinkTelem1Getc(uint8_t* c)
{
  if (!mavlinkTelem.serial1_enabled) return 0;
  if (mavlinkTelem.serial1_isexternal) return mavlinkMBridgeRxFifo.pop(*c);

  return auxSerialRxFifo.pop(*c);
}

bool mavlinkTelem1HasSpace(uint16_t count)
{
  if (!mavlinkTelem.serial1_enabled) return 0;
  if (mavlinkTelem.serial1_isexternal) return mavlinkMBridgeTxFifo.hasSpace(count);

//  if (auxSerialMode != UART_MODE_MAVLINK) return false;
  return auxSerialTxFifo.hasSpace(count);
}

bool mavlinkTelem1PutBuf(const uint8_t* buf, const uint16_t count)
{
  if (!mavlinkTelem.serial1_enabled || !buf) return false;
  if (mavlinkTelem.serial1_isexternal) return mavlinkTelemExternalPutBuf(buf, count);

  if (!auxSerialTxFifo.hasSpace(count)) return false;
//  if (auxSerialMode != UART_MODE_MAVLINK || !buf || !auxSerialTxFifo.hasSpace(count)) {
//    return false;
//  }
  for (uint16_t i = 0; i < count; i++) auxSerialTxFifo.push(buf[i]);
  USART_ITConfig(AUX_SERIAL_USART, USART_IT_TXE, ENABLE);
  return true;
}

#else
uint32_t mavlinkTelem1Available(void){ return 0; }
uint8_t mavlinkTelem1Getc(uint8_t* c){ return 0; }
bool mavlinkTelem1HasSpace(uint16_t count){ return false; }
bool mavlinkTelem1PutBuf(const uint8_t* buf, const uint16_t count){ return false; }
#endif

#if defined(AUX2_SERIAL)

uint32_t mavlinkTelem2Available(void)
{
  if (!mavlinkTelem.serial2_enabled) return 0;
  if (mavlinkTelem.serial2_isexternal) return mavlinkMBridgeRxFifo.size();

//  if (aux2SerialMode != UART_MODE_MAVLINK) return 0;
  return aux2SerialRxFifo.size();
}

// call only after check with mavlinkTelem2Available()
uint8_t mavlinkTelem2Getc(uint8_t* c)
{
  if (!mavlinkTelem.serial2_enabled) return 0;
  if (mavlinkTelem.serial2_isexternal) return mavlinkMBridgeRxFifo.pop(*c);

  return aux2SerialRxFifo.pop(*c);
}

bool mavlinkTelem2HasSpace(uint16_t count)
{
  if (!mavlinkTelem.serial2_enabled) return 0;
  if (mavlinkTelem.serial2_isexternal) return mavlinkMBridgeTxFifo.hasSpace(count);

//  if (aux2SerialMode != UART_MODE_MAVLINK) return false;
  return aux2SerialTxFifo.hasSpace(count);
}

bool mavlinkTelem2PutBuf(const uint8_t* buf, const uint16_t count)
{
  if (!mavlinkTelem.serial2_enabled || !buf) return false;
  if (mavlinkTelem.serial2_isexternal) return mavlinkTelemExternalPutBuf(buf, count);

  if (!aux2SerialTxFifo.hasSpace(count)) return false;
//  if (aux2SerialMode != UART_MODE_MAVLINK || !buf || !aux2SerialTxFifo.hasSpace(count)) {
//    return false;
//  }
  for (uint16_t i = 0; i < count; i++) aux2SerialTxFifo.push(buf[i]);
  USART_ITConfig(AUX2_SERIAL_USART, USART_IT_TXE, ENABLE);
  return true;
}

#else
uint32_t mavlinkTelem2Available(void){ return 0; }
uint8_t mavlinkTelem2Getc(uint8_t* c){ return 0; }
bool mavlinkTelem2HasSpace(uint16_t count){ return false; }
bool mavlinkTelem2PutBuf(const uint8_t* buf, const uint16_t count){ return false; }
#endif

// -- USB handlers --

#if defined(TELEMETRY_MAVLINK_USB_SERIAL)

uint32_t mavlinkTelem3Available(void)
{
  if (getSelectedUsbMode() != USB_MAVLINK_MODE) return 0;
  return mavlinkTelemUsbRxFifo.size();
}

// call only after check with mavlinkTelem2Available()
uint8_t mavlinkTelem3Getc(uint8_t* c)
{
  return mavlinkTelemUsbRxFifo.pop(*c);
}

bool mavlinkTelem3HasSpace(uint16_t count)
{
  if (getSelectedUsbMode() != USB_MAVLINK_MODE) return false;
  return true; //??
}

bool mavlinkTelem3PutBuf(const uint8_t* buf, const uint16_t count)
{
  if (getSelectedUsbMode() != USB_MAVLINK_MODE || !buf) {
    return false;
  }
  for (uint16_t i = 0; i < count; i++) {
    usbSerialPutc(buf[i]);
  }
  return true;
}

#else
uint32_t mavlinkTelem3Available(void){ return 0; }
uint8_t mavlinkTelem3Getc(uint8_t* c){ return 0; }
bool mavlinkTelem3HasSpace(uint16_t count){ return false; }
bool mavlinkTelem3PutBuf(const uint8_t* buf, const uint16_t count){ return false; }
#endif

// -- more Interface helpers --

uint8_t fmav_router_time_100ms(void)
{
   return get_tmr10ms() / 10;
}

// -- MavlinkTelem handlers --

uint32_t mavlinkRcOverrideRate(void)
{
  switch (g_model.mavlinkRcOverride) {
    case 1:
    case 2:
    case 3:
    case 4:
    case 5:
      return 100 / g_model.mavlinkRcOverride; // 1, 2, 3, 4, 5 Hz => 100 / val;
    case 6:
      return 13; // 7.7 Hz
    case 7:
      return 10; // 10 Hz
    case 8:
    case 9:
    case 10:
    case 11:
    case 12:
    case 13:
    case 14:
      return 16 - g_model.mavlinkRcOverride; // 12.5, 14.3, 16.7, 20, 25, 33.3, 50 Hz => 16 - val;
  }
  return 10; // 100 ms = 10 Hz = default
}

const uint32_t MavlinkTelem::version(void)
{
 return OWVERSION;
}

const char* MavlinkTelem::versionstr(void)
{
  return OWVERSIONSTR;
}

const char* MavlinkTelem::banner(void)
{
  return "OpenTx with MAVLink " VERSION " " OWVERSIONSTR;
}

uint32_t MavlinkTelem::getTime_10ms(void)
{
  return get_tmr10ms();
}

uint32_t MavlinkTelem::getTime_10us(void) // ca 11.9h, should be sufficient
{
  uint16_t t2MHz_now = getTmr2MHz();
  _t10us_last += (t2MHz_now - _t2MHz_last);
  _t2MHz_last = t2MHz_now;
  return _t10us_last/20;
}

void MavlinkTelem::telemetrySetValue(uint16_t id, uint8_t subId, uint8_t instance, int32_t value, uint32_t unit, uint32_t prec)
{
  if (g_model.mavlinkRssi) {
    if (!radio.is_receiving && !radio.is_receiving65 && !radio.is_receiving35) return;
  }

  if (g_model.mavlinkMimicSensors) {
    setTelemetryValue(PROTOCOL_TELEMETRY_FRSKY_SPORT, id, subId, instance, value, unit, prec);
    telemetryStreaming = MAVLINK_TELEM_RADIO_RECEIVING_TIMEOUT;
  }
}

// only for MAVLINK_MSG_ID_RADIO_STATUS, MAVLINK_MSG_ID_RC_CHANNELS, MAVLINK_MSG_ID_RC_CHANNELS_RAW
void MavlinkTelem::telemetrySetRssiValue(uint8_t rssi)
{
  if (g_model.mavlinkRssiScale > 0) {
    if (g_model.mavlinkRssiScale < 255) { //if not full range, respect  UINT8_MAX
      if (rssi == UINT8_MAX) rssi = 0;
    }
    if (rssi > g_model.mavlinkRssiScale) rssi = g_model.mavlinkRssiScale; //constrain
    rssi = (uint8_t)( ((uint16_t)rssi * 100) / g_model.mavlinkRssiScale); //scale to 0..100
  }
  else { //mavlink default
    if (rssi == UINT8_MAX) rssi = 0;
  }

  radio.rssi_scaled = rssi;

  if (g_model.mavlinkRssi) {
    if (!radio.is_receiving && !radio.is_receiving65 && !radio.is_receiving35) return;

    telemetryData.rssi.set(rssi);
    telemetryStreaming = MAVLINK_TELEM_RADIO_RECEIVING_TIMEOUT;
  }

  if (g_model.mavlinkMimicSensors) {
    setTelemetryValue(PROTOCOL_TELEMETRY_FRSKY_SPORT, RSSI_ID, 0, 1, (int32_t)rssi, UNIT_DB, 0);
    telemetryStreaming = MAVLINK_TELEM_RADIO_RECEIVING_TIMEOUT;
  }
  //#if defined(MULTIMODULE)
  //{ TX_RSSI_ID, TX_RSSI_ID, 0, ZSTR_TX_RSSI   , UNIT_DB , 0 },
  //{ TX_LQI_ID , TX_LQI_ID,  0, ZSTR_TX_QUALITY, UNIT_RAW, 0 },
}

// is probably not needed, aren't they reset by telementryStreaming timeout?
void MavlinkTelem::telemetryResetRssiValue(void)
{
  if (radio.is_receiving || radio.is_receiving65 || radio.is_receiving35) return;

  radio.rssi_scaled = 0;

  if (g_model.mavlinkRssi) {
    telemetryData.rssi.reset();
  }

  if (g_model.mavlinkMimicSensors) {
    setTelemetryValue(PROTOCOL_TELEMETRY_FRSKY_SPORT, RSSI_ID, 0, 1, 0, UNIT_DB, 0);
  }
}

/* legacy, keep to remember what we did bool MavlinkTelem::telemetryVoiceEnabled(void)
{
  if (!g_model.mavlinkRssi && !g_model.mavlinkMimicSensors) return true;

  if (g_model.mavlinkRssi && !radio.rssi_voice_disabled) return true;

  return false;
} */

bool MavlinkTelem::telemetryVoiceCriticalDisabled(void)
{
  if (radio.rssi_voice_critical_disabled) return true;
  return false;
}

bool MavlinkTelem::telemetryVoiceTelemetryOkDisabled(void)
{
  if (radio.rssi_voice_telemetryok_disabled) return true;
  return false;
}


