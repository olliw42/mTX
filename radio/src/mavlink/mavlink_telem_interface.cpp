/*
 * The MAVLink for OpenTx project
 * (c) www.olliw.eu, OlliW, OlliW42
 */

#include "opentx.h"

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

MAVLINK_RAM_SECTION Fifo<uint8_t, 32> mavlinkTelemExternalTxFifo_frame;
MAVLINK_RAM_SECTION Fifo<uint8_t, 1024> mavlinkTelemExternalTxFifo;
MAVLINK_RAM_SECTION Fifo<uint8_t, 1024> mavlinkTelemExternalRxFifo;

void extmoduleMavlinkTelemStop(void)
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

void extmoduleMavlinkTelemStart(void)
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
  NVIC_SetPriority(TELEMETRY_USART_IRQn, 6);
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

#define MAVLINKPACKET_SIZE 17
#define CHANNELPACKET_SIZE 22
#define CHANNELPACKET_STX  0xFF

inline void mavlinkTelemExternal_send_mavlinkpacket(void)
{
  uint32_t count = mavlinkTelemExternalTxFifo.size();
  if (count > MAVLINKPACKET_SIZE) count = MAVLINKPACKET_SIZE;

  // always send header, this synchronizes slave
  mavlinkTelemExternalTxFifo_frame.push('O');
  mavlinkTelemExternalTxFifo_frame.push('W');
  mavlinkTelemExternalTxFifo_frame.push((uint8_t)count);

  // send payload
  for (uint16_t i = 0; i < count; i++) {
    uint8_t c;
    mavlinkTelemExternalTxFifo.pop(c);
    mavlinkTelemExternalTxFifo_frame.push(c);
  }
}

typedef union {
  uint8_t c[CHANNELPACKET_SIZE];
  struct {
    uint16_t channel0  : 11; // 11 bits per channel * 16 channels = 22 bytes
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
  } __attribute__ ((__packed__));
} tChannelBuffer;

inline void mavlinkTelemExternal_send_channelpacket(void)
{
  // always send header, this synchronizes slave
  mavlinkTelemExternalTxFifo_frame.push('O');
  mavlinkTelemExternalTxFifo_frame.push('W');
  mavlinkTelemExternalTxFifo_frame.push((uint8_t)CHANNELPACKET_STX); // marker for channel packet

  // prepare payload
  // do not confuse with sbus, it is sbus packet format, but not sbus values
  // we center the values, which are in range -2048...2047
  tChannelBuffer payload;
  #define CH(i) (uint16_t)( (channelOutputs[i] + 2*PPM_CH_CENTER(i) - 2*PPM_CENTER + 2048)/2 )
  payload.channel0 = CH(0);
  payload.channel1 = CH(1);
  payload.channel2 = CH(2);
  payload.channel3 = CH(3);
  payload.channel4 = CH(4);
  payload.channel5 = CH(5);
  payload.channel6 = CH(6);
  payload.channel7 = CH(7);
  payload.channel8 = CH(8);
  payload.channel9 = CH(9);
  payload.channel10 = CH(10);
  payload.channel11 = CH(11);
  payload.channel12 = CH(12);
  payload.channel13 = CH(13);
  payload.channel14 = CH(14);
  payload.channel15 = CH(15);

  // send payload
  for (uint16_t i = 0; i < CHANNELPACKET_SIZE; i++) {
    mavlinkTelemExternalTxFifo_frame.push(payload.c[i]);
  }
}

void mavlinkTelemExternal_wakeup(void)
{
  static uint8_t slot_counter = 0;

  // we do it at the beginning, so it gives few cycles before TX is enabled
  TELEMETRY_DIR_GPIO->BSRRL = TELEMETRY_DIR_GPIO_PIN; // enable output
  TELEMETRY_USART->CR1 &= ~USART_CR1_RE; // turn off receiver

  // every 10th slot we send a channel packet
  if (slot_counter == 0)
    mavlinkTelemExternal_send_channelpacket();
  else
    mavlinkTelemExternal_send_mavlinkpacket();

  USART_ITConfig(TELEMETRY_USART, USART_IT_TXE, ENABLE); // enable TX interrupt, starts sending

  slot_counter++;
  if (slot_counter >= 10) slot_counter = 0;
}

uint32_t mavlinkTelemExternalAvailable(void)
{
  return mavlinkTelemExternalRxFifo.size();
}

uint8_t mavlinkTelemExternalGetc(uint8_t* c)
{
  return mavlinkTelemExternalRxFifo.pop(*c);
}

bool mavlinkTelemExternalHasSpace(uint16_t count)
{
  return mavlinkTelemExternalTxFifo.hasSpace(count);
}

bool mavlinkTelemExternalPutBuf(const uint8_t *buf, const uint16_t count)
{
  if (!mavlinkTelemExternalTxFifo.hasSpace(count)) return false;
  for (uint16_t i = 0; i < count; i++) mavlinkTelemExternalTxFifo.push(buf[i]);
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
MAVLINK_RAM_SECTION Fifo<uint8_t, 1024> mavlinkTelemUsbRxFifo;
#endif

#if defined(AUX_SERIAL)

uint32_t mavlinkTelem1Available(void)
{
  if (!mavlinkTelem.serial1_enabled) return 0;
  if (mavlinkTelem.serial1_isexternal) return mavlinkTelemExternalRxFifo.size();

//  if (auxSerialMode != UART_MODE_MAVLINK) return 0;
  return auxSerialRxFifo.size();
}

// call only after check with mavlinkTelem2Available()
uint8_t mavlinkTelem1Getc(uint8_t* c)
{
  if (!mavlinkTelem.serial1_enabled) return 0;
  if (mavlinkTelem.serial1_isexternal) return mavlinkTelemExternalRxFifo.pop(*c);

  return auxSerialRxFifo.pop(*c);
}

bool mavlinkTelem1HasSpace(uint16_t count)
{
  if (!mavlinkTelem.serial1_enabled) return 0;
  if (mavlinkTelem.serial1_isexternal) return mavlinkTelemExternalTxFifo.hasSpace(count);

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
  if (mavlinkTelem.serial2_isexternal) return mavlinkTelemExternalRxFifo.size();

//  if (aux2SerialMode != UART_MODE_MAVLINK) return 0;
  return aux2SerialRxFifo.size();
}

// call only after check with mavlinkTelem2Available()
uint8_t mavlinkTelem2Getc(uint8_t* c)
{
  if (!mavlinkTelem.serial2_enabled) return 0;
  if (mavlinkTelem.serial2_isexternal) return mavlinkTelemExternalRxFifo.pop(*c);

  return aux2SerialRxFifo.pop(*c);
}

bool mavlinkTelem2HasSpace(uint16_t count)
{
  if (!mavlinkTelem.serial2_enabled) return 0;
  if (mavlinkTelem.serial2_isexternal) return mavlinkTelemExternalTxFifo.hasSpace(count);

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

tmr10ms_t mavlinkRcOverrideRate(void)
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

// -- MavlinkTelem handlers --

// map aux1,aux2,external onto serial1 & serial2
void MavlinkTelem::map_serials(void)
{
  if (_external_enabled) {
    if (_aux1_enabled && _aux2_enabled) {
      // shit, what should we do??? we give aux,aux2 priority
      serial1_enabled = serial2_enabled = true;
      serial1_isexternal = serial2_isexternal = false;
    }
    else if (_aux1_enabled && !_aux2_enabled) {
      serial1_enabled = true;
      serial1_isexternal = false;
      serial2_enabled = serial2_isexternal = true;
    }
    else if (!_aux1_enabled && _aux2_enabled) {
      serial1_enabled = serial1_isexternal = true;
      serial2_enabled = true;
      serial2_isexternal = false;
    }
    else {
      serial1_enabled = serial1_isexternal = true;
      serial2_enabled = serial2_isexternal = false;
    }
  }
  else{
    serial1_enabled = _aux1_enabled;
    serial2_enabled = _aux2_enabled;
    serial1_isexternal = serial2_isexternal = false;
  }
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


