/*
 * (c) www.olliw.eu, OlliW, OlliW42
 */

#include "opentx.h"

MAVLINK_SECTION MavlinkTelem mavlinkTelem;

// -- CoOS RTOS mavlink task handlers --

RTOS_TASK_HANDLE mavlinkTaskId;
RTOS_DEFINE_STACK(mavlinkStack, MAVLINK_STACK_SIZE);

struct MavlinkTaskStat {
  uint16_t start = 0;
  uint16_t last = 0;
  uint16_t max = 0;
  uint16_t load = 0;
};
struct MavlinkTaskStat mavlinkTaskStat;

uint16_t mavlinkTaskRunTime(void)
{
  return mavlinkTaskStat.start/2;
}

uint16_t mavlinkTaskRunTimeMax(void)
{
  return mavlinkTaskStat.max/2;
}

uint16_t mavlinkTaskLoad(void)
{
  return mavlinkTaskStat.load;
}

void mavlinkTask(void * pdata)
{
  while (true) {
    uint16_t start_last = mavlinkTaskStat.start;
    mavlinkTaskStat.start = getTmr2MHz();

    mavlinkTelem.wakeup();

    mavlinkTaskStat.last = getTmr2MHz() - mavlinkTaskStat.start;
    if (mavlinkTaskStat.last > mavlinkTaskStat.max) mavlinkTaskStat.max = mavlinkTaskStat.last;
    mavlinkTaskStat.load = mavlinkTaskStat.last / (mavlinkTaskStat.start - start_last);

    RTOS_WAIT_TICKS(2);
  }
}

void mavlinkStart()
{
  RTOS_CREATE_TASK(mavlinkTaskId, mavlinkTask, "mavlink", mavlinkStack, MAVLINK_STACK_SIZE, MAVLINK_TASK_PRIO);
}

// -- Miscellaneous stuff --

void MavlinkTelem::telemetrySetValue(uint16_t id, uint8_t subId, uint8_t instance, int32_t value, uint32_t unit, uint32_t prec)
{
  if (g_model.mavlinkRssi) {
    if (!radio.is_receiving && !radio.is_receiving65 && !radio.is_receiving35) return;
  }

  if (g_model.mavlinkMimicSensors) {
    setTelemetryValue(PROTOCOL_TELEMETRY_FRSKY_SPORT, id, subId, instance, value, unit, prec);
    telemetryStreaming = MAVLINK_TELEM_RADIO_RECEIVING_TIMEOUT; //2 * TELEMETRY_TIMEOUT10ms; // 2 seconds
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
    rssi = (uint8_t)( ((uint16_t)rssi * 100) / g_model.mavlinkRssiScale); //scale to 0..99
  }
  else { //mavlink default
    if (rssi == UINT8_MAX) rssi = 0;
  }

  radio.rssi_scaled = rssi;

  if (g_model.mavlinkRssi) {
    if (!radio.is_receiving && !radio.is_receiving65 && !radio.is_receiving35) return;
  }

  if (g_model.mavlinkRssi) {
    telemetryData.rssi.set(rssi);
    telemetryStreaming = MAVLINK_TELEM_RADIO_RECEIVING_TIMEOUT; //2 * TELEMETRY_TIMEOUT10ms; // 2 seconds
  }

  if (g_model.mavlinkRssi || g_model.mavlinkMimicSensors) {
    setTelemetryValue(PROTOCOL_TELEMETRY_FRSKY_SPORT, RSSI_ID, 0, 1, (int32_t)rssi, UNIT_DB, 0);
    telemetryStreaming = MAVLINK_TELEM_RADIO_RECEIVING_TIMEOUT; //2 * TELEMETRY_TIMEOUT10ms; // 2 seconds
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

  if (g_model.mavlinkRssi)
    telemetryData.rssi.reset();

  if (g_model.mavlinkRssi || g_model.mavlinkMimicSensors)
    setTelemetryValue(PROTOCOL_TELEMETRY_FRSKY_SPORT, RSSI_ID, 0, 1, 0, UNIT_DB, 0);
}

bool MavlinkTelem::telemetryVoiceEnabled(void)
{
  if (!g_model.mavlinkRssi && !g_model.mavlinkMimicSensors) return true;

  if (g_model.mavlinkRssi && !radio.rssi_voice_disabled) return true;

  return false;
}

// -- TASK handlers --
// tasks can be set directly with SETTASK()
// some tasks don't need immediate execution, or need reliable request
// this is what these handlers are for
// they push the task to a fifo, and also allow to set number of retries and retry rates

void MavlinkTelem::push_task(uint8_t idx, uint32_t task)
{
  struct Task t = {.task = task, .idx = idx};
  _taskFifo.push(t);
}

void MavlinkTelem::pop_and_set_task(void)
{
  struct Task t;
  if (_taskFifo.pop(t)) SETTASK(t.idx, t.task);
}

// -- REQUEST handlers --

void MavlinkTelem::set_request(uint8_t idx, uint32_t task, uint8_t retry, tmr10ms_t rate)
{
  push_task(idx, task);

  _request_is_waiting[idx] |= task;

  if (retry == 0) return; //well, if there would be another pending we would not kill it

  int8_t empty_i = -1;

  // first check if request is already pending, at the same time find free slot, to avoid having to loop twice
  for (uint16_t i = 0; i < REQUESTLIST_MAX; i++) {
    // TODO: should we modify the retry & rate of the pending task?
    if ((_requestList[i].idx == idx) && (_requestList[i].task == task)) return; // already pending, we can get out of here
    if ((empty_i < 0) && !_requestList[i].task) empty_i = i; // free slot
  }

  // if not already pending, add it
  if (empty_i < 0) return; // no free slot

  _requestList[empty_i].task = task;
  _requestList[empty_i].idx = idx;
  _requestList[empty_i].retry = retry;
  _requestList[empty_i].tlast = get_tmr10ms();
  _requestList[empty_i].trate = rate;
}

void MavlinkTelem::clear_request(uint8_t idx, uint32_t task)
{
  for (uint16_t i = 0; i < REQUESTLIST_MAX; i++) {
    if ((_requestList[i].idx == idx) && (_requestList[i].task == task)) {
      _requestList[i].task = 0;
      _request_is_waiting[idx] &=~ task;
    }
  }
}

//what happens if a clear never comes?
// well, this is what retry = UINT8_MAX says, right

void MavlinkTelem::do_requests(void)
{
  tmr10ms_t tnow = get_tmr10ms();

  for (uint16_t i = 0; i < TASKIDX_MAX; i++) _request_is_waiting[i] = 0;

  for (uint16_t i = 0; i < REQUESTLIST_MAX; i++) {
    if (!_requestList[i].task) continue;

    _request_is_waiting[_requestList[i].idx] |= _requestList[i].task;

    if ((tnow - _requestList[i].tlast) >= _requestList[i].trate) {
      push_task(_requestList[i].idx, _requestList[i].task);
      _requestList[i].tlast = get_tmr10ms();
      if (_requestList[i].retry < UINT8_MAX) {
        if (_requestList[i].retry) _requestList[i].retry--;
        if (!_requestList[i].retry) _requestList[i].task = 0; // clear request
      }
    }
  }

  if ((tnow - _taskFifo_tlast) > 6) { // 60 ms decimation
    _taskFifo_tlast = tnow;
    // change this, so that it skips tasks with 0, this would allow an easy means to clear tasks also in the Fifo
    if (!_taskFifo.isEmpty()) pop_and_set_task();
  }
}

// -- MAVLink stuff --

bool MavlinkTelem::isInVersionV2(void)
{
  return (_status.flags & MAVLINK_STATUS_FLAG_IN_MAVLINK1) ? false : true;
}

void MavlinkTelem::setOutVersionV2(void)
{
  _status.flags &=~ MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
}

void MavlinkTelem::setOutVersionV1(void)
{
  _status.flags |= MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
}

// -- Generate MAVLink messages --
// these should never be called directly, should only by called by the task handler

void MavlinkTelem::_generateCmdLong(
    uint8_t tsystem, uint8_t tcomponent, uint16_t cmd,
    float p1, float p2, float p3, float p4, float p5, float p6, float p7)
{
  setOutVersionV2();
  mavlink_msg_command_long_pack(
      _my_sysid, _my_compid, &_msg_out,
      tsystem, tcomponent, cmd, 0, p1, p2, p3, p4, p5, p6, p7
      );
  _msg_out_available = true;
}

void MavlinkTelem::generateHeartbeat(uint8_t base_mode, uint32_t custom_mode, uint8_t system_status)
{
  setOutVersionV2();
  mavlink_msg_heartbeat_pack(
      _my_sysid, _my_compid, &_msg_out,
      MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID, base_mode, custom_mode, system_status
      );
  _msg_out_available = true;
}

void MavlinkTelem::generateParamRequestList(uint8_t tsystem, uint8_t tcomponent)
{
  setOutVersionV2();
  mavlink_msg_param_request_list_pack(
      _my_sysid, _my_compid, &_msg_out,
      tsystem, tcomponent
      );
  _msg_out_available = true;
}

void MavlinkTelem::generateParamRequestRead(uint8_t tsystem, uint8_t tcomponent, const char* param_name)
{
char param_id[16];

  strncpy(param_id, param_name, 16);
  setOutVersionV2();
  mavlink_msg_param_request_read_pack(
      _my_sysid, _my_compid, &_msg_out,
      tsystem, tcomponent, param_id, -1
      );
  _msg_out_available = true;
}

void MavlinkTelem::generateRequestDataStream(
    uint8_t tsystem, uint8_t tcomponent, uint8_t data_stream, uint16_t rate_hz, uint8_t startstop)
{
  setOutVersionV2();
  mavlink_msg_request_data_stream_pack(
      _my_sysid, _my_compid, &_msg_out,
      tsystem, tcomponent, data_stream, rate_hz, startstop
      );
  _msg_out_available = true;
}

//ArduPilot: ignores param7
void MavlinkTelem::generateCmdSetMessageInterval(uint8_t tsystem, uint8_t tcomponent, uint8_t msgid, int32_t period_us, uint8_t startstop)
{
  _generateCmdLong(tsystem, tcomponent, MAV_CMD_SET_MESSAGE_INTERVAL, msgid, (startstop) ? period_us : -1.0f);
}

// -- Task handlers --

// -- Handle incoming MAVLink messages, which are for the Gimbal --
// -- Handle incoming MAVLink messages, which are for the Gimbal Manager --
// -- Handle incoming MAVLink messages, which are for the Camera --
// -- Handle incoming MAVLink messages, which are for the Autopilot --
// -- Handle incoming MAVLink messages, which are for QShots --

// -- Main handler for incoming MAVLink messages --

void MavlinkTelem::handleMessage(void)
{
  if (_msg.sysid == 0) return; //this can't be anything meaningful

  // autodetect sys id, and handle autopilot connecting
  if (!isSystemIdValid() || (autopilot.compid == 0)) {
    if (_msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
      mavlink_heartbeat_t payload;
      mavlink_msg_heartbeat_decode(&_msg, &payload);
      if ((_msg.compid == MAV_COMP_ID_AUTOPILOT1) || (payload.autopilot != MAV_AUTOPILOT_INVALID)) {
        _sysid = _msg.sysid;
        autopilottype = payload.autopilot;
        vehicletype = payload.type;
        _resetAutopilot();
        autopilot.compid = _msg.compid;
        autopilot.requests_triggered = 1; //we need to postpone and schedule them
      }
    }
    if (!isSystemIdValid()) return;
  }

  // discoverers
  // somewhat inefficient, lots of heartbeat decodes, we probably want a separate heartbeat handler

  if ((camera.compid == 0) && (_msg.msgid == MAVLINK_MSG_ID_HEARTBEAT)) {
    mavlink_heartbeat_t payload;
    mavlink_msg_heartbeat_decode(&_msg, &payload);
    if ( (payload.autopilot == MAV_AUTOPILOT_INVALID) &&
       ( (payload.type == MAV_TYPE_CAMERA) ||
         ((_msg.compid >= MAV_COMP_ID_CAMERA) && (_msg.compid <= MAV_COMP_ID_CAMERA6)) ) ) {
      _resetCamera();
      camera.compid = _msg.compid;
      camera.requests_triggered = 1; //we schedule them
    }
  }

  if ((gimbal.compid == 0) && (_msg.msgid == MAVLINK_MSG_ID_HEARTBEAT)) {
    mavlink_heartbeat_t payload;
    mavlink_msg_heartbeat_decode(&_msg, &payload);
    if ( (payload.autopilot == MAV_AUTOPILOT_INVALID) &&
       ( (payload.type == MAV_TYPE_GIMBAL) ||
         ((_msg.compid == MAV_COMP_ID_GIMBAL) ||
         ((_msg.compid >= MAV_COMP_ID_GIMBAL2) && (_msg.compid <= MAV_COMP_ID_GIMBAL6))) ) ) {
      _resetGimbalAndGimbalClient();
      gimbal.compid = _msg.compid;
      gimbal.is_initialized = true; //no startup requests, so true
    }
  }

  if ((gimbalmanager.compid == 0) && (gimbal.compid > 0) && (_msg.msgid == MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_STATUS)) {
    mavlink_storm32_gimbal_manager_status_t payload;
    mavlink_msg_storm32_gimbal_manager_status_decode(&_msg, &payload);
    if (payload.gimbal_id == gimbal.compid) { //this is the gimbal's gimbal manager
      _resetGimbalClient();
      gimbalmanager.compid = _msg.compid;
      gimbalmanagerOut.device_flags = payload.device_flags;
      gimbalmanagerOut.manager_flags = payload.manager_flags;
      gimbalmanager.requests_triggered = 1; //we schedule them
    }
  }

  // reset receiving timeout, but ignore RADIO_STATUS
  if (_msg.msgid != MAVLINK_MSG_ID_RADIO_STATUS) {
    _is_receiving = MAVLINK_TELEM_RECEIVING_TIMEOUT;
  }

  // MAVLINK
  //if (msgFifo_enabled) msgRxFifo.push(_msg);

  // MAVSDK
  // also try to convert the MAVLink messages to FrSky sensors

  // RADIO_STATUS is somewhat tricky, this may need doing it better if there are more sources of it
  // SiK comes as vehicle 51, comp 68!
  // it must NOT be rated as _is_recieving!
  if (_msg.msgid == MAVLINK_MSG_ID_RADIO_STATUS) {
    mavlink_radio_status_t payload;
    mavlink_msg_radio_status_decode(&_msg, &payload);
    radio.rssi = payload.rssi;
    radio.remrssi = payload.remrssi;
    radio.noise = payload.noise;
    radio.remnoise = payload.remnoise;
    radio.is_receiving = MAVLINK_TELEM_RADIO_RECEIVING_TIMEOUT;
    telemetrySetRssiValue(radio.rssi);
    return;
  }

  //we handle all qshot wherever they come from
  handleMessageQShot();

  if (_msg.sysid != _sysid) return; //this is not from our system

  // handle messages coming from autopilot
  if (autopilot.compid && (_msg.compid == autopilot.compid)) {
    handleMessageAutopilot();
  }
  if (camera.compid && (_msg.compid == camera.compid)) {
    handleMessageCamera();
  }
  if (gimbal.compid && (_msg.compid == gimbal.compid)) {
    handleMessageGimbal();
  }
  if (gimbalmanager.compid && (_msg.compid == gimbalmanager.compid)) {
    handleMessageGimbalClient();
  }
}

// -- Main task handler --

void MavlinkTelem::doTask(void)
{
  tmr10ms_t tnow = get_tmr10ms();

  bool tick_1Hz = false;

  if ((tnow - _my_heartbeat_tlast) > 100) { //1 sec
    _my_heartbeat_tlast = tnow;
    SETTASK(TASK_ME, TASK_SENDMYHEARTBEAT);

    msg_rx_persec = _msg_rx_persec_cnt;
    _msg_rx_persec_cnt = 0;
    bytes_rx_persec = _bytes_rx_persec_cnt;
    _bytes_rx_persec_cnt = 0;

    tick_1Hz = true;
  }

  if (!isSystemIdValid()) return;

  // trigger startup requests

  // we need to wait until at least one heartbeat was send out before requesting data streams
  if (autopilot.compid && autopilot.requests_triggered) {
    if (tick_1Hz) autopilot.requests_triggered++;
    if (autopilot.requests_triggered > 3) { // wait for 3 heartbeats
      autopilot.requests_triggered = 0;
      setAutopilotStartupRequests();
    }
  }

  // we wait until at least one heartbeat was send out, and autopilot requests have been done
  if (camera.compid && camera.requests_triggered && !autopilot.requests_triggered) {
    if (tick_1Hz) camera.requests_triggered++;
    if (camera.requests_triggered > 1) { // wait for the next heartbeat
      camera.requests_triggered = 0;
      setCameraStartupRequests();
    }
  }

  // we wait until at least one heartbeat was send out, and autopilot requests have been done
  if (gimbal.compid && gimbal.requests_triggered && !autopilot.requests_triggered) {
    if (tick_1Hz) gimbal.requests_triggered++;
    if (gimbal.requests_triggered > 1) { // wait for the next heartbeat
      gimbal.requests_triggered = 0;
      setGimbalStartupRequests();
    }
  }
  if (gimbalmanager.compid && gimbalmanager.requests_triggered && !autopilot.requests_triggered) {
    if (tick_1Hz) gimbalmanager.requests_triggered++;
    if (gimbalmanager.requests_triggered > 1) { // wait for the next heartbeat
      gimbalmanager.requests_triggered = 0;
      setGimbalClientStartupRequests();
    }
  }

  if (!autopilot.is_initialized) autopilot.is_initialized = (autopilot.requests_waiting_mask == 0); 
  
  if (!camera.is_initialized) camera.is_initialized = (camera.requests_waiting_mask == 0); 
  
  if (!gimbal.is_initialized) gimbal.is_initialized = (gimbal.requests_waiting_mask == 0); 
  
  if (!gimbalmanager.is_initialized) gimbalmanager.is_initialized = (gimbalmanager.requests_waiting_mask == 0); 
  
  // handle pending requests
  do_requests();

  // do rc override
  // ArduPilot has a DAMED BUG!!!
  // per MAVLink spec 0 and UNIT16_MAX should not be considered for channels >= 8, but it doesn't do it for 0
  // but we can hope that it handles 0 for the higher channels
  if (g_model.mavlinkRcOverride && param.SYSID_MYGCS >= 0) {
    if ((tnow - _rcoverride_tlast) >= 5) { //50 ms
      _rcoverride_tlast = tnow;
      for (uint8_t i = 0; i < 8; i++) {
        /* would this be the right way to figure out which output is actually active ??
        MixData * md;
        if (i < MAX_MIXERS && (md=mixAddress(i))->srcRaw && md->destCh == i) {
          int value = channelOutputs[i] + 2 * PPM_CH_CENTER(i) - 2 * PPM_CENTER;
          _tovr_chan_raw[i] = value;
        }
        else {
          _tovr_chan_raw[i] = UINT16_MAX;
        }*/
        // the first four channels may not be ordered like with transmitter!!
        int value = channelOutputs[i]/2 + PPM_CH_CENTER(i);
        _tovr_chan_raw[i] = value;
      }
      for (uint8_t i = 8; i < 18; i++) { 
        _tovr_chan_raw[i] = 0; 
      }
      SETTASK(TASK_AUTOPILOT, TASK_SENDMSG_RC_CHANNELS_OVERRIDE);
    }
  }

  // handle pending tasks
  // do only one task and hence one msg_out per loop
  if (!_msg_out_available && TASK_IS_PENDING()) {
    //TASK_ME
    if (_task[TASK_ME] & TASK_SENDMYHEARTBEAT) {
      RESETTASK(TASK_ME,TASK_SENDMYHEARTBEAT);
      uint8_t base_mode = MAV_MODE_PREFLIGHT | MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | MAV_MODE_FLAG_SAFETY_ARMED;
      uint8_t system_status = MAV_STATE_UNINIT | MAV_STATE_ACTIVE;
      uint32_t custom_mode = 0;
      generateHeartbeat(base_mode, custom_mode, system_status);
      return; //do only one per loop
    }

    //other TASKS
    if (doTaskAutopilot()) return;
    if (doTaskGimbalAndGimbalClient()) return;
    if (doTaskCamera()) return;
    if (doTaskAutopilotLowPriority()) return;
    if (doTaskCameraLowPriority()) return;
    if (doTaskQShot()) return;
  }
}

// -- Wakeup call from OpenTx --
// this is the main entry point
#if !defined(AUX_SERIAL)
uint32_t mavlinkTelemAvailable(void){ return 0; }
uint8_t mavlinkTelemGetc(uint8_t *c){ return 0; }
bool mavlinkTelemPutBuf(const uint8_t *buf, const uint16_t count){ return false; }
#endif
#if !defined(AUX2_SERIAL)
uint32_t mavlinkTelem2Available(void){ return 0; }
uint8_t mavlinkTelem2Getc(uint8_t *c){ return 0; }
bool mavlinkTelem2PutBuf(const uint8_t *buf, const uint16_t count){ return false; }
#endif

void MavlinkTelem::wakeup()
{
  // check configuration
  bool serial1_enabled = g_eeGeneral.auxSerialMode == UART_MODE_MAVLINK;
  bool serial2_enabled = g_eeGeneral.aux2SerialMode == UART_MODE_MAVLINK;

  if ((_serial1_enabled != serial1_enabled) || (_serial2_enabled != serial2_enabled) ||
      (_serial1_baudrate != g_eeGeneral.mavlinkBaudrate) || (_serial2_baudrate != g_eeGeneral.mavlinkBaudrate2)) {
    _serial1_enabled = serial1_enabled;
    _serial2_enabled = serial2_enabled;
    _serial1_baudrate = g_eeGeneral.mavlinkBaudrate;
    _serial2_baudrate = g_eeGeneral.mavlinkBaudrate2;
    _reset();
  }

  // skip out if not enabled
  if (!_serial1_enabled && !_serial2_enabled) return;

  // look for incoming messages, also do statistics
  uint32_t available = (_serial1_enabled) ? mavlinkTelemAvailable() : mavlinkTelem2Available();
  if (available > 128) available = 128; //limit how much we read at once, shouldn't ever trigger, 11.1 ms @ 115200
  for (uint32_t i = 0; i < available; i++) {
    uint8_t c;
    if (_serial1_enabled) mavlinkTelemGetc(&c); else mavlinkTelem2Getc(&c);
    _bytes_rx_persec_cnt++;
    if (mavlink_parse_char(MAVLINK_COMM_0, c, &_msg, &_status)) {
      // check for lost messages by analyzing seq
      if (_seq_rx_last >= 0) {
        uint16_t seq = _msg.seq;
        if (seq < _seq_rx_last) seq += 256;
        _seq_rx_last++;
        if (seq > _seq_rx_last) msg_rx_lost += (seq - _seq_rx_last);
      }
      _seq_rx_last = _msg.seq;
      handleMessage();
      msg_rx_count++;
      _msg_rx_persec_cnt++;
    }
  }

  // do tasks
  doTask();

  // send out any pending messages
  if (_msg_out_available) {
    uint16_t count = mavlink_msg_to_send_buffer(_txbuf, &_msg_out);
    bool res = (_serial1_enabled) ? mavlinkTelemPutBuf(_txbuf, count) : mavlinkTelem2PutBuf(_txbuf, count);
    if (res) {
      _msg_out_available = false;
    }
  }
}

// -- 10 ms tick --

void MavlinkTelem::tick10ms()
{
  #define check(x,y) if(x){ (x)--; if(!(x)){ (y); }}

  check(_is_receiving, _reset());

  check(radio.is_receiving, _resetRadio());
  check(radio.is_receiving65, _resetRadio65());
  check(radio.is_receiving35, _resetRadio35());

  check(autopilot.is_receiving, _resetAutopilot());
  check(gimbal.is_receiving, _resetGimbalAndGimbalClient());
  check(gimbalmanager.is_receiving, _resetGimbalClient());
  check(camera.is_receiving, _resetCamera());
}

// -- Resets --

void MavlinkTelem::_resetRadio(void)
{
  radio.is_receiving = 0;

  radio.rssi = UINT8_MAX;
  radio.remrssi = UINT8_MAX;
  radio.noise = 0;
  radio.remnoise = 0;

  telemetryResetRssiValue();
}

void MavlinkTelem::_resetRadio65(void)
{
  radio.is_receiving65 = 0;
  radio.rssi65 = UINT8_MAX;

  telemetryResetRssiValue();
}

void MavlinkTelem::_resetRadio35(void)
{
  radio.is_receiving35 = 0;
  radio.rssi35 = UINT8_MAX;

  telemetryResetRssiValue();
}

void MavlinkTelem::_reset(void)
{
  mavlink_reset_channel_status(MAVLINK_COMM_0);

  _my_sysid = MAVLINK_TELEM_MY_SYSID;
  _my_compid = MAVLINK_TELEM_MY_COMPID;

  msg_rx_count = 0;
  msg_rx_lost = 0;
  msg_rx_persec = 0;
  bytes_rx_persec = 0;
  _msg_rx_persec_cnt = 0;
  _bytes_rx_persec_cnt = 0;
  _seq_rx_last = -1;

  _sysid = 0;
  autopilottype = MAV_AUTOPILOT_GENERIC; //TODO: shouldn't these be in _resetAutopilot() ??
  vehicletype = MAV_TYPE_GENERIC;
  flightmode = 0;

  for (uint16_t i = 0; i < TASKIDX_MAX; i++) _task[i] = 0;
  _taskFifo.clear();
  _taskFifo_tlast = 0;
  for (uint16_t i = 0; i < REQUESTLIST_MAX; i++) _requestList[i].task = 0;

  _resetRadio();
  _resetRadio65();
  _resetRadio35();
  radio.rssi_scaled = 0;
  radio.rssi_voice_disabled = false;

  _resetAutopilot();
  _resetGimbalAndGimbalClient();
  _resetCamera();

  _resetQShot();

  // MAVLINK
  //msgRxFifo.clear();
  //msgFifo_enabled = false;
}
