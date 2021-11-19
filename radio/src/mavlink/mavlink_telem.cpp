/*
 * The MAVLink for OpenTx project
 * (c) www.olliw.eu, OlliW, OlliW42
 */

#include "opentx.h"
#include "stamp.h"

MAVLINK_RAM_SECTION MavlinkTelem mavlinkTelem;

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

  if (retry == 0) return; // well, if there would be another pending we would not kill it

  int8_t empty_i = -1;

  // first check if request is already pending, at the same time find free slot, to avoid having to loop twice
  for (uint16_t i = 0; i < REQUESTLIST_MAX; i++) {
    //TODO: should we modify the retry & rate of the pending task?
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

// what happens if a clear never comes?
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

// -- My Parameters

const fmav_param_entry_t fmav_param_list[FASTMAVLINK_PARAM_NUM] = {
  {(uint8_t*)&(mavlinkTelem.p.my_sysid), MAV_PARAM_TYPE_UINT8, "MY_SYSID" },
  {(uint8_t*)&(mavlinkTelem.p.my_compid), MAV_PARAM_TYPE_UINT8, "MY_COMPID" },
  {(uint8_t*)&(mavlinkTelem.p.mavlinkRssi), MAV_PARAM_TYPE_UINT8, "RSSI_ENABLE" },
  {(uint8_t*)&(mavlinkTelem.p.mavlinkRssiScale), MAV_PARAM_TYPE_UINT8, "RSSI_SCALE" },
  {(uint8_t*)&(mavlinkTelem.p.mavlinkMimicSensors), MAV_PARAM_TYPE_UINT8, "MIMIC_SENSORS" },
  {(uint8_t*)&(mavlinkTelem.p.mavlinkRcOverride), MAV_PARAM_TYPE_UINT8, "RC_OVERRIDE" },
  {(uint8_t*)&(mavlinkTelem.p.mavlinkSendPosition), MAV_PARAM_TYPE_UINT8, "SEND_POSITION" },
};

void MavlinkTelem::_mavlink_copy_g2p(void)
{
  p.my_sysid = MAVLINK_TELEM_MY_SYSID; // cannot be written, only read
  p.my_compid = MAVLINK_TELEM_MY_COMPID; // cannot be written, only read

  p.mavlinkRssi = g_model.mavlinkRssi;
  p.mavlinkRssiScale = g_model.mavlinkRssiScale;
  p.mavlinkMimicSensors = g_model.mavlinkMimicSensors;
  p.mavlinkRcOverride = g_model.mavlinkRcOverride;
  p.mavlinkSendPosition = g_model.mavlinkSendPosition;
}

void MavlinkTelem::_mavlink_copy_p2g(void)
{
  g_model.mavlinkRssi = (p.mavlinkRssi > 0) ? 1 : 0;
  g_model.mavlinkRssiScale = p.mavlinkRssiScale;
  g_model.mavlinkMimicSensors = (p.mavlinkMimicSensors > 0) ? 1 : 0;
  if (mavlinkTelem.p.mavlinkRcOverride > 14) p.mavlinkRcOverride = 0;
  g_model.mavlinkRcOverride = p.mavlinkRcOverride;
  g_model.mavlinkSendPosition = (p.mavlinkSendPosition > 0) ? 1 : 0;
}

// -- Generate MAVLink messages --
// these should never be called directly, should only by called by the task handler

void MavlinkTelem::_generateCmdLong(
    uint8_t tsystem, uint8_t tcomponent, uint16_t cmd,
    float p1, float p2, float p3, float p4, float p5, float p6, float p7)
{
  fmav_msg_command_long_pack(
      &_msg_out, _my_sysid, _my_compid,
      tsystem, tcomponent, cmd, 0, p1, p2, p3, p4, p5, p6, p7,
      &_status_out
      );
  _msg_out_available = true;
}

void MavlinkTelem::generateHeartbeat(uint8_t base_mode, uint32_t custom_mode, uint8_t system_status)
{
  fmav_msg_heartbeat_pack(
      &_msg_out, _my_sysid, _my_compid,
      MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID, base_mode, custom_mode, system_status,
      &_status_out
      );
  _msg_out_available = true;
}

void MavlinkTelem::generateAutopilotVersion(void)
{
uint64_t capabilities = 0;
uint32_t flight_sw_version = OWVERSION;
uint32_t middleware_sw_version = 0;
uint32_t os_sw_version = 2314;
uint32_t board_version = 0;
uint16_t vendor_id = 0x1209; // USBD_VID_PID_CODES // https://pid.codes
uint16_t product_id = 0x4F54; // USBD_HID_PID // OpenTX assigned PID
uint64_t uid = 0;
uint8_t dummy[20] = {0};

  capabilities = MAV_PROTOCOL_CAPABILITY_MAVLINK2;

  // the STM32's factory-programmed UUID memory = three values of 32 bits starting at this address
  // we generate it from the STM32 unique id, which has 12 bytes = 96 bits
  #define STM32F4_UUID_ADR  0x1FFF7A10
  memcpy(&uid, (uint32_t*)STM32F4_UUID_ADR, sizeof(uid));

  fmav_msg_autopilot_version_pack(
      &_msg_out, _my_sysid, _my_compid,
      capabilities,
      flight_sw_version, middleware_sw_version, os_sw_version, board_version,
      dummy, dummy, dummy,
      vendor_id, product_id, uid, dummy,
      &_status_out
      );
  _msg_out_available = true;
}

void MavlinkTelem::generateStatustext(uint8_t severity, const char* text, uint16_t id, uint8_t chunk_seq)
{
  fmav_msg_statustext_pack(
      &_msg_out, _my_sysid, _my_compid,
      severity, text, id, chunk_seq,
      &_status_out
      );
  _msg_out_available = true;
}

void MavlinkTelem::generateParamValue(const char* param_name, float param_value, uint8_t param_type, uint16_t param_count, uint16_t param_index)
{
char param_id[16];

  memset(param_id, 0, 16);
  strncpy(param_id, param_name, 16);
  fmav_msg_param_value_pack(
      &_msg_out, _my_sysid, _my_compid,
      param_id, param_value, param_type, param_count, param_index,
      &_status_out
      );
  _msg_out_available = true;
}

void MavlinkTelem::generateParamRequestList(uint8_t tsystem, uint8_t tcomponent)
{
  fmav_msg_param_request_list_pack(
      &_msg_out, _my_sysid, _my_compid,
      tsystem, tcomponent,
      &_status_out
      );
  _msg_out_available = true;
}

void MavlinkTelem::generateParamRequestRead(uint8_t tsystem, uint8_t tcomponent, const char* param_name)
{
char param_id[16];

  memset(param_id, 0, 16);
  strncpy(param_id, param_name, 16);
  fmav_msg_param_request_read_pack(
      &_msg_out, _my_sysid, _my_compid,
      tsystem, tcomponent, param_id, -1,
      &_status_out
      );
  _msg_out_available = true;
}

void MavlinkTelem::generateParamSet(uint8_t tsystem, uint8_t tcomponent, const char* param_name, float param_value, uint8_t param_type)
{
char param_id[16];

  memset(param_id, 0, 16);
  strncpy(param_id, param_name, 16);
  fmav_msg_param_set_pack(
      &_msg_out, _my_sysid, _my_compid,
      tsystem, tcomponent, param_id, param_value, param_type,
      &_status_out
      );
  _msg_out_available = true;
}

// -- Mavsdk Convenience Task Wrapper --
// to make it easy for api_mavsdk to call functions

void MavlinkTelem::sendGolbalPositionInt(int32_t lat, int32_t lon, float alt, float relative_alt, float vx, float vy, float vz, float hdg_deg)
{
  _gpi_lat = lat;
  _gpi_lon = lon;
  _gpi_alt = alt * 1000.0f;
  _gpi_relative_alt = relative_alt * 1000.0f;
  _gpi_vx = vx * 100.0f;
  _gpi_vy = vy * 100.0f;
  _gpi_vz = vz * 100.0f;
  _gpi_hdg = (hdg_deg > 360.0f) ? UINT16_MAX : hdg_deg;
  SETTASK(TASK_ME, TASK_ME_SENDMSG_GLOBAL_POSITION_INT);
}

// -- Main message handler for incoming MAVLink messages --

void MavlinkTelem::handleMessage(void)
{
  if (_msg.sysid == 0) return; // this can't be anything meaningful

  // autodetect sys id, and handle autopilot connecting
  if (!isSystemIdValid() || (autopilot.compid == 0)) {
    if (_msg.msgid == FASTMAVLINK_MSG_ID_HEARTBEAT) {
      fmav_heartbeat_t payload;
      fmav_msg_heartbeat_decode(&payload, &_msg);
      if ((_msg.compid == MAV_COMP_ID_AUTOPILOT1) || (payload.autopilot != MAV_AUTOPILOT_INVALID)) {
        _sysid = _msg.sysid;
        autopilottype = payload.autopilot;
        vehicletype = payload.type;
        _resetAutopilot();
        autopilot.compid = _msg.compid;
        autopilot.requests_triggered = 1; // we need to postpone and schedule them
      }
    }
    if (!isSystemIdValid()) return;
  }

  msg_rx_count++;
  _msg_rx_persec_cnt++;
  _bytes_rx_persec_cnt += fmav_msg_frame_len(&_msg);

  // discoverers
  // somewhat inefficient, lots of heartbeat decodes, we probably want a separate heartbeat handler

  if ((camera.compid == 0) && (_msg.msgid == FASTMAVLINK_MSG_ID_HEARTBEAT)) {
    fmav_heartbeat_t payload;
    fmav_msg_heartbeat_decode(&payload, &_msg);
    if ( (payload.autopilot == MAV_AUTOPILOT_INVALID) &&
       ( (payload.type == MAV_TYPE_CAMERA) ||
         ((_msg.compid >= MAV_COMP_ID_CAMERA) && (_msg.compid <= MAV_COMP_ID_CAMERA6)) ) ) {
      _resetCamera();
      camera.compid = _msg.compid;
      camera.requests_triggered = 1; // we schedule them
    }
  }

  if ((gimbal.compid == 0) && (_msg.msgid == FASTMAVLINK_MSG_ID_HEARTBEAT)) {
    fmav_heartbeat_t payload;
    fmav_msg_heartbeat_decode(&payload, &_msg);
    if ( (payload.autopilot == MAV_AUTOPILOT_INVALID) &&
       ( (payload.type == MAV_TYPE_GIMBAL) ||
         ((_msg.compid == MAV_COMP_ID_GIMBAL) ||
         ((_msg.compid >= MAV_COMP_ID_GIMBAL2) && (_msg.compid <= MAV_COMP_ID_GIMBAL6))) ) ) {
      _resetGimbalAndGimbalClient();
      gimbal.compid = _msg.compid;
      gimbal.is_initialized = true; // no startup requests, so true
    }
  }

  if ((gimbalmanager.compid == 0) && (gimbal.compid > 0) && (_msg.msgid == FASTMAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_STATUS)) {
    fmav_storm32_gimbal_manager_status_t payload;
    fmav_msg_storm32_gimbal_manager_status_decode(&payload, &_msg);
    if (payload.gimbal_id == gimbal.compid) { // this is the gimbal's gimbal manager
      _resetGimbalClient();
      gimbalmanager.compid = _msg.compid;
      gimbalmanagerOut.device_flags = payload.device_flags;
      gimbalmanagerOut.manager_flags = payload.manager_flags;
      gimbalmanager.requests_triggered = 1; // we schedule them
    }
  }

  // reset receiving timeout, but ignore RADIO_STATUS
  if (_msg.msgid != FASTMAVLINK_MSG_ID_RADIO_STATUS) {
    _is_receiving = MAVLINK_TELEM_RECEIVING_TIMEOUT;
  }

  // MAVLINK API
  mavapiHandleMessage(&_msg);

  // MAVSDK
  // also try to convert the MAVLink messages to FrSky sensors

  // RADIO_STATUS is somewhat tricky, this may need doing it better if there are more sources of it
  // SiK comes as vehicle 51, comp 68!
  // it must NOT be rated as _is_recieving!
  if (_msg.msgid == FASTMAVLINK_MSG_ID_RADIO_STATUS) {
    fmav_radio_status_t payload;
    fmav_msg_radio_status_decode(&payload, &_msg);
    radio.rssi = payload.rssi;
    radio.remrssi = payload.remrssi;
    radio.noise = payload.noise;
    radio.remnoise = payload.remnoise;
    radio.is_receiving = MAVLINK_TELEM_RADIO_RECEIVING_TIMEOUT;
    telemetrySetRssiValue(radio.remrssi); // let's report the rssi of the air side
    return;
  }

  // handle messages coming from a GCS or alike
  if (_msg.sysid != _sysid && (_msg.compid >= MAV_COMP_ID_MISSIONPLANNER && _msg.compid <= MAV_COMP_ID_PATHPLANNER)) {
    handleMessageGcsAndAlike();
  }

  // we handle all qshot wherever they come from
  handleMessageQShot();

  // handle parameter
  paramHandleMessage(&_msg);

  if (_msg.sysid != _sysid) return; // this is not from our system

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
    SETTASK(TASK_ME, TASK_ME_SENDMYHEARTBEAT);

    msg_rx_persec = _msg_rx_persec_cnt;
    bytes_rx_persec = _bytes_rx_persec_cnt;
    _msg_rx_persec_cnt = 0;
    _bytes_rx_persec_cnt = 0;

    msg_tx_persec = _msg_tx_persec_cnt;
    bytes_tx_persec = _bytes_tx_persec_cnt;
    _msg_tx_persec_cnt = 0;
    _bytes_tx_persec_cnt = 0;

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
    tmr10ms_t dt_10ms = mavlinkRcOverrideRate();
    if ((tnow - _rcoverride_tlast) >= dt_10ms) {
      _rcoverride_tlast += dt_10ms;
      if ((tnow - _rcoverride_tlast) >= dt_10ms) _rcoverride_tlast = tnow; //we are late, so get back in sync
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

  // do send global position int
  if (g_model.mavlinkSendPosition && param.SYSID_MYGCS >= 0) {
    if ((auxSerialMode == UART_MODE_GPS || aux2SerialMode == UART_MODE_GPS) &&
        (gps_msg_received_tlast > _gps_tlast)) {
      _gps_tlast = gps_msg_received_tlast;
      _txgps_has_pos_int_fix = ((gpsData.fix) && (gpsData.numSat >= 8) && (gpsData.hdop < 150));
      if (_txgps_has_pos_int_fix) {
        _gpi_lat = gpsData2.lat_1e7;
        _gpi_lon = gpsData2.lon_1e7;
        _gpi_alt = gpsData2.alt_cm * 10; // the gps height is extremely inaccurate
        _gpi_relative_alt = 1250; // home altitude ??? just set it to 1.25m
        _gpi_vx = _gpi_vy = _gpi_vz = 0;
        if (0) { //gpsData.speed > 10) { //don't do it ever currently
          constexpr float FPI = 3.141592653589793f;
          constexpr float FDEGTORAD = FPI/180.0f;
          float v = gpsData2.speed_cms * 0.01f;
          float course = gpsData2.cog_cdeg * 0.01f * FDEGTORAD;
          _gpi_vx = cosf(course) * v;
          _gpi_vy = sinf(course) * v;
        }
        _gpi_hdg = UINT16_MAX; //gpsData.groundCourse * 10;
        SETTASK(TASK_ME, TASK_ME_SENDMSG_GLOBAL_POSITION_INT);
      }
    }
  }

  if (!mavapiMsgOutEmpty()) SETTASK(TASK_ME, TASK_SENDMSG_MAVLINK_API);
  if (!_paramOutFifo.isEmpty()) SETTASK(TASK_ME, TASK_SENDMSG_MAVLINK_PARAM);

  // handle pending tasks
  // do only one task and hence one msg_out per loop
  if (!_msg_out_available && TASK_IS_PENDING()) {
    // other TASKS
    if (doTaskAutopilot()) return;
    if (doTaskGimbalAndGimbalClient()) return;
    if (doTaskCamera()) return;

    // TASK_ME
    if (_task[TASK_ME] & TASK_ME_SENDMYHEARTBEAT) {
      RESETTASK(TASK_ME, TASK_ME_SENDMYHEARTBEAT);
      uint8_t base_mode = MAV_MODE_PREFLIGHT | MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | MAV_MODE_FLAG_SAFETY_ARMED;
      uint8_t system_status = MAV_STATE_UNINIT | MAV_STATE_ACTIVE;
      uint32_t custom_mode = 0;
      generateHeartbeat(base_mode, custom_mode, system_status);
      return; //do only one per loop
    }
    if (_task[TASK_ME] & TASK_SENDMSG_MAVLINK_API) {
      RESETTASK(TASK_ME, TASK_SENDMSG_MAVLINK_API);
      mavapiGenerateMessage();
      return; //do only one per loop
    }
    if (_task[TASK_ME] & TASK_SENDMSG_MAVLINK_PARAM) {
      RESETTASK(TASK_ME, TASK_SENDMSG_MAVLINK_PARAM);
      paramGenerateMessage();
      return; //do only one per loop
    }
    if (_task[TASK_ME] & TASK_ME_SENDMSG_GLOBAL_POSITION_INT) {
      RESETTASK(TASK_ME,TASK_ME_SENDMSG_GLOBAL_POSITION_INT);
      generateGlobalPositionInt(_gpi_lat, _gpi_lon, _gpi_alt, _gpi_relative_alt, _gpi_vx, _gpi_vy, _gpi_vz, _gpi_hdg);
      return; //do only one per loop
    }
    if (_task[TASK_ME] & TASK_ME_SENDMYAUTOPILOTVERSION) {
      RESETTASK(TASK_ME, TASK_ME_SENDMYAUTOPILOTVERSION);
      generateAutopilotVersion();
      return; //do only one per loop
    }
    if (_task[TASK_ME] & TASK_ME_SENDMYBANNER) {
      RESETTASK(TASK_ME, TASK_ME_SENDMYBANNER);
      const char banner[] = "OpenTx with MAVLink " VERSION " " OWVERSIONSTR;
      char text[FASTMAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN];
      memset(text, 0, FASTMAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN);
      strncpy(text, banner, FASTMAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN);
      generateStatustext(MAV_SEVERITY_INFO, text, 0, 0);
      return; //do only one per loop
    }
    if (_task[TASK_ME] & TASK_ME_SENDMYPARAMLIST) {
      //RESETTASK(TASK_ME, TASK_ME_SENDMYPARAMLIST);
      if (tnow - _prl_tlast >= 10) {
        fmav_param_union_t param_union;
        if (fmav_param_get_param_union(&param_union, _prl_index)) {
          generateParamValue(fmav_param_list[_prl_index].name, param_union.p_float, param_union.type, FASTMAVLINK_PARAM_NUM, _prl_index);
        }
        _prl_index++;
        _prl_tlast = tnow;
        if (_prl_index >= FASTMAVLINK_PARAM_NUM) RESETTASK(TASK_ME, TASK_ME_SENDMYPARAMLIST);
        return; //do only one per loop
      }
      // do not return here, so other tasks can go on
    }
    if (_task[TASK_ME] & TASK_ME_SENDMYPARAMVALUE) {
      RESETTASK(TASK_ME, TASK_ME_SENDMYPARAMVALUE);
      fmav_param_union_t param_union;
      if (fmav_param_get_param_union(&param_union, _pv_index)) {
        generateParamValue(fmav_param_list[_pv_index].name, param_union.p_float, param_union.type, FASTMAVLINK_PARAM_NUM, _pv_index);
      }
      return; //do only one per loop
    }

    // other TASKS low priority
    if (doTaskAutopilotLowPriority()) return;
    if (doTaskCameraLowPriority()) return;
    if (doTaskQShot()) return;
  }
}

// -- Wakeup call from OpenTx --
// this is the main entry point

// ourself = link 0
// serial1 = link 1
// serial2 = link 2
// usb     = link 3

void MavlinkTelem::wakeup()
{
  // track configuration changes
  bool aux1_enabled = (g_eeGeneral.auxSerialMode == UART_MODE_MAVLINK);
  bool aux2_enabled = (g_eeGeneral.aux2SerialMode == UART_MODE_MAVLINK);
#if defined(TELEMETRY_MAVLINK_USB_SERIAL)
  bool usb_enabled = (getSelectedUsbMode() == USB_MAVLINK_MODE);
#else
  bool usb_enabled = false;
#endif
  bool external_enabled = isModuleMavlink(EXTERNAL_MODULE);

  if ((_aux1_enabled != aux1_enabled) || (_aux2_enabled != aux2_enabled) ||
      (_aux1_baudrate != g_eeGeneral.mavlinkBaudrate) || (_aux2_baudrate != g_eeGeneral.mavlinkBaudrate2) ||
      (_external_enabled != external_enabled)) {
    _aux1_enabled = aux1_enabled;
    _aux2_enabled = aux2_enabled;
    _aux1_baudrate = g_eeGeneral.mavlinkBaudrate;
    _aux2_baudrate = g_eeGeneral.mavlinkBaudrate2;
    _external_enabled = external_enabled;
    map_serials();
    _reset();
  }

  if (_usb_enabled != usb_enabled) {
    _usb_enabled = usb_enabled;
    fmav_router_reset_link(3);
  }

  if (moduleState[EXTERNAL_MODULE].protocol == PROTOCOL_CHANNELS_MAVLINK) mavlinkTelemExternal_wakeup();

  // skip out if not one of the serial1, serial2 is enabled
  if (!serial1_enabled && !serial2_enabled) return;

  // look for incoming messages on all channels
  // only do one at a time
  #define INCc(x,p)  {x++; if(x >= p) x = 0;}

  INCc(_scheduled_serial, 3);
  uint8_t currently_scheduled_serial = _scheduled_serial;

  uint32_t available = 0;
  switch (currently_scheduled_serial) {
    case 0: available = mavlinkTelem1Available(); break;
    case 1: available = mavlinkTelem2Available(); break;
    case 2: available = mavlinkTelem3Available(); break;
  }
  if (available > 128) available = 128; // 128 = 22 ms @ 57600 bps

  uint8_t c;
  fmav_result_t result;

  // read serial1
  if (currently_scheduled_serial == 0) {
    for (uint32_t i = 0; i < available; i++) {
      if (!mavlinkTelem1Getc(&c)) break;
      if (fmav_parse_and_check_to_frame_buf(&result, _buf1, &_status1, c)) {
        fmav_router_handle_message(1, &result);
        if (fmav_router_send_to_link(1)) {} // WE DO NOT REFLECT, SO THIS MUST NEVER HAPPEN !!
        if (fmav_router_send_to_link(2)) { mavlinkTelem2PutBuf(_buf1, result.frame_len); }
        if (fmav_router_send_to_link(3)) { mavlinkTelem3PutBuf(_buf1, result.frame_len); }
        if (result.res == FASTMAVLINK_PARSE_RESULT_OK && fmav_router_send_to_link(0)) {
          fmav_frame_buf_to_msg(&_msg, &result, _buf1);
          handleMessage(); // checks _msg, and puts any result into a task queue
        }
      }
    }
  }

  // read serial2
  if (currently_scheduled_serial == 1) {
    for (uint32_t i = 0; i < available; i++) {
      if (!mavlinkTelem2Getc(&c)) break;
      if (fmav_parse_and_check_to_frame_buf(&result, _buf2, &_status2, c)) {
        fmav_router_handle_message(2, &result);
        if (fmav_router_send_to_link(1)) { mavlinkTelem1PutBuf(_buf2, result.frame_len); }
        if (fmav_router_send_to_link(2)) {} // WE DO NOT REFLECT, SO THIS MUST NEVER HAPPEN !!
        if (fmav_router_send_to_link(3)) { mavlinkTelem3PutBuf(_buf2, result.frame_len); }
        if (result.res == FASTMAVLINK_PARSE_RESULT_OK && fmav_router_send_to_link(0)) {
          fmav_frame_buf_to_msg(&_msg, &result, _buf2);
          handleMessage(); // checks _msg, and puts any result into a task queue
        }
      }
    }
  }

  // read serial3 = usb
  if (currently_scheduled_serial == 2) {
    for (uint32_t i = 0; i < available; i++) {
      if (!mavlinkTelem3Getc(&c)) break;
      if (fmav_parse_and_check_to_frame_buf(&result, _buf3, &_status3, c)) {
        fmav_router_handle_message(3, &result);
        if (fmav_router_send_to_link(1)) { mavlinkTelem1PutBuf(_buf3, result.frame_len); }
        if (fmav_router_send_to_link(2)) { mavlinkTelem2PutBuf(_buf3, result.frame_len); }
        if (fmav_router_send_to_link(3)) {} // WE DO NOT REFLECT, SO THIS MUST NEVER HAPPEN !!
        if (result.res == FASTMAVLINK_PARSE_RESULT_OK && fmav_router_send_to_link(0)) {
          fmav_frame_buf_to_msg(&_msg, &result, _buf3);
          handleMessage(); // checks _msg, and puts any result into a task queue
        }
      }
    }
  }

  // do tasks
  doTask(); // checks task queue _msg, and puts one result into _msg_out

  // send out pending message
  if (_msg_out_available) {
    fmav_router_handle_message_by_msg(0, &_msg_out);
    if (fmav_router_send_to_link(1) || fmav_router_send_to_link(2) || fmav_router_send_to_link(3)) {
      uint16_t count = fmav_msg_to_frame_buf(_buf_out, &_msg_out);
      // check that message can be send to all enabled serials
      if ((!serial1_enabled || mavlinkTelem1HasSpace(count)) &&
          (!serial2_enabled || mavlinkTelem2HasSpace(count)) &&
          (!_usb_enabled || mavlinkTelem3HasSpace(count))) {
        if (serial1_enabled && fmav_router_send_to_link(1)) mavlinkTelem1PutBuf(_buf_out, count);
        if (serial2_enabled && fmav_router_send_to_link(2)) mavlinkTelem2PutBuf(_buf_out, count);
        if (_usb_enabled && fmav_router_send_to_link(3)) mavlinkTelem3PutBuf(_buf_out, count);
        _msg_out_available = false;
        msg_tx_count++;
        _msg_tx_persec_cnt++;
        _bytes_tx_persec_cnt += count;
      }
    } else {
      _msg_out_available = false; // message is targeted at unknown component
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

  // keep 10us timer updated
  time10us();
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
  // sanitize g_eeGeneral
#if defined(CLI) || defined(DEBUG)
#define UART_MODE_NONE_OR_DEBUG UART_MODE_DEBUG
#else
#define UART_MODE_NONE_OR_DEBUG UART_MODE_NONE
#endif
#if !defined(AUX_SERIAL)
  if (g_eeGeneral.auxSerialMode == UART_MODE_MAVLINK) g_eeGeneral.auxSerialMode = UART_MODE_NONE_OR_DEBUG;
#endif
#if !defined(AUX2_SERIAL)
  if (g_eeGeneral.aux2SerialMode == UART_MODE_MAVLINK) g_eeGeneral.aux2SerialMode = UART_MODE_NONE_OR_DEBUG;
#endif

  _my_sysid = MAVLINK_TELEM_MY_SYSID;
  _my_compid = MAVLINK_TELEM_MY_COMPID;

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
  radio.rssi_voice_critical_disabled = false;
  radio.rssi_voice_telemetryok_disabled = false;

  _resetAutopilot();
  _resetGimbalAndGimbalClient();
  _resetCamera();

  _resetQShot();

  fmav_status_reset(&_status1);
  fmav_status_reset(&_status2);
  fmav_status_reset(&_status3);
  fmav_status_reset(&_status_out);
  fmav_router_reset();
  fmav_router_add_ourself(MAVLINK_TELEM_MY_SYSID, MAVLINK_TELEM_MY_COMPID);

  msg_rx_count = 0;
  msg_rx_persec = 0;
  bytes_rx_persec = 0;
  _msg_rx_persec_cnt = 0;
  _bytes_rx_persec_cnt = 0;

  msg_tx_count = 0;
  msg_tx_persec = 0;
  bytes_tx_persec = 0;
  _msg_tx_persec_cnt = 0;
  _bytes_tx_persec_cnt = 0;

  _txgps_has_pos_int_fix = false;

  mavapiInit();
}


void MavlinkTelem::_init(void)
{
  fmav_router_init();
  fmav_router_set_link_properties_all(
      FASTMAVLINK_ROUTER_LINK_PROPERTY_FLAG_ALWAYS_SEND_HEARTBEAT |
      FASTMAVLINK_ROUTER_LINK_PROPERTY_FLAG_DISCOVER_BY_HEARTBEAT
      );
  _reset();
  _mavlink_copy_g2p();
}
