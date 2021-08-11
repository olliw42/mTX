/*
 * The MAVLink for OpenTx project
 * (c) www.olliw.eu, OlliW, OlliW42
 */

#include "opentx.h"

// -- MAVLINK API --

// -- Receive stuff --

// we probably need to differentiate not only by msgid, but also by sysid-compid
// if two components send the same message at (too) high rate considering only msgid leads to message loss

uint8_t MavlinkTelem::_mavapiMsgInFindOrAdd(uint32_t msgid)
{
  for (uint8_t i = 0; i < MAVMSGLIST_MAX; i++) {
    if (!_mavapi_rx_list[i]) continue;
    if (_mavapi_rx_list[i]->msgid == msgid) { return i; }
  }
  for (uint8_t i = 0; i < MAVMSGLIST_MAX; i++) {
    if (_mavapi_rx_list[i]) continue;
    // free spot, so add it
    _mavapi_rx_list[i] = (MavMsg*)malloc(sizeof(MavMsg));
    if (!_mavapi_rx_list[i]) return UINT8_MAX; // grrrr
    _mavapi_rx_list[i]->msgid = msgid;
    _mavapi_rx_list[i]->payload_ptr = NULL;
    _mavapi_rx_list[i]->updated = false;
    return i;
  }
  return UINT8_MAX;
}

void MavlinkTelem::mavapiHandleMessage(fmav_message_t* msg)
{
  if (!_mavapi_rx_enabled) return;

  uint8_t i = _mavapiMsgInFindOrAdd(msg->msgid);
  if (i == UINT8_MAX) return;

  if (!_mavapi_rx_list[i]->payload_ptr) _mavapi_rx_list[i]->payload_ptr = malloc(msg->payload_max_len);
  if (!_mavapi_rx_list[i]->payload_ptr) return; // grrrr

  _mavapi_rx_list[i]->sysid = msg->sysid;
  _mavapi_rx_list[i]->compid = msg->compid;
  _mavapi_rx_list[i]->target_sysid = msg->target_sysid;
  _mavapi_rx_list[i]->target_compid = msg->target_compid;
  memcpy(_mavapi_rx_list[i]->payload_ptr, msg->payload, msg->payload_max_len);
  _mavapi_rx_list[i]->updated = true;
  _mavapi_rx_list[i]->timestamp = time10us();
}

void MavlinkTelem::mavapiMsgInEnable(bool flag)
{
  _mavapi_rx_enabled = flag;
}

uint8_t MavlinkTelem::mavapiMsgInCount(void)
{
  if (!_mavapi_rx_enabled) return 0;

  uint8_t cnt = 0;
  for (uint8_t i = 0; i < MAVMSGLIST_MAX; i++) if(_mavapi_rx_list[i]) cnt++;
  return cnt;
}

MavlinkTelem::MavMsg* MavlinkTelem::mavapiMsgInGet(uint32_t msgid)
{
  if (!_mavapi_rx_enabled) return NULL;

  uint8_t i_found = UINT8_MAX;
  for (uint8_t i = 0; i < MAVMSGLIST_MAX; i++) {
    if (!_mavapi_rx_list[i]) continue;
    if (!_mavapi_rx_list[i]->payload_ptr) continue; // it must have been received completely
    if (_mavapi_rx_list[i]->msgid == msgid) { i_found = i; }
  }
  if (i_found == UINT8_MAX) return NULL;
  return _mavapi_rx_list[i_found];
}

MavlinkTelem::MavMsg* MavlinkTelem::mavapiMsgInGetLast(void)
{
  if (!_mavapi_rx_enabled) return NULL;

  uint32_t t_max = 0;
  uint8_t i_found = UINT8_MAX;
  for (uint8_t i = 0; i < MAVMSGLIST_MAX; i++) {
    if (!_mavapi_rx_list[i]) continue;
    if (!_mavapi_rx_list[i]->payload_ptr) continue; // it must have been received completely
    if (_mavapi_rx_list[i]->timestamp > t_max) { t_max = _mavapi_rx_list[i]->timestamp; i_found = i; }
  }
  if (i_found == UINT8_MAX) return NULL;
  return _mavapi_rx_list[i_found];
}

// -- Send stuff --

void MavlinkTelem::mavapiMsgOutEnable(bool flag)
{
  _mavapi_tx_enabled = flag;

  if (_mavapi_tx_enabled && _mavapiMsgOutFifo == NULL) {
    _mavapiMsgOutFifo = (fmav_message_t*)malloc(sizeof(fmav_message_t) * MAVOUTFIFO_MAX);
    if (!_mavapiMsgOutFifo) _mavapi_tx_enabled = false; // grrrr
  }
}


// returns the pointer into which we should write, without advancing write index, probe()-like
fmav_message_t* MavlinkTelem::mavapiMsgOutPtr(void)
{
  if (!_mavapi_tx_enabled) return NULL;

  uint32_t wi_next = (_wi + 1) & (MAVOUTFIFO_MAX - 1);
  if (wi_next == _ri) return NULL; // blocking push, push not allowed if full
  return &(_mavapiMsgOutFifo[_wi]);
}

// advances write index, and sets task, push()-like
void MavlinkTelem::mavapiMsgOutSet(void)
{
  if (!_mavapi_tx_enabled) return;

  _wi = (_wi + 1) & (MAVOUTFIFO_MAX - 1);
  //SETTASK(TASK_ME, TASK_SENDMSG_MAVLINK_API);
}

// generate from msg at read index, and advance read index, pop()-like
void MavlinkTelem::mavapiGenerateMessage(void)
{
  if (!_mavapi_tx_enabled) return;

  if (_wi == _ri) return; // empty

  fmav_message_t* msgptr = &(_mavapiMsgOutFifo[_ri]);
  _ri = (_ri + 1) & (MAVOUTFIFO_MAX - 1);
  memcpy(&_msg_out, msgptr, sizeof(fmav_message_t));
  fmav_finalize_msg(&_msg_out, &_status_out);
  _msg_out_available = true;
}

bool MavlinkTelem::mavapiMsgOutEmpty(void)
{
  return (_wi == _ri);
}

// -- MAVLINK API PARAMETERS --

//returns an index into the _paramInList
uint8_t MavlinkTelem::_param_find(uint8_t sysid, uint8_t compid, const char* param_id)
{
  for(uint8_t i = 0; i < _paramInList_count; i++) {
    if ((_paramInList[i].sysid == sysid) &&
        (_paramInList[i].compid == compid) &&
        !strncmp(_paramInList[i].id, param_id, 16)) { return i; }
  }
  return UINT8_MAX;
}

void MavlinkTelem::paramHandleMessage(fmav_message_t* msg)
{
  if (!_paramInList_count) return;

  if (msg->msgid != FASTMAVLINK_MSG_ID_PARAM_VALUE) return;

  fmav_param_value_t payload;
  fmav_msg_param_value_decode(&payload, msg);

  uint8_t i = _param_find(msg->sysid, msg->compid, payload.param_id);
  if (i == UINT8_MAX) return;

  _paramInList[i].value = payload.param_value;
  //_paramInList[i].type = payload.param_type;
  _paramInList[i].updated = true;
}

void MavlinkTelem::paramGenerateMessage(void)
{
  struct ParamItem p;

  if (!_paramOutFifo.pop(p)) return;

  if (p.request_or_set) {
    generateParamSet(p.sysid, p.compid, p.id, p.value, p.type);
  }
  else {
    generateParamRequestRead(p.sysid, p.compid, p.id);
  }
}

//-- for lua interface

uint8_t MavlinkTelem::registerParam(uint8_t sysid, uint8_t compid, const char* param_id, uint8_t param_type)
{
  if (_paramInList_count >= MAVPARAMLIST_MAX) return UINT8_MAX;

  //we check if it is already in list, to avoid multiple registration
  uint8_t i = _param_find(sysid, compid, param_id);
  if (i != UINT8_MAX) return UINT8_MAX;

  i = _paramInList_count;

  _paramInList[i].sysid = sysid;
  _paramInList[i].compid = compid;
  memset(_paramInList[i].id, 0, 17);
  strncpy(_paramInList[i].id, param_id, 16);

  _paramInList[i].value = 0.0f;
  _paramInList[i].type = param_type;
  _paramInList[i].updated = false;
  _paramInList[i].request_or_set = false;

  _paramInList_count++;

  return i;
}

MavlinkTelem::ParamItem* MavlinkTelem::getParamValue(uint8_t i)
{
  if (i >= _paramInList_count) return NULL;
  return &(_paramInList[i]);
}

bool MavlinkTelem::sendParamRequest(uint8_t i)
{
  if (i >= _paramInList_count) return false;

  if (_paramOutFifo.isFull()) return false;

  struct ParamItem p;
  p.sysid = _paramInList[i].sysid;
  p.compid = _paramInList[i].compid;
  strcpy(p.id, _paramInList[i].id);
  p.request_or_set = false;
  _paramOutFifo.push(p);
  //SETTASK(TASK_ME, TASK_SENDMSG_MAVLINK_PARAM);

  return true;
}

bool MavlinkTelem::sendParamSet(uint8_t i, float param_value)
{
  if (i >= _paramInList_count) return false;

  if (_paramOutFifo.isFull()) return false;

  struct ParamItem p;
  p.sysid = _paramInList[i].sysid;
  p.compid = _paramInList[i].compid;
  strcpy(p.id, _paramInList[i].id);
  p.value = param_value;
  p.type = _paramInList[i].type;
  p.request_or_set = true;
  _paramOutFifo.push(p);
  //SETTASK(TASK_ME, TASK_SENDMSG_MAVLINK_PARAM);

  return true;
}

bool MavlinkTelem::paramIsArdupilot(uint8_t i)
{
  if (i >= _paramInList_count) return false;

  return (_paramInList[i].compid == autopilot.compid) &&
         (_paramInList[i].sysid == systemSysId()) &&
         (autopilottype == MAV_AUTOPILOT_ARDUPILOTMEGA);
}




