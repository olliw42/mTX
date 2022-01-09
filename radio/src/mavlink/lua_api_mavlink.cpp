//*******************************************************
// MavTelem Library
// Copyright (c) OlliW, OlliW42, www.olliw.eu
// LGPL3
// https://www.gnu.org/licenses/lgpl-3.0.en.html
//*******************************************************

#include <ctype.h>
#include <stdio.h>
#include "opentx.h"
#include "../lua/lua_api.h"
#include "thirdparty/Mavlink/opentx_lua_lib_constants.h"
#include "thirdparty/Mavlink/opentx_lua_lib_messages.h"


static int luaMavlinkGetVersion(lua_State * L)
{
  lua_pushinteger(L, FASTMAVLINK_MAVLINK_VERSION); // this is the version reported also by the heartbeat
  return 1;
}

static int luaMavlinkGetChannelStatus(lua_State * L)
{
  lua_createtable(L, 0, 6);
  lua_pushtableinteger(L, "msg_rx_count", mavlinkTelem.msg_rx_count);
  lua_pushtableinteger(L, "msg_rx_per_sec", mavlinkTelem.msg_rx_persec);
  lua_pushtableinteger(L, "bytes_rx_per_sec", mavlinkTelem.bytes_rx_persec);
  lua_pushtableinteger(L, "msg_tx_count", mavlinkTelem.msg_tx_count);
  lua_pushtableinteger(L, "msg_tx_per_sec", mavlinkTelem.msg_tx_persec);
  lua_pushtableinteger(L, "bytes_tx_per_sec", mavlinkTelem.bytes_tx_persec);
  return 1;
}

//-- some statistics --

extern int _end;
extern int _heap_end;
extern unsigned char *heap;

static int luaMavlinkGetMemUsed(lua_State * L)
{
  uint32_t s = luaGetMemUsed(lsScripts);
#if defined(COLORLCD)
  uint32_t w = luaGetMemUsed(lsWidgets);
  uint32_t e = luaExtraMemoryUsage;
#else
  uint32_t w = 0;
  uint32_t e = 0;
#endif
  lua_createtable(L, 0, 6);
  lua_pushtableinteger(L, "scripts", s);
  lua_pushtableinteger(L, "widgets", w);
  lua_pushtableinteger(L, "extra", e);
  lua_pushtableinteger(L, "total", s+w+e);
  lua_pushtableinteger(L, "heap_used", (int)(heap - (unsigned char *)&_end));
  lua_pushtableinteger(L, "heap_free", (int)((unsigned char *)&_heap_end - heap));
  return 1;
}

static int luaMavlinkGetStackUsed(lua_State * L)
{
  lua_createtable(L, 0, 10);
  lua_pushtableinteger(L, "main_available", stackAvailable()*4);
  lua_pushtableinteger(L, "main_size", stackSize()*4);
  lua_pushtableinteger(L, "menus_available", menusStack.available()*4);
  lua_pushtableinteger(L, "menus_size", menusStack.size());
  lua_pushtableinteger(L, "mixer_available", mixerStack.available()*4);
  lua_pushtableinteger(L, "mixer_size", mixerStack.size());
  lua_pushtableinteger(L, "audio_available", audioStack.available()*4);
  lua_pushtableinteger(L, "audio_size", audioStack.size());
  lua_pushtableinteger(L, "mavlink_available", mavlinkStack.available()*4);
  lua_pushtableinteger(L, "mavlink_size", mavlinkStack.size());
  return 1;
}

static int luaMavlinkGetTaskStats(lua_State *L)
{
  lua_newtable(L);
  lua_pushtableinteger(L, "time", mavlinkTaskRunTime());
  lua_pushtableinteger(L, "max", mavlinkTaskRunTimeMax());
  lua_pushtableinteger(L, "loop", mavlinkTaskLoop());
  return 1;
}

//-- mavlink api --

static int luaMavlinkGetMyIds(lua_State *L)
{
  lua_pushinteger(L, mavlinkTelem.mySysId());
  lua_pushinteger(L, mavlinkTelem.myCompId());
  return 2;
}

static int luaMavlinkGetSystemId(lua_State *L)
{
  if (mavlinkTelem.isSystemIdValid())
    lua_pushinteger(L, mavlinkTelem.systemSysId());
  else
    lua_pushnil(L);
  return 1;
}

static int luaMavlinkGetAutopilotIds(lua_State *L)
{
  if (mavlinkTelem.isSystemIdValid()) {
    lua_pushinteger(L, mavlinkTelem.systemSysId());
    if (mavlinkTelem.autopilot.compid > 0) {
      lua_pushinteger(L, mavlinkTelem.autopilot.compid);
    }
    else
      lua_pushnil(L);
  }
  else {
    lua_pushnil(L);
    lua_pushnil(L);
  }
  return 2;
}

static int luaMavlinkGetCameraIds(lua_State *L)
{
  if (mavlinkTelem.isSystemIdValid()) {
    lua_pushinteger(L, mavlinkTelem.systemSysId());
    if (mavlinkTelem.camera.compid > 0)
      lua_pushinteger(L, mavlinkTelem.camera.compid);
    else
      lua_pushnil(L);
  }
  else {
    lua_pushnil(L);
    lua_pushnil(L);
  }
  return 2;
}

static int luaMavlinkGetGimbalIds(lua_State *L)
{
  if (mavlinkTelem.isSystemIdValid()) {
    lua_pushinteger(L, mavlinkTelem.systemSysId());
    if (mavlinkTelem.gimbal.compid > 0)
      lua_pushinteger(L, mavlinkTelem.gimbal.compid);
    else
      lua_pushnil(L);
  }
  else {
    lua_pushnil(L);
    lua_pushnil(L);
  }
  return 2;
}

static int luaMavlinkGetGimbalManagerIds(lua_State *L)
{
  if (mavlinkTelem.isSystemIdValid()) {
    lua_pushinteger(L, mavlinkTelem.systemSysId());
    if (mavlinkTelem.gimbalmanager.compid > 0)
      lua_pushinteger(L, mavlinkTelem.gimbalmanager.compid);
    else
      lua_pushnil(L);
  }
  else {
    lua_pushnil(L);
    lua_pushnil(L);
  }
  return 2;
}

//-- mavlink api, messages --

static int luaMavlinkInEnable(lua_State *L)
{
  bool flag = LUAL_CHECKBOOLEAN(L, 1); //(luaL_checkinteger(L, 1) > 0);
  mavlinkTelem.mavapiMsgInEnable(flag);
  return 0;
}

static int luaMavlinkInCount(lua_State *L)
{
  lua_pushinteger(L, mavlinkTelem.mavapiMsgInCount());
  return 1;
}

static int luaMavlinkGetMessage(lua_State *L)
{
  int msgid = luaL_checknumber(L, 1);

  MavlinkTelem::MavMsg* mavmsg = mavlinkTelem.mavapiMsgInGet(msgid);
  if (!mavmsg) {
    lua_pushnil(L);
  }
  else {
    luaMavlinkPushMavMsg(L, mavmsg);
    mavmsg->updated = false;
  }
  return 1;
}

static int luaMavlinkGetMessageLast(lua_State *L)
{
  MavlinkTelem::MavMsg* mavmsg = mavlinkTelem.mavapiMsgInGetLast();
  if (!mavmsg) {
    lua_pushnil(L);
  }
  else {
    luaMavlinkPushMavMsg(L, mavmsg);
  }
  return 1;
}

static int luaMavlinkOutEnable(lua_State *L)
{
  bool flag = LUAL_CHECKBOOLEAN(L, 1); //(luaL_checkinteger(L, 1) > 0);
  mavlinkTelem.mavapiMsgOutEnable(flag);
  return 0;
}

static int luaMavlinkIsFree(lua_State *L)
{
  lua_pushboolean(L, (mavlinkTelem.mavapiMsgOutPtr() != NULL));
  return 1;
}

static int luaMavlinkSendMessage(lua_State *L)
{
  // we enable it the moment we use sendMessage() the first time, more user convenient
  if (!mavlinkTelem._mavapi_tx_enabled) mavlinkTelem.mavapiMsgOutEnable(true);

  fmav_message_t* msg_out = mavlinkTelem.mavapiMsgOutPtr();

  if (!lua_istable(L, -1) || !msg_out) {
    lua_pushnil(L);
  }
  else if (luaMavlinkCheckMsgOut(L, msg_out)) {
    mavlinkTelem.mavapiMsgOutSet();
    lua_pushboolean(L, true);
  }
  else {
    lua_pushboolean(L, false);
  }
  return 1;
}

//-- mavlink api, parameters --

static int luaMavlinkMemcpyToInteger(lua_State *L)
{
  float f = luaL_checknumber(L, 1);
  uint32_t i;
  memcpy(&i, &f, 4);
  lua_pushinteger(L, i);
  return 1;
}

static int luaMavlinkMemcpyToNumber(lua_State *L)
{
  uint32_t i = luaL_checkinteger(L, 1);
  float f;
  memcpy(&f, &i, 4);
  lua_pushnumber(L, f);
  return 1;
}

static int luaMavlinkRegisterParam(lua_State *L)
{
  int sysid = luaL_checkinteger(L, 1);
  int compid = luaL_checkinteger(L, 2);
  const char* param_id = luaL_checkstring(L, 3);
  int param_type = luaL_checkinteger(L, 4);

  uint8_t handle = mavlinkTelem.registerParam(sysid, compid, param_id, param_type);

  if (handle < UINT8_MAX) {
    lua_pushinteger(L, handle);
  }
  else{
    lua_pushnil(L);
  }
  return 1;
}

static int luaMavlinkClearParamRegister(lua_State *L)
{
  mavlinkTelem._paramInList_count = 0;
  mavlinkTelem._paramOutFifo.clear();
  return 0;
}

static int luaMavlinkGetParamValue(lua_State *L)
{
  int handle = luaL_checkinteger(L, 1);
  int type = luaL_optunsigned(L, 3, -1); //<0: convert automatically, 0: follow standard, >0: ardupilot

  MavlinkTelem::ParamItem* p = mavlinkTelem.getParamValue(handle);
  if (p) {

    if ((type == 0) || ((type < 0) && !mavlinkTelem.paramIsArdupilot(handle))) {
        //convert it
    }

    lua_newtable(L);
    lua_pushtablenumber(L, "param_value", p->value);
    lua_pushtablestring(L, "param_id", p->id);
    lua_pushtableboolean(L, "updated", p->updated);
    p->updated = false;
  }
  else {
    lua_pushnil(L);
  }
  return 1;
}

static int luaMavlinkSendParamRequest(lua_State *L)
{
  int handle = luaL_checkinteger(L, 1);

  lua_pushboolean(L, mavlinkTelem.sendParamRequest(handle));
  return 1;
}

static int luaMavlinkSendParamSet(lua_State *L)
{
  int handle = luaL_checknumber(L, 1);
  float param_value = luaL_checknumber(L, 2);
  int type = luaL_optunsigned(L, 3, -1); //<0: convert automatically, 0: follow standard, >0: ardupilot

  if ((type == 0) || ((type < 0) && !mavlinkTelem.paramIsArdupilot(handle))) {
    //convert it
  }

  lua_pushboolean(L, mavlinkTelem.sendParamSet(handle, param_value));
  return 1;
}


//------------------------------------------------------------
// mavlink luaL and luaR arrays
//------------------------------------------------------------

const luaL_Reg mavlinkLib[] = {
  { "getVersion", luaMavlinkGetVersion },
  { "getChannelStatus", luaMavlinkGetChannelStatus },
  { "getMemUsed", luaMavlinkGetMemUsed },
  { "getStackUsed", luaMavlinkGetStackUsed },
  { "getTaskStats", luaMavlinkGetTaskStats },

  { "getMyIds", luaMavlinkGetMyIds },
  { "getSystemId", luaMavlinkGetSystemId },
  { "getAutopilotIds", luaMavlinkGetAutopilotIds },
  { "getCameraIds", luaMavlinkGetCameraIds },
  { "getGimbalIds", luaMavlinkGetGimbalIds },
  { "getGimbalManagerIds", luaMavlinkGetGimbalManagerIds },

  { "enableIn", luaMavlinkInEnable },
  { "getInCount", luaMavlinkInCount },
  { "getMessage", luaMavlinkGetMessage },
  { "getMessageLast", luaMavlinkGetMessageLast },
  { "enableOut", luaMavlinkOutEnable },
  { "isFree", luaMavlinkIsFree },
  { "sendMessage", luaMavlinkSendMessage },
  { "memcpyToInteger", luaMavlinkMemcpyToInteger },
  { "memcpyToNumber", luaMavlinkMemcpyToNumber },
  { "registerParam", luaMavlinkRegisterParam },
  { "clearParamRegister", luaMavlinkClearParamRegister },
  { "getParamValue", luaMavlinkGetParamValue },
  { "sendParamRequest", luaMavlinkSendParamRequest },
  { "sendParamSet", luaMavlinkSendParamSet },

  { nullptr, nullptr }  /* sentinel */
};

const luaR_value_entry mavlinkConstants[] = {
  MAVLINK_LIB_CONSTANTS

  { nullptr, 0 }  /* sentinel */
};

