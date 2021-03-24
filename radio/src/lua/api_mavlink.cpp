/*
 * Copyright (C) OpenTX
 *
 * Based on code named
 *   th9x - http://code.google.com/p/th9x
 *   er9x - http://code.google.com/p/er9x
 *   gruvin9x - http://code.google.com/p/gruvin9x
 *
 * License GPLv2: http://www.gnu.org/licenses/gpl-2.0.html
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
/*
 *  (c) www.olliw.eu, OlliW, OlliW42
 */

#include <ctype.h>
#include <stdio.h>
#include "opentx.h"
#include "lua_api.h"
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

//-- mavlink api, messages --

static int luaMavlinkMessageEnable(lua_State *L)
{
  bool flag = (luaL_checkinteger(L, 1) > 0);
  mavlinkTelem.mavMsgListEnable(flag);
  return 0;
}

static int luaMavlinkMessageCount(lua_State *L)
{
  lua_pushinteger(L, mavlinkTelem.mavMsgListCount());
  return 1;
}

static int luaMavlinkGetMessage(lua_State *L)
{
  int msgid = luaL_checknumber(L, 1);

  MavlinkTelem::MavMsg* mavmsg = mavlinkTelem.mavMsgListGet(msgid);
  if (!mavmsg) {
    lua_pushnil(L);
  }
  else {
    luaMavlinkPushMavMsg(L, mavmsg);
  }
  return 1;
}

static int luaMavlinkGetMessageLast(lua_State *L)
{
  MavlinkTelem::MavMsg* mavmsg = mavlinkTelem.mavMsgListGetLast();
  if (!mavmsg) {
    lua_pushnil(L);
  }
  else {
    luaMavlinkPushMavMsg(L, mavmsg);
  }
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

  { "messageEnable", luaMavlinkMessageEnable },
  { "getMessageCount", luaMavlinkMessageCount },
  { "getMessage", luaMavlinkGetMessage },
  { "getMessageLast", luaMavlinkGetMessageLast },

  { nullptr, nullptr }  /* sentinel */
};

const luaR_value_entry mavlinkConstants[] = {
  MAVLINK_LIB_CONSTANTS
  { nullptr, 0 }  /* sentinel */
};

