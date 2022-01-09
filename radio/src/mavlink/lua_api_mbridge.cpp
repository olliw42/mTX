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

//-- mbridge api --

static int luaMBridgeGetLinkStats(lua_State * L)
{
  lua_newtable(L);
  lua_pushtableinteger(L, "rssi", mBridge.link_stats.rssi);
  lua_pushtableinteger(L, "LQ", mBridge.link_stats.LQ);
  lua_pushtableinteger(L, "snr", mBridge.link_stats.snr);
  lua_pushtableinteger(L, "rx_rssi", mBridge.link_stats.receiver_rssi);
  lua_pushtableinteger(L, "rx_LQ", mBridge.link_stats.receiver_LQ);
  // only momentarily for debug
  lua_pushtableinteger(L, "LQ_frames_received", mBridge.link_stats.LQ_frames_received);
  lua_pushtableinteger(L, "LQ_received", mBridge.link_stats.LQ_received);
  lua_pushtableinteger(L, "LQ_valid_received", mBridge.link_stats.LQ_valid_received);
  return 1;
}

//------------------------------------------------------------
// mbridge luaL and luaR arrays
//------------------------------------------------------------

const luaL_Reg mbridgeLib[] = {
  { "getLinkStats", luaMBridgeGetLinkStats },

  { nullptr, nullptr }  /* sentinel */
};

const luaR_value_entry mbridgeConstants[] = {

  { nullptr, 0 }  /* sentinel */
};

