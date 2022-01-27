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
  lua_pushtableinteger(L, "LQ", mBridge.link_stats.LQ);
  lua_pushtableinteger(L, "rssi_inst", mBridge.link_stats.rssi_instantaneous);
  lua_pushtableinteger(L, "snr_inst", mBridge.link_stats.snr_instantaneous);
  lua_pushtableinteger(L, "rssi2_inst", mBridge.link_stats.rssi2_instantaneous);
  lua_pushtableinteger(L, "ant_no", mBridge.link_stats.ant_no);
  lua_pushtableinteger(L, "rssi_filt", mBridge.link_stats.rssi_filtered);
  lua_pushtableinteger(L, "snr_filt", mBridge.link_stats.snr_filtered);
  lua_pushtableinteger(L, "rssi2_filt", mBridge.link_stats.rssi2_filtered);

  lua_pushtableinteger(L, "rx_LQ", mBridge.link_stats.receiver_LQ);
  lua_pushtableinteger(L, "rx_LQ_serial", mBridge.link_stats.receiver_LQ_serial);
  lua_pushtableinteger(L, "rx_rssi_inst", mBridge.link_stats.receiver_rssi_instantaneous);
  lua_pushtableinteger(L, "rx_rssi2_inst", mBridge.link_stats.receiver_rssi2_instantaneous);
  lua_pushtableinteger(L, "rx_ant_no", mBridge.link_stats.receiver_ant_no);
  lua_pushtableinteger(L, "rx_rssi_filt", mBridge.link_stats.receiver_rssi_filtered);
  lua_pushtableinteger(L, "rx_rssi2_filt", mBridge.link_stats.receiver_rssi2_filtered);

  lua_pushtableinteger(L, "LQ_serial_transmitted", mBridge.link_stats.LQ_fresh_serial_packets_transmitted);
  lua_pushtableinteger(L, "byte_rate_transmitted", mBridge.link_stats.bytes_per_sec_transmitted);
  lua_pushtableinteger(L, "LQ_valid_received", mBridge.link_stats.LQ_valid_received);
  lua_pushtableinteger(L, "LQ_serial_received", mBridge.link_stats.LQ_fresh_serial_packets_received);
  lua_pushtableinteger(L, "byte_rate_received", mBridge.link_stats.bytes_per_sec_received);
  lua_pushtableinteger(L, "LQ_frames_received", mBridge.link_stats.LQ_received);
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

