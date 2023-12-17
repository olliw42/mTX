//*******************************************************
// mTX: MAVLink for OpenTx Project
// Copyright (c) OlliW, OlliW42, www.olliw.eu
// LGPL3
// https://www.gnu.org/licenses/lgpl-3.0.en.html
// MavTelem Library
//*******************************************************

#include <ctype.h>
#include <stdio.h>
#include "opentx.h"
#include "../lua/lua_api.h"

//-- mbridge api --

static int luaMBridgeEnabled(lua_State * L)
{
  lua_pushboolean(L, (moduleState[EXTERNAL_MODULE].protocol == PROTOCOL_CHANNELS_MBRIDGE));
  return 1;
}

static int luaMBridgeGetLinkStats(lua_State * L)
{
  lua_newtable(L);
  lua_pushtableinteger(L, "LQ", mBridge.link_stats.LQ);
  lua_pushtableinteger(L, "rssi1_inst", mBridge.link_stats.rssi1_instantaneous);
  lua_pushtableinteger(L, "rssi2_inst", mBridge.link_stats.rssi2_instantaneous);

  lua_pushtableinteger(L, "rssi_inst", (mBridge.link_stats.receive_antenna == 0) ? mBridge.link_stats.rssi1_instantaneous : mBridge.link_stats.rssi2_instantaneous);

  lua_pushtableinteger(L, "snr_inst", mBridge.link_stats.snr_instantaneous);
  lua_pushtableinteger(L, "receive_antenna", mBridge.link_stats.receive_antenna);
  lua_pushtableinteger(L, "transmit_antenna", mBridge.link_stats.transmit_antenna);
  lua_pushtableinteger(L, "diversity", mBridge.link_stats.diversity);

  lua_pushtableinteger(L, "rssi1_filt", mBridge.link_stats.rssi1_filtered);
  lua_pushtableinteger(L, "rssi2_filt", mBridge.link_stats.rssi2_filtered);
  lua_pushtableinteger(L, "snr_filt", mBridge.link_stats.snr_filtered);

  lua_pushtableinteger(L, "rx_LQ", mBridge.link_stats.receiver_LQ);
  lua_pushtableinteger(L, "rx_LQ_serial", mBridge.link_stats.receiver_LQ_serial);
  lua_pushtableinteger(L, "rx_rssi_inst", mBridge.link_stats.receiver_rssi_instantaneous);
  lua_pushtableinteger(L, "rx_receive_antenna", mBridge.link_stats.receiver_receive_antenna);
  lua_pushtableinteger(L, "rx_transmit_antenna", mBridge.link_stats.receiver_transmit_antenna);
  lua_pushtableinteger(L, "rx_diversity", mBridge.link_stats.receiver_diversity);

  lua_pushtableinteger(L, "rx_rssi_filt", mBridge.link_stats.receiver_rssi_filtered);

  lua_pushtableinteger(L, "LQ_serial_transmitted", mBridge.link_stats.LQ_fresh_serial_packets_transmitted);
  lua_pushtableinteger(L, "byte_rate_transmitted", mBridge.link_stats.bytes_per_sec_transmitted);
  lua_pushtableinteger(L, "LQ_valid_received", mBridge.link_stats.LQ_valid_received);
  lua_pushtableinteger(L, "LQ_serial_received", mBridge.link_stats.LQ_fresh_serial_packets_received);
  lua_pushtableinteger(L, "byte_rate_received", mBridge.link_stats.bytes_per_sec_received);
  lua_pushtableinteger(L, "LQ_frames_received", mBridge.link_stats.LQ_received);

  return 1;
}

static int luaMBridgeGetRssiTables(lua_State * L)
{
  if (mBridge.link_stats.fhss_cnt >= MBRIDGE_RSSI_LIST_LEN) {
    lua_pushnil(L);
  }
  else {
    lua_newtable(L);
    lua_pushtableinteger(L, "fhss_i", mBridge.link_stats.fhss_curr_i);
    lua_pushtableinteger(L, "fhss_cnt", mBridge.link_stats.fhss_cnt);

    lua_pushstring(L, "rssi1");
    lua_newtable(L);
    for (uint8_t i = 0; i < mBridge.link_stats.fhss_cnt; i++) {
      lua_pushinteger(L, i);
      lua_pushinteger(L, mBridge.link_stats.rssi1_list[i]);
      lua_settable(L, -3);
    }
    lua_settable(L, -3);

    lua_pushstring(L, "rssi2");
    lua_newtable(L);
    for (uint8_t i = 0; i < mBridge.link_stats.fhss_cnt; i++) {
      lua_pushinteger(L, i);
      lua_pushinteger(L, mBridge.link_stats.rssi2_list[i]);
      lua_settable(L, -3);
    }
    lua_settable(L, -3);

    lua_pushstring(L, "rx_rssi");
    lua_newtable(L);
    for (uint8_t i = 0; i < mBridge.link_stats.fhss_cnt; i++) {
      lua_pushinteger(L, i);
      lua_pushinteger(L, mBridge.link_stats.receiver_rssi_list[i]);
      lua_settable(L, -3);
    }
    lua_settable(L, -3);

    lua_pushstring(L, "rx_antenna");
    lua_newtable(L);
    for (uint8_t i = 0; i < mBridge.link_stats.fhss_cnt; i++) {
      lua_pushinteger(L, i);
      lua_pushinteger(L, mBridge.link_stats.receiver_antenna_list[i]);
      lua_settable(L, -3);
    }
    lua_settable(L, -3);

  }
  return 1;
}

static int luaMBridgeCmdPop(lua_State * L)
{
  struct MBridge::CmdPacket pkt;

  if (!mBridge.rx_cmd_fifo.pop(pkt)) {
    lua_pushnil(L);
  }
  else {
    lua_newtable(L);
    lua_pushtableinteger(L, "cmd", pkt.cmd);
    lua_pushtableinteger(L, "len", pkt.len);
    lua_pushstring(L, "payload");
    lua_newtable(L);
    for (uint8_t i = 0; i < pkt.len; i++) {
      lua_pushinteger(L, i);
      lua_pushinteger(L, pkt.payload[i]);
      lua_settable(L, -3);
    }
    lua_settable(L, -3);
  }
  return 1;
}

static int luaMBridgeCmdPush(lua_State * L)
{
  if (!mavlinkTelem.externalEnabled()) { // not enabled
    lua_pushnil(L);
  } else
  if (lua_gettop(L) == 0) { // called with no function parameters
    lua_pushboolean(L, false);
  } else {
    struct MBridge::CmdPacket pkt;
    pkt.cmd = luaL_checkunsigned(L, 1);
    luaL_checktype(L, 2, LUA_TTABLE);
    pkt.len = luaL_len(L, 2);
    for (uint8_t i = 0; i < pkt.len; i++) {
      lua_rawgeti(L, 2, i+1);
      pkt.payload[i] = luaL_checkunsigned(L, -1);
    }
    mBridge.tx_cmd_fifo.push(pkt);
    lua_pushboolean(L, true);
  }
  return 1;
}


//------------------------------------------------------------
// mbridge luaL and luaR arrays
//------------------------------------------------------------

const luaL_Reg mbridgeLib[] = {
  { "enabled", luaMBridgeEnabled },
  { "getLinkStats", luaMBridgeGetLinkStats },
  { "getRssiLists", luaMBridgeGetRssiTables },
  { "cmdPop", luaMBridgeCmdPop },
  { "cmdPush", luaMBridgeCmdPush },

  { nullptr, nullptr }  /* sentinel */
};

const luaR_value_entry mbridgeConstants[] = {
/* since we have to emulate anyhow for vanilla OpenTx, no reason to have them
  { "CMD_TX_LINK_STATS", MBRIDGE_CMD_TX_LINK_STATS },
  { "CMD_REQUEST_INFO", MBRIDGE_CMD_REQUEST_INFO },
  { "CMD_DEVICE_ITEM_TX", MBRIDGE_CMD_DEVICE_ITEM_TX },
  { "CMD_DEVICE_ITEM_RX", MBRIDGE_CMD_DEVICE_ITEM_RX },
  { "CMD_PARAM_REQUEST_LIST", MBRIDGE_CMD_PARAM_REQUEST_LIST },
  { "CMD_PARAM_ITEM", MBRIDGE_CMD_PARAM_ITEM },
  { "CMD_PARAM_ITEM2", MBRIDGE_CMD_PARAM_ITEM2 },
  { "CMD_PARAM_ITEM3", MBRIDGE_CMD_PARAM_ITEM3 },
  { "CMD_REQUEST_CMD", MBRIDGE_CMD_REQUEST_CMD },
  { "CMD_INFO", MBRIDGE_CMD_INFO },
  { "CMD_PARAM_SET", MBRIDGE_CMD_PARAM_SET },
  { "CMD_PARAM_STORE", MBRIDGE_CMD_PARAM_STORE },
  { "CMD_BIND", MBRIDGE_CMD_BIND_START },

  { "PARAM_TYPE_UINT8", MBRIDGE_PARAM_TYPE_UINT8 },
  { "PARAM_TYPE_INT8", MBRIDGE_PARAM_TYPE_INT8 },
  { "PARAM_TYPE_UINT16", MBRIDGE_PARAM_TYPE_UINT16 },
  { "PARAM_TYPE_INT16", MBRIDGE_PARAM_TYPE_INT16 },
  { "PARAM_TYPE_LIST", MBRIDGE_PARAM_TYPE_LIST },
  { "PARAM_TYPE_STR6", MBRIDGE_PARAM_TYPE_STR6 }, */

  { nullptr, 0 }  /* sentinel */
};

