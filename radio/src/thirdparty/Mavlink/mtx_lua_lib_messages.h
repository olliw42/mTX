//*******************************************************
// mTX: MAVLink for OpenTx Project
// Copyright (c) OlliW, OlliW42, www.olliw.eu
// Licence GPL2 or (at your option) GPL3
// https://www.gnu.org/licenses/gpl-2.0.en.html
// https://www.gnu.org/licenses/gpl-3.0.en.html
//*******************************************************
    
//------------------------------------------------------------
// mavlink messages
//------------------------------------------------------------
// all message from mtx.xml
// auto generated


//------------------------------------------------------------
// push
//------------------------------------------------------------

#define lua_pushtableinteger_raw(L, k, v)  (lua_pushstring(L,(k)), lua_pushinteger(L,(v)), lua_rawset(L,-3))
#define lua_pushtablenumber_raw(L, k, v)   (lua_pushstring(L,(k)), lua_pushnumber(L,(v)), lua_rawset(L,-3))
#define lua_pushtablestring_raw(L, k, v)   (lua_pushstring(L,(k)), lua_pushstring(L,(v)), lua_rawset(L,-3))
#define lua_pushtableinumber_raw(L, i, v)  (lua_pushnumber(L,(i)), lua_pushnumber(L,(v)), lua_rawset(L,-3))
#define lua_pushtableinumber(L, i, v)      (lua_pushnumber(L,(i)), lua_pushnumber(L,(v)), lua_settable(L,-3))


static void luaMavlinkPushMavMsg(lua_State *L, MavlinkTelem::MavMsg* mavmsg)
{
  lua_newtable(L);
  lua_pushtableinteger(L, "sysid", mavmsg->sysid);
  lua_pushtableinteger(L, "compid", mavmsg->compid);
  lua_pushtableinteger(L, "msgid", mavmsg->msgid);
  lua_pushtableinteger(L, "target_sysid", mavmsg->target_sysid);
  lua_pushtableinteger(L, "target_compid", mavmsg->target_compid);
  lua_pushtableboolean(L, "updated", mavmsg->updated);

  switch (mavmsg->msgid) {
  case FASTMAVLINK_MSG_ID_HEARTBEAT: { // #0
    fmav_heartbeat_t* payload = (fmav_heartbeat_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "type", payload->type);
    lua_pushtablenumber(L, "autopilot", payload->autopilot);
    lua_pushtablenumber(L, "base_mode", payload->base_mode);
    lua_pushtablenumber(L, "custom_mode", payload->custom_mode);
    lua_pushtablenumber(L, "system_status", payload->system_status);
    lua_pushtablenumber(L, "mavlink_version", payload->mavlink_version);
    return;
    }
  case FASTMAVLINK_MSG_ID_SYS_STATUS: { // #1
    fmav_sys_status_t* payload = (fmav_sys_status_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "onboard_control_sensors_present", payload->onboard_control_sensors_present);
    lua_pushtablenumber(L, "onboard_control_sensors_enabled", payload->onboard_control_sensors_enabled);
    lua_pushtablenumber(L, "onboard_control_sensors_health", payload->onboard_control_sensors_health);
    lua_pushtablenumber(L, "load", payload->load);
    lua_pushtablenumber(L, "voltage_battery", payload->voltage_battery);
    lua_pushtablenumber(L, "current_battery", payload->current_battery);
    lua_pushtablenumber(L, "battery_remaining", payload->battery_remaining);
    lua_pushtablenumber(L, "drop_rate_comm", payload->drop_rate_comm);
    lua_pushtablenumber(L, "errors_comm", payload->errors_comm);
    lua_pushtablenumber(L, "errors_count1", payload->errors_count1);
    lua_pushtablenumber(L, "errors_count2", payload->errors_count2);
    lua_pushtablenumber(L, "errors_count3", payload->errors_count3);
    lua_pushtablenumber(L, "errors_count4", payload->errors_count4);
    lua_pushtablenumber(L, "onboard_con_sen_pre_extended", payload->onboard_control_sensors_present_extended);
    lua_pushtablenumber(L, "onboard_con_sen_ena_extended", payload->onboard_control_sensors_enabled_extended);
    lua_pushtablenumber(L, "onboard_con_sen_hea_extended", payload->onboard_control_sensors_health_extended);
    return;
    }
  case FASTMAVLINK_MSG_ID_SYSTEM_TIME: { // #2
    fmav_system_time_t* payload = (fmav_system_time_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "time_unix_usec", payload->time_unix_usec);
    lua_pushtablenumber(L, "time_boot_ms", payload->time_boot_ms);
    return;
    }
  case FASTMAVLINK_MSG_ID_PING: { // #4
    fmav_ping_t* payload = (fmav_ping_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "time_usec", payload->time_usec);
    lua_pushtablenumber(L, "seq", payload->seq);
    return;
    }
  case FASTMAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL: { // #5
    fmav_change_operator_control_t* payload = (fmav_change_operator_control_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "control_request", payload->control_request);
    lua_pushtablenumber(L, "version", payload->version);
    lua_pushstring(L, "passkey"); // array passkey[25]
    lua_newtable(L);
    for (int i = 0; i < 25; i++) { 
      lua_pushtableinumber(L, i+1, payload->passkey[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    return;
    }
  case FASTMAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL_ACK: { // #6
    fmav_change_operator_control_ack_t* payload = (fmav_change_operator_control_ack_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "gcs_system_id", payload->gcs_system_id);
    lua_pushtablenumber(L, "control_request", payload->control_request);
    lua_pushtablenumber(L, "ack", payload->ack);
    return;
    }
  case FASTMAVLINK_MSG_ID_AUTH_KEY: { // #7
    fmav_auth_key_t* payload = (fmav_auth_key_t*)(mavmsg->payload_ptr);
    lua_pushstring(L, "key"); // array key[32]
    lua_newtable(L);
    for (int i = 0; i < 32; i++) { 
      lua_pushtableinumber(L, i+1, payload->key[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    return;
    }
  case FASTMAVLINK_MSG_ID_LINK_NODE_STATUS: { // #8
    fmav_link_node_status_t* payload = (fmav_link_node_status_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "timestamp", payload->timestamp);
    lua_pushtablenumber(L, "tx_buf", payload->tx_buf);
    lua_pushtablenumber(L, "rx_buf", payload->rx_buf);
    lua_pushtablenumber(L, "tx_rate", payload->tx_rate);
    lua_pushtablenumber(L, "rx_rate", payload->rx_rate);
    lua_pushtablenumber(L, "rx_parse_err", payload->rx_parse_err);
    lua_pushtablenumber(L, "tx_overflows", payload->tx_overflows);
    lua_pushtablenumber(L, "rx_overflows", payload->rx_overflows);
    lua_pushtablenumber(L, "messages_sent", payload->messages_sent);
    lua_pushtablenumber(L, "messages_received", payload->messages_received);
    lua_pushtablenumber(L, "messages_lost", payload->messages_lost);
    return;
    }
  case FASTMAVLINK_MSG_ID_SET_MODE: { // #11
    fmav_set_mode_t* payload = (fmav_set_mode_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "base_mode", payload->base_mode);
    lua_pushtablenumber(L, "custom_mode", payload->custom_mode);
    return;
    }
  case FASTMAVLINK_MSG_ID_PARAM_REQUEST_READ: { // #20
    fmav_param_request_read_t* payload = (fmav_param_request_read_t*)(mavmsg->payload_ptr);
    lua_pushstring(L, "param_id"); // array param_id[16]
    lua_newtable(L);
    for (int i = 0; i < 16; i++) { 
      lua_pushtableinumber(L, i+1, payload->param_id[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushtablenumber(L, "param_index", payload->param_index);
    return;
    }
  case FASTMAVLINK_MSG_ID_PARAM_REQUEST_LIST: { // #21
    return;
    }
  case FASTMAVLINK_MSG_ID_PARAM_VALUE: { // #22
    fmav_param_value_t* payload = (fmav_param_value_t*)(mavmsg->payload_ptr);
    lua_pushstring(L, "param_id"); // array param_id[16]
    lua_newtable(L);
    for (int i = 0; i < 16; i++) { 
      lua_pushtableinumber(L, i+1, payload->param_id[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushtablenumber(L, "param_value", payload->param_value);
    lua_pushtablenumber(L, "param_type", payload->param_type);
    lua_pushtablenumber(L, "param_count", payload->param_count);
    lua_pushtablenumber(L, "param_index", payload->param_index);
    return;
    }
  case FASTMAVLINK_MSG_ID_PARAM_SET: { // #23
    fmav_param_set_t* payload = (fmav_param_set_t*)(mavmsg->payload_ptr);
    lua_pushstring(L, "param_id"); // array param_id[16]
    lua_newtable(L);
    for (int i = 0; i < 16; i++) { 
      lua_pushtableinumber(L, i+1, payload->param_id[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushtablenumber(L, "param_value", payload->param_value);
    lua_pushtablenumber(L, "param_type", payload->param_type);
    return;
    }
  case FASTMAVLINK_MSG_ID_GPS_RAW_INT: { // #24
    fmav_gps_raw_int_t* payload = (fmav_gps_raw_int_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "time_usec", payload->time_usec);
    lua_pushtablenumber(L, "fix_type", payload->fix_type);
    lua_pushtablenumber(L, "lat", payload->lat);
    lua_pushtablenumber(L, "lon", payload->lon);
    lua_pushtablenumber(L, "alt", payload->alt);
    lua_pushtablenumber(L, "eph", payload->eph);
    lua_pushtablenumber(L, "epv", payload->epv);
    lua_pushtablenumber(L, "vel", payload->vel);
    lua_pushtablenumber(L, "cog", payload->cog);
    lua_pushtablenumber(L, "satellites_visible", payload->satellites_visible);
    lua_pushtablenumber(L, "alt_ellipsoid", payload->alt_ellipsoid);
    lua_pushtablenumber(L, "h_acc", payload->h_acc);
    lua_pushtablenumber(L, "v_acc", payload->v_acc);
    lua_pushtablenumber(L, "vel_acc", payload->vel_acc);
    lua_pushtablenumber(L, "hdg_acc", payload->hdg_acc);
    lua_pushtablenumber(L, "yaw", payload->yaw);
    return;
    }
  case FASTMAVLINK_MSG_ID_GPS_STATUS: { // #25
    fmav_gps_status_t* payload = (fmav_gps_status_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "satellites_visible", payload->satellites_visible);
    lua_pushstring(L, "satellite_prn"); // array satellite_prn[20]
    lua_newtable(L);
    for (int i = 0; i < 20; i++) { 
      lua_pushtableinumber(L, i+1, payload->satellite_prn[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushstring(L, "satellite_used"); // array satellite_used[20]
    lua_newtable(L);
    for (int i = 0; i < 20; i++) { 
      lua_pushtableinumber(L, i+1, payload->satellite_used[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushstring(L, "satellite_elevation"); // array satellite_elevation[20]
    lua_newtable(L);
    for (int i = 0; i < 20; i++) { 
      lua_pushtableinumber(L, i+1, payload->satellite_elevation[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushstring(L, "satellite_azimuth"); // array satellite_azimuth[20]
    lua_newtable(L);
    for (int i = 0; i < 20; i++) { 
      lua_pushtableinumber(L, i+1, payload->satellite_azimuth[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushstring(L, "satellite_snr"); // array satellite_snr[20]
    lua_newtable(L);
    for (int i = 0; i < 20; i++) { 
      lua_pushtableinumber(L, i+1, payload->satellite_snr[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    return;
    }
  case FASTMAVLINK_MSG_ID_SCALED_IMU: { // #26
    fmav_scaled_imu_t* payload = (fmav_scaled_imu_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "time_boot_ms", payload->time_boot_ms);
    lua_pushtablenumber(L, "xacc", payload->xacc);
    lua_pushtablenumber(L, "yacc", payload->yacc);
    lua_pushtablenumber(L, "zacc", payload->zacc);
    lua_pushtablenumber(L, "xgyro", payload->xgyro);
    lua_pushtablenumber(L, "ygyro", payload->ygyro);
    lua_pushtablenumber(L, "zgyro", payload->zgyro);
    lua_pushtablenumber(L, "xmag", payload->xmag);
    lua_pushtablenumber(L, "ymag", payload->ymag);
    lua_pushtablenumber(L, "zmag", payload->zmag);
    lua_pushtablenumber(L, "temperature", payload->temperature);
    return;
    }
  case FASTMAVLINK_MSG_ID_RAW_IMU: { // #27
    fmav_raw_imu_t* payload = (fmav_raw_imu_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "time_usec", payload->time_usec);
    lua_pushtablenumber(L, "xacc", payload->xacc);
    lua_pushtablenumber(L, "yacc", payload->yacc);
    lua_pushtablenumber(L, "zacc", payload->zacc);
    lua_pushtablenumber(L, "xgyro", payload->xgyro);
    lua_pushtablenumber(L, "ygyro", payload->ygyro);
    lua_pushtablenumber(L, "zgyro", payload->zgyro);
    lua_pushtablenumber(L, "xmag", payload->xmag);
    lua_pushtablenumber(L, "ymag", payload->ymag);
    lua_pushtablenumber(L, "zmag", payload->zmag);
    lua_pushtablenumber(L, "id", payload->id);
    lua_pushtablenumber(L, "temperature", payload->temperature);
    return;
    }
  case FASTMAVLINK_MSG_ID_RAW_PRESSURE: { // #28
    fmav_raw_pressure_t* payload = (fmav_raw_pressure_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "time_usec", payload->time_usec);
    lua_pushtablenumber(L, "press_abs", payload->press_abs);
    lua_pushtablenumber(L, "press_diff1", payload->press_diff1);
    lua_pushtablenumber(L, "press_diff2", payload->press_diff2);
    lua_pushtablenumber(L, "temperature", payload->temperature);
    return;
    }
  case FASTMAVLINK_MSG_ID_SCALED_PRESSURE: { // #29
    fmav_scaled_pressure_t* payload = (fmav_scaled_pressure_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "time_boot_ms", payload->time_boot_ms);
    lua_pushtablenumber(L, "press_abs", payload->press_abs);
    lua_pushtablenumber(L, "press_diff", payload->press_diff);
    lua_pushtablenumber(L, "temperature", payload->temperature);
    lua_pushtablenumber(L, "temperature_press_diff", payload->temperature_press_diff);
    return;
    }
  case FASTMAVLINK_MSG_ID_ATTITUDE: { // #30
    fmav_attitude_t* payload = (fmav_attitude_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "time_boot_ms", payload->time_boot_ms);
    lua_pushtablenumber(L, "roll", payload->roll);
    lua_pushtablenumber(L, "pitch", payload->pitch);
    lua_pushtablenumber(L, "yaw", payload->yaw);
    lua_pushtablenumber(L, "rollspeed", payload->rollspeed);
    lua_pushtablenumber(L, "pitchspeed", payload->pitchspeed);
    lua_pushtablenumber(L, "yawspeed", payload->yawspeed);
    return;
    }
  case FASTMAVLINK_MSG_ID_ATTITUDE_QUATERNION: { // #31
    fmav_attitude_quaternion_t* payload = (fmav_attitude_quaternion_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "time_boot_ms", payload->time_boot_ms);
    lua_pushtablenumber(L, "q1", payload->q1);
    lua_pushtablenumber(L, "q2", payload->q2);
    lua_pushtablenumber(L, "q3", payload->q3);
    lua_pushtablenumber(L, "q4", payload->q4);
    lua_pushtablenumber(L, "rollspeed", payload->rollspeed);
    lua_pushtablenumber(L, "pitchspeed", payload->pitchspeed);
    lua_pushtablenumber(L, "yawspeed", payload->yawspeed);
    lua_pushstring(L, "repr_offset_q"); // array repr_offset_q[4]
    lua_newtable(L);
    for (int i = 0; i < 4; i++) { 
      lua_pushtableinumber(L, i+1, payload->repr_offset_q[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    return;
    }
  case FASTMAVLINK_MSG_ID_LOCAL_POSITION_NED: { // #32
    fmav_local_position_ned_t* payload = (fmav_local_position_ned_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "time_boot_ms", payload->time_boot_ms);
    lua_pushtablenumber(L, "x", payload->x);
    lua_pushtablenumber(L, "y", payload->y);
    lua_pushtablenumber(L, "z", payload->z);
    lua_pushtablenumber(L, "vx", payload->vx);
    lua_pushtablenumber(L, "vy", payload->vy);
    lua_pushtablenumber(L, "vz", payload->vz);
    return;
    }
  case FASTMAVLINK_MSG_ID_GLOBAL_POSITION_INT: { // #33
    fmav_global_position_int_t* payload = (fmav_global_position_int_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "time_boot_ms", payload->time_boot_ms);
    lua_pushtablenumber(L, "lat", payload->lat);
    lua_pushtablenumber(L, "lon", payload->lon);
    lua_pushtablenumber(L, "alt", payload->alt);
    lua_pushtablenumber(L, "relative_alt", payload->relative_alt);
    lua_pushtablenumber(L, "vx", payload->vx);
    lua_pushtablenumber(L, "vy", payload->vy);
    lua_pushtablenumber(L, "vz", payload->vz);
    lua_pushtablenumber(L, "hdg", payload->hdg);
    return;
    }
  case FASTMAVLINK_MSG_ID_RC_CHANNELS_SCALED: { // #34
    fmav_rc_channels_scaled_t* payload = (fmav_rc_channels_scaled_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "time_boot_ms", payload->time_boot_ms);
    lua_pushtablenumber(L, "port", payload->port);
    lua_pushtablenumber(L, "chan1_scaled", payload->chan1_scaled);
    lua_pushtablenumber(L, "chan2_scaled", payload->chan2_scaled);
    lua_pushtablenumber(L, "chan3_scaled", payload->chan3_scaled);
    lua_pushtablenumber(L, "chan4_scaled", payload->chan4_scaled);
    lua_pushtablenumber(L, "chan5_scaled", payload->chan5_scaled);
    lua_pushtablenumber(L, "chan6_scaled", payload->chan6_scaled);
    lua_pushtablenumber(L, "chan7_scaled", payload->chan7_scaled);
    lua_pushtablenumber(L, "chan8_scaled", payload->chan8_scaled);
    lua_pushtablenumber(L, "rssi", payload->rssi);
    return;
    }
  case FASTMAVLINK_MSG_ID_RC_CHANNELS_RAW: { // #35
    fmav_rc_channels_raw_t* payload = (fmav_rc_channels_raw_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "time_boot_ms", payload->time_boot_ms);
    lua_pushtablenumber(L, "port", payload->port);
    lua_pushtablenumber(L, "chan1_raw", payload->chan1_raw);
    lua_pushtablenumber(L, "chan2_raw", payload->chan2_raw);
    lua_pushtablenumber(L, "chan3_raw", payload->chan3_raw);
    lua_pushtablenumber(L, "chan4_raw", payload->chan4_raw);
    lua_pushtablenumber(L, "chan5_raw", payload->chan5_raw);
    lua_pushtablenumber(L, "chan6_raw", payload->chan6_raw);
    lua_pushtablenumber(L, "chan7_raw", payload->chan7_raw);
    lua_pushtablenumber(L, "chan8_raw", payload->chan8_raw);
    lua_pushtablenumber(L, "rssi", payload->rssi);
    return;
    }
  case FASTMAVLINK_MSG_ID_SERVO_OUTPUT_RAW: { // #36
    fmav_servo_output_raw_t* payload = (fmav_servo_output_raw_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "time_usec", payload->time_usec);
    lua_pushtablenumber(L, "port", payload->port);
    lua_pushtablenumber(L, "servo1_raw", payload->servo1_raw);
    lua_pushtablenumber(L, "servo2_raw", payload->servo2_raw);
    lua_pushtablenumber(L, "servo3_raw", payload->servo3_raw);
    lua_pushtablenumber(L, "servo4_raw", payload->servo4_raw);
    lua_pushtablenumber(L, "servo5_raw", payload->servo5_raw);
    lua_pushtablenumber(L, "servo6_raw", payload->servo6_raw);
    lua_pushtablenumber(L, "servo7_raw", payload->servo7_raw);
    lua_pushtablenumber(L, "servo8_raw", payload->servo8_raw);
    lua_pushtablenumber(L, "servo9_raw", payload->servo9_raw);
    lua_pushtablenumber(L, "servo10_raw", payload->servo10_raw);
    lua_pushtablenumber(L, "servo11_raw", payload->servo11_raw);
    lua_pushtablenumber(L, "servo12_raw", payload->servo12_raw);
    lua_pushtablenumber(L, "servo13_raw", payload->servo13_raw);
    lua_pushtablenumber(L, "servo14_raw", payload->servo14_raw);
    lua_pushtablenumber(L, "servo15_raw", payload->servo15_raw);
    lua_pushtablenumber(L, "servo16_raw", payload->servo16_raw);
    return;
    }
  case FASTMAVLINK_MSG_ID_MISSION_REQUEST_PARTIAL_LIST: { // #37
    fmav_mission_request_partial_list_t* payload = (fmav_mission_request_partial_list_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "start_index", payload->start_index);
    lua_pushtablenumber(L, "end_index", payload->end_index);
    lua_pushtablenumber(L, "mission_type", payload->mission_type);
    return;
    }
  case FASTMAVLINK_MSG_ID_MISSION_WRITE_PARTIAL_LIST: { // #38
    fmav_mission_write_partial_list_t* payload = (fmav_mission_write_partial_list_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "start_index", payload->start_index);
    lua_pushtablenumber(L, "end_index", payload->end_index);
    lua_pushtablenumber(L, "mission_type", payload->mission_type);
    return;
    }
  case FASTMAVLINK_MSG_ID_MISSION_ITEM: { // #39
    fmav_mission_item_t* payload = (fmav_mission_item_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "seq", payload->seq);
    lua_pushtablenumber(L, "frame", payload->frame);
    lua_pushtablenumber(L, "command", payload->command);
    lua_pushtablenumber(L, "current", payload->current);
    lua_pushtablenumber(L, "autocontinue", payload->autocontinue);
    lua_pushtablenumber(L, "param1", payload->param1);
    lua_pushtablenumber(L, "param2", payload->param2);
    lua_pushtablenumber(L, "param3", payload->param3);
    lua_pushtablenumber(L, "param4", payload->param4);
    lua_pushtablenumber(L, "x", payload->x);
    lua_pushtablenumber(L, "y", payload->y);
    lua_pushtablenumber(L, "z", payload->z);
    lua_pushtablenumber(L, "mission_type", payload->mission_type);
    return;
    }
  case FASTMAVLINK_MSG_ID_MISSION_REQUEST: { // #40
    fmav_mission_request_t* payload = (fmav_mission_request_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "seq", payload->seq);
    lua_pushtablenumber(L, "mission_type", payload->mission_type);
    return;
    }
  case FASTMAVLINK_MSG_ID_MISSION_SET_CURRENT: { // #41
    fmav_mission_set_current_t* payload = (fmav_mission_set_current_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "seq", payload->seq);
    return;
    }
  case FASTMAVLINK_MSG_ID_MISSION_CURRENT: { // #42
    fmav_mission_current_t* payload = (fmav_mission_current_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "seq", payload->seq);
    lua_pushtablenumber(L, "total", payload->total);
    lua_pushtablenumber(L, "mission_state", payload->mission_state);
    lua_pushtablenumber(L, "mission_mode", payload->mission_mode);
    lua_pushtablenumber(L, "mission_id", payload->mission_id);
    lua_pushtablenumber(L, "fence_id", payload->fence_id);
    lua_pushtablenumber(L, "rally_points_id", payload->rally_points_id);
    return;
    }
  case FASTMAVLINK_MSG_ID_MISSION_REQUEST_LIST: { // #43
    fmav_mission_request_list_t* payload = (fmav_mission_request_list_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "mission_type", payload->mission_type);
    return;
    }
  case FASTMAVLINK_MSG_ID_MISSION_COUNT: { // #44
    fmav_mission_count_t* payload = (fmav_mission_count_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "count", payload->count);
    lua_pushtablenumber(L, "mission_type", payload->mission_type);
    lua_pushtablenumber(L, "opaque_id", payload->opaque_id);
    return;
    }
  case FASTMAVLINK_MSG_ID_MISSION_CLEAR_ALL: { // #45
    fmav_mission_clear_all_t* payload = (fmav_mission_clear_all_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "mission_type", payload->mission_type);
    return;
    }
  case FASTMAVLINK_MSG_ID_MISSION_ITEM_REACHED: { // #46
    fmav_mission_item_reached_t* payload = (fmav_mission_item_reached_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "seq", payload->seq);
    return;
    }
  case FASTMAVLINK_MSG_ID_MISSION_ACK: { // #47
    fmav_mission_ack_t* payload = (fmav_mission_ack_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "type", payload->type);
    lua_pushtablenumber(L, "mission_type", payload->mission_type);
    lua_pushtablenumber(L, "opaque_id", payload->opaque_id);
    return;
    }
  case FASTMAVLINK_MSG_ID_SET_GPS_GLOBAL_ORIGIN: { // #48
    fmav_set_gps_global_origin_t* payload = (fmav_set_gps_global_origin_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "latitude", payload->latitude);
    lua_pushtablenumber(L, "longitude", payload->longitude);
    lua_pushtablenumber(L, "altitude", payload->altitude);
    lua_pushtablenumber(L, "time_usec", payload->time_usec);
    return;
    }
  case FASTMAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN: { // #49
    fmav_gps_global_origin_t* payload = (fmav_gps_global_origin_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "latitude", payload->latitude);
    lua_pushtablenumber(L, "longitude", payload->longitude);
    lua_pushtablenumber(L, "altitude", payload->altitude);
    lua_pushtablenumber(L, "time_usec", payload->time_usec);
    return;
    }
  case FASTMAVLINK_MSG_ID_PARAM_MAP_RC: { // #50
    fmav_param_map_rc_t* payload = (fmav_param_map_rc_t*)(mavmsg->payload_ptr);
    lua_pushstring(L, "param_id"); // array param_id[16]
    lua_newtable(L);
    for (int i = 0; i < 16; i++) { 
      lua_pushtableinumber(L, i+1, payload->param_id[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushtablenumber(L, "param_index", payload->param_index);
    lua_pushtablenumber(L, "parameter_rc_channel_index", payload->parameter_rc_channel_index);
    lua_pushtablenumber(L, "param_value0", payload->param_value0);
    lua_pushtablenumber(L, "scale", payload->scale);
    lua_pushtablenumber(L, "param_value_min", payload->param_value_min);
    lua_pushtablenumber(L, "param_value_max", payload->param_value_max);
    return;
    }
  case FASTMAVLINK_MSG_ID_MISSION_REQUEST_INT: { // #51
    fmav_mission_request_int_t* payload = (fmav_mission_request_int_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "seq", payload->seq);
    lua_pushtablenumber(L, "mission_type", payload->mission_type);
    return;
    }
  case FASTMAVLINK_MSG_ID_SAFETY_SET_ALLOWED_AREA: { // #54
    fmav_safety_set_allowed_area_t* payload = (fmav_safety_set_allowed_area_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "frame", payload->frame);
    lua_pushtablenumber(L, "p1x", payload->p1x);
    lua_pushtablenumber(L, "p1y", payload->p1y);
    lua_pushtablenumber(L, "p1z", payload->p1z);
    lua_pushtablenumber(L, "p2x", payload->p2x);
    lua_pushtablenumber(L, "p2y", payload->p2y);
    lua_pushtablenumber(L, "p2z", payload->p2z);
    return;
    }
  case FASTMAVLINK_MSG_ID_SAFETY_ALLOWED_AREA: { // #55
    fmav_safety_allowed_area_t* payload = (fmav_safety_allowed_area_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "frame", payload->frame);
    lua_pushtablenumber(L, "p1x", payload->p1x);
    lua_pushtablenumber(L, "p1y", payload->p1y);
    lua_pushtablenumber(L, "p1z", payload->p1z);
    lua_pushtablenumber(L, "p2x", payload->p2x);
    lua_pushtablenumber(L, "p2y", payload->p2y);
    lua_pushtablenumber(L, "p2z", payload->p2z);
    return;
    }
  case FASTMAVLINK_MSG_ID_ATTITUDE_QUATERNION_COV: { // #61
    fmav_attitude_quaternion_cov_t* payload = (fmav_attitude_quaternion_cov_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "time_usec", payload->time_usec);
    lua_pushstring(L, "q"); // array q[4]
    lua_newtable(L);
    for (int i = 0; i < 4; i++) { 
      lua_pushtableinumber(L, i+1, payload->q[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushtablenumber(L, "rollspeed", payload->rollspeed);
    lua_pushtablenumber(L, "pitchspeed", payload->pitchspeed);
    lua_pushtablenumber(L, "yawspeed", payload->yawspeed);
    lua_pushstring(L, "covariance"); // array covariance[9]
    lua_newtable(L);
    for (int i = 0; i < 9; i++) { 
      lua_pushtableinumber(L, i+1, payload->covariance[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    return;
    }
  case FASTMAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT: { // #62
    fmav_nav_controller_output_t* payload = (fmav_nav_controller_output_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "nav_roll", payload->nav_roll);
    lua_pushtablenumber(L, "nav_pitch", payload->nav_pitch);
    lua_pushtablenumber(L, "nav_bearing", payload->nav_bearing);
    lua_pushtablenumber(L, "target_bearing", payload->target_bearing);
    lua_pushtablenumber(L, "wp_dist", payload->wp_dist);
    lua_pushtablenumber(L, "alt_error", payload->alt_error);
    lua_pushtablenumber(L, "aspd_error", payload->aspd_error);
    lua_pushtablenumber(L, "xtrack_error", payload->xtrack_error);
    return;
    }
  case FASTMAVLINK_MSG_ID_GLOBAL_POSITION_INT_COV: { // #63
    fmav_global_position_int_cov_t* payload = (fmav_global_position_int_cov_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "time_usec", payload->time_usec);
    lua_pushtablenumber(L, "estimator_type", payload->estimator_type);
    lua_pushtablenumber(L, "lat", payload->lat);
    lua_pushtablenumber(L, "lon", payload->lon);
    lua_pushtablenumber(L, "alt", payload->alt);
    lua_pushtablenumber(L, "relative_alt", payload->relative_alt);
    lua_pushtablenumber(L, "vx", payload->vx);
    lua_pushtablenumber(L, "vy", payload->vy);
    lua_pushtablenumber(L, "vz", payload->vz);
    lua_pushstring(L, "covariance"); // array covariance[36]
    lua_newtable(L);
    for (int i = 0; i < 36; i++) { 
      lua_pushtableinumber(L, i+1, payload->covariance[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    return;
    }
  case FASTMAVLINK_MSG_ID_LOCAL_POSITION_NED_COV: { // #64
    fmav_local_position_ned_cov_t* payload = (fmav_local_position_ned_cov_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "time_usec", payload->time_usec);
    lua_pushtablenumber(L, "estimator_type", payload->estimator_type);
    lua_pushtablenumber(L, "x", payload->x);
    lua_pushtablenumber(L, "y", payload->y);
    lua_pushtablenumber(L, "z", payload->z);
    lua_pushtablenumber(L, "vx", payload->vx);
    lua_pushtablenumber(L, "vy", payload->vy);
    lua_pushtablenumber(L, "vz", payload->vz);
    lua_pushtablenumber(L, "ax", payload->ax);
    lua_pushtablenumber(L, "ay", payload->ay);
    lua_pushtablenumber(L, "az", payload->az);
    lua_pushstring(L, "covariance"); // array covariance[45]
    lua_newtable(L);
    for (int i = 0; i < 45; i++) { 
      lua_pushtableinumber(L, i+1, payload->covariance[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    return;
    }
  case FASTMAVLINK_MSG_ID_RC_CHANNELS: { // #65
    fmav_rc_channels_t* payload = (fmav_rc_channels_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "time_boot_ms", payload->time_boot_ms);
    lua_pushtablenumber(L, "chancount", payload->chancount);
    lua_pushtablenumber(L, "chan1_raw", payload->chan1_raw);
    lua_pushtablenumber(L, "chan2_raw", payload->chan2_raw);
    lua_pushtablenumber(L, "chan3_raw", payload->chan3_raw);
    lua_pushtablenumber(L, "chan4_raw", payload->chan4_raw);
    lua_pushtablenumber(L, "chan5_raw", payload->chan5_raw);
    lua_pushtablenumber(L, "chan6_raw", payload->chan6_raw);
    lua_pushtablenumber(L, "chan7_raw", payload->chan7_raw);
    lua_pushtablenumber(L, "chan8_raw", payload->chan8_raw);
    lua_pushtablenumber(L, "chan9_raw", payload->chan9_raw);
    lua_pushtablenumber(L, "chan10_raw", payload->chan10_raw);
    lua_pushtablenumber(L, "chan11_raw", payload->chan11_raw);
    lua_pushtablenumber(L, "chan12_raw", payload->chan12_raw);
    lua_pushtablenumber(L, "chan13_raw", payload->chan13_raw);
    lua_pushtablenumber(L, "chan14_raw", payload->chan14_raw);
    lua_pushtablenumber(L, "chan15_raw", payload->chan15_raw);
    lua_pushtablenumber(L, "chan16_raw", payload->chan16_raw);
    lua_pushtablenumber(L, "chan17_raw", payload->chan17_raw);
    lua_pushtablenumber(L, "chan18_raw", payload->chan18_raw);
    lua_pushtablenumber(L, "rssi", payload->rssi);
    return;
    }
  case FASTMAVLINK_MSG_ID_REQUEST_DATA_STREAM: { // #66
    fmav_request_data_stream_t* payload = (fmav_request_data_stream_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "req_stream_id", payload->req_stream_id);
    lua_pushtablenumber(L, "req_message_rate", payload->req_message_rate);
    lua_pushtablenumber(L, "start_stop", payload->start_stop);
    return;
    }
  case FASTMAVLINK_MSG_ID_DATA_STREAM: { // #67
    fmav_data_stream_t* payload = (fmav_data_stream_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "stream_id", payload->stream_id);
    lua_pushtablenumber(L, "message_rate", payload->message_rate);
    lua_pushtablenumber(L, "on_off", payload->on_off);
    return;
    }
  case FASTMAVLINK_MSG_ID_MANUAL_CONTROL: { // #69
    fmav_manual_control_t* payload = (fmav_manual_control_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "x", payload->x);
    lua_pushtablenumber(L, "y", payload->y);
    lua_pushtablenumber(L, "z", payload->z);
    lua_pushtablenumber(L, "r", payload->r);
    lua_pushtablenumber(L, "buttons", payload->buttons);
    lua_pushtablenumber(L, "buttons2", payload->buttons2);
    lua_pushtablenumber(L, "enabled_extensions", payload->enabled_extensions);
    lua_pushtablenumber(L, "s", payload->s);
    lua_pushtablenumber(L, "t", payload->t);
    lua_pushtablenumber(L, "aux1", payload->aux1);
    lua_pushtablenumber(L, "aux2", payload->aux2);
    lua_pushtablenumber(L, "aux3", payload->aux3);
    lua_pushtablenumber(L, "aux4", payload->aux4);
    lua_pushtablenumber(L, "aux5", payload->aux5);
    lua_pushtablenumber(L, "aux6", payload->aux6);
    return;
    }
  case FASTMAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE: { // #70
    fmav_rc_channels_override_t* payload = (fmav_rc_channels_override_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "chan1_raw", payload->chan1_raw);
    lua_pushtablenumber(L, "chan2_raw", payload->chan2_raw);
    lua_pushtablenumber(L, "chan3_raw", payload->chan3_raw);
    lua_pushtablenumber(L, "chan4_raw", payload->chan4_raw);
    lua_pushtablenumber(L, "chan5_raw", payload->chan5_raw);
    lua_pushtablenumber(L, "chan6_raw", payload->chan6_raw);
    lua_pushtablenumber(L, "chan7_raw", payload->chan7_raw);
    lua_pushtablenumber(L, "chan8_raw", payload->chan8_raw);
    lua_pushtablenumber(L, "chan9_raw", payload->chan9_raw);
    lua_pushtablenumber(L, "chan10_raw", payload->chan10_raw);
    lua_pushtablenumber(L, "chan11_raw", payload->chan11_raw);
    lua_pushtablenumber(L, "chan12_raw", payload->chan12_raw);
    lua_pushtablenumber(L, "chan13_raw", payload->chan13_raw);
    lua_pushtablenumber(L, "chan14_raw", payload->chan14_raw);
    lua_pushtablenumber(L, "chan15_raw", payload->chan15_raw);
    lua_pushtablenumber(L, "chan16_raw", payload->chan16_raw);
    lua_pushtablenumber(L, "chan17_raw", payload->chan17_raw);
    lua_pushtablenumber(L, "chan18_raw", payload->chan18_raw);
    return;
    }
  case FASTMAVLINK_MSG_ID_MISSION_ITEM_INT: { // #73
    fmav_mission_item_int_t* payload = (fmav_mission_item_int_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "seq", payload->seq);
    lua_pushtablenumber(L, "frame", payload->frame);
    lua_pushtablenumber(L, "command", payload->command);
    lua_pushtablenumber(L, "current", payload->current);
    lua_pushtablenumber(L, "autocontinue", payload->autocontinue);
    lua_pushtablenumber(L, "param1", payload->param1);
    lua_pushtablenumber(L, "param2", payload->param2);
    lua_pushtablenumber(L, "param3", payload->param3);
    lua_pushtablenumber(L, "param4", payload->param4);
    lua_pushtablenumber(L, "x", payload->x);
    lua_pushtablenumber(L, "y", payload->y);
    lua_pushtablenumber(L, "z", payload->z);
    lua_pushtablenumber(L, "mission_type", payload->mission_type);
    return;
    }
  case FASTMAVLINK_MSG_ID_VFR_HUD: { // #74
    fmav_vfr_hud_t* payload = (fmav_vfr_hud_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "airspeed", payload->airspeed);
    lua_pushtablenumber(L, "groundspeed", payload->groundspeed);
    lua_pushtablenumber(L, "heading", payload->heading);
    lua_pushtablenumber(L, "throttle", payload->throttle);
    lua_pushtablenumber(L, "alt", payload->alt);
    lua_pushtablenumber(L, "climb", payload->climb);
    return;
    }
  case FASTMAVLINK_MSG_ID_COMMAND_INT: { // #75
    fmav_command_int_t* payload = (fmav_command_int_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "frame", payload->frame);
    lua_pushtablenumber(L, "command", payload->command);
    lua_pushtablenumber(L, "current", payload->current);
    lua_pushtablenumber(L, "autocontinue", payload->autocontinue);
    lua_pushtablenumber(L, "param1", payload->param1);
    lua_pushtablenumber(L, "param2", payload->param2);
    lua_pushtablenumber(L, "param3", payload->param3);
    lua_pushtablenumber(L, "param4", payload->param4);
    lua_pushtablenumber(L, "x", payload->x);
    lua_pushtablenumber(L, "y", payload->y);
    lua_pushtablenumber(L, "z", payload->z);
    return;
    }
  case FASTMAVLINK_MSG_ID_COMMAND_LONG: { // #76
    fmav_command_long_t* payload = (fmav_command_long_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "command", payload->command);
    lua_pushtablenumber(L, "confirmation", payload->confirmation);
    lua_pushtablenumber(L, "param1", payload->param1);
    lua_pushtablenumber(L, "param2", payload->param2);
    lua_pushtablenumber(L, "param3", payload->param3);
    lua_pushtablenumber(L, "param4", payload->param4);
    lua_pushtablenumber(L, "param5", payload->param5);
    lua_pushtablenumber(L, "param6", payload->param6);
    lua_pushtablenumber(L, "param7", payload->param7);
    return;
    }
  case FASTMAVLINK_MSG_ID_COMMAND_ACK: { // #77
    fmav_command_ack_t* payload = (fmav_command_ack_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "command", payload->command);
    lua_pushtablenumber(L, "result", payload->result);
    lua_pushtablenumber(L, "progress", payload->progress);
    lua_pushtablenumber(L, "result_param2", payload->result_param2);
    return;
    }
  case FASTMAVLINK_MSG_ID_COMMAND_CANCEL: { // #80
    fmav_command_cancel_t* payload = (fmav_command_cancel_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "command", payload->command);
    return;
    }
  case FASTMAVLINK_MSG_ID_MANUAL_SETPOINT: { // #81
    fmav_manual_setpoint_t* payload = (fmav_manual_setpoint_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "time_boot_ms", payload->time_boot_ms);
    lua_pushtablenumber(L, "roll", payload->roll);
    lua_pushtablenumber(L, "pitch", payload->pitch);
    lua_pushtablenumber(L, "yaw", payload->yaw);
    lua_pushtablenumber(L, "thrust", payload->thrust);
    lua_pushtablenumber(L, "mode_switch", payload->mode_switch);
    lua_pushtablenumber(L, "manual_override_switch", payload->manual_override_switch);
    return;
    }
  case FASTMAVLINK_MSG_ID_SET_ATTITUDE_TARGET: { // #82
    fmav_set_attitude_target_t* payload = (fmav_set_attitude_target_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "time_boot_ms", payload->time_boot_ms);
    lua_pushtablenumber(L, "type_mask", payload->type_mask);
    lua_pushstring(L, "q"); // array q[4]
    lua_newtable(L);
    for (int i = 0; i < 4; i++) { 
      lua_pushtableinumber(L, i+1, payload->q[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushtablenumber(L, "body_roll_rate", payload->body_roll_rate);
    lua_pushtablenumber(L, "body_pitch_rate", payload->body_pitch_rate);
    lua_pushtablenumber(L, "body_yaw_rate", payload->body_yaw_rate);
    lua_pushtablenumber(L, "thrust", payload->thrust);
    lua_pushstring(L, "thrust_body"); // array thrust_body[3]
    lua_newtable(L);
    for (int i = 0; i < 3; i++) { 
      lua_pushtableinumber(L, i+1, payload->thrust_body[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    return;
    }
  case FASTMAVLINK_MSG_ID_ATTITUDE_TARGET: { // #83
    fmav_attitude_target_t* payload = (fmav_attitude_target_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "time_boot_ms", payload->time_boot_ms);
    lua_pushtablenumber(L, "type_mask", payload->type_mask);
    lua_pushstring(L, "q"); // array q[4]
    lua_newtable(L);
    for (int i = 0; i < 4; i++) { 
      lua_pushtableinumber(L, i+1, payload->q[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushtablenumber(L, "body_roll_rate", payload->body_roll_rate);
    lua_pushtablenumber(L, "body_pitch_rate", payload->body_pitch_rate);
    lua_pushtablenumber(L, "body_yaw_rate", payload->body_yaw_rate);
    lua_pushtablenumber(L, "thrust", payload->thrust);
    return;
    }
  case FASTMAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED: { // #84
    fmav_set_position_target_local_ned_t* payload = (fmav_set_position_target_local_ned_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "time_boot_ms", payload->time_boot_ms);
    lua_pushtablenumber(L, "coordinate_frame", payload->coordinate_frame);
    lua_pushtablenumber(L, "type_mask", payload->type_mask);
    lua_pushtablenumber(L, "x", payload->x);
    lua_pushtablenumber(L, "y", payload->y);
    lua_pushtablenumber(L, "z", payload->z);
    lua_pushtablenumber(L, "vx", payload->vx);
    lua_pushtablenumber(L, "vy", payload->vy);
    lua_pushtablenumber(L, "vz", payload->vz);
    lua_pushtablenumber(L, "afx", payload->afx);
    lua_pushtablenumber(L, "afy", payload->afy);
    lua_pushtablenumber(L, "afz", payload->afz);
    lua_pushtablenumber(L, "yaw", payload->yaw);
    lua_pushtablenumber(L, "yaw_rate", payload->yaw_rate);
    return;
    }
  case FASTMAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED: { // #85
    fmav_position_target_local_ned_t* payload = (fmav_position_target_local_ned_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "time_boot_ms", payload->time_boot_ms);
    lua_pushtablenumber(L, "coordinate_frame", payload->coordinate_frame);
    lua_pushtablenumber(L, "type_mask", payload->type_mask);
    lua_pushtablenumber(L, "x", payload->x);
    lua_pushtablenumber(L, "y", payload->y);
    lua_pushtablenumber(L, "z", payload->z);
    lua_pushtablenumber(L, "vx", payload->vx);
    lua_pushtablenumber(L, "vy", payload->vy);
    lua_pushtablenumber(L, "vz", payload->vz);
    lua_pushtablenumber(L, "afx", payload->afx);
    lua_pushtablenumber(L, "afy", payload->afy);
    lua_pushtablenumber(L, "afz", payload->afz);
    lua_pushtablenumber(L, "yaw", payload->yaw);
    lua_pushtablenumber(L, "yaw_rate", payload->yaw_rate);
    return;
    }
  case FASTMAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT: { // #86
    fmav_set_position_target_global_int_t* payload = (fmav_set_position_target_global_int_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "time_boot_ms", payload->time_boot_ms);
    lua_pushtablenumber(L, "coordinate_frame", payload->coordinate_frame);
    lua_pushtablenumber(L, "type_mask", payload->type_mask);
    lua_pushtablenumber(L, "lat_int", payload->lat_int);
    lua_pushtablenumber(L, "lon_int", payload->lon_int);
    lua_pushtablenumber(L, "alt", payload->alt);
    lua_pushtablenumber(L, "vx", payload->vx);
    lua_pushtablenumber(L, "vy", payload->vy);
    lua_pushtablenumber(L, "vz", payload->vz);
    lua_pushtablenumber(L, "afx", payload->afx);
    lua_pushtablenumber(L, "afy", payload->afy);
    lua_pushtablenumber(L, "afz", payload->afz);
    lua_pushtablenumber(L, "yaw", payload->yaw);
    lua_pushtablenumber(L, "yaw_rate", payload->yaw_rate);
    return;
    }
  case FASTMAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT: { // #87
    fmav_position_target_global_int_t* payload = (fmav_position_target_global_int_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "time_boot_ms", payload->time_boot_ms);
    lua_pushtablenumber(L, "coordinate_frame", payload->coordinate_frame);
    lua_pushtablenumber(L, "type_mask", payload->type_mask);
    lua_pushtablenumber(L, "lat_int", payload->lat_int);
    lua_pushtablenumber(L, "lon_int", payload->lon_int);
    lua_pushtablenumber(L, "alt", payload->alt);
    lua_pushtablenumber(L, "vx", payload->vx);
    lua_pushtablenumber(L, "vy", payload->vy);
    lua_pushtablenumber(L, "vz", payload->vz);
    lua_pushtablenumber(L, "afx", payload->afx);
    lua_pushtablenumber(L, "afy", payload->afy);
    lua_pushtablenumber(L, "afz", payload->afz);
    lua_pushtablenumber(L, "yaw", payload->yaw);
    lua_pushtablenumber(L, "yaw_rate", payload->yaw_rate);
    return;
    }
  case FASTMAVLINK_MSG_ID_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET: { // #89
    fmav_local_position_ned_system_global_offset_t* payload = (fmav_local_position_ned_system_global_offset_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "time_boot_ms", payload->time_boot_ms);
    lua_pushtablenumber(L, "x", payload->x);
    lua_pushtablenumber(L, "y", payload->y);
    lua_pushtablenumber(L, "z", payload->z);
    lua_pushtablenumber(L, "roll", payload->roll);
    lua_pushtablenumber(L, "pitch", payload->pitch);
    lua_pushtablenumber(L, "yaw", payload->yaw);
    return;
    }
  case FASTMAVLINK_MSG_ID_HIL_STATE: { // #90
    fmav_hil_state_t* payload = (fmav_hil_state_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "time_usec", payload->time_usec);
    lua_pushtablenumber(L, "roll", payload->roll);
    lua_pushtablenumber(L, "pitch", payload->pitch);
    lua_pushtablenumber(L, "yaw", payload->yaw);
    lua_pushtablenumber(L, "rollspeed", payload->rollspeed);
    lua_pushtablenumber(L, "pitchspeed", payload->pitchspeed);
    lua_pushtablenumber(L, "yawspeed", payload->yawspeed);
    lua_pushtablenumber(L, "lat", payload->lat);
    lua_pushtablenumber(L, "lon", payload->lon);
    lua_pushtablenumber(L, "alt", payload->alt);
    lua_pushtablenumber(L, "vx", payload->vx);
    lua_pushtablenumber(L, "vy", payload->vy);
    lua_pushtablenumber(L, "vz", payload->vz);
    lua_pushtablenumber(L, "xacc", payload->xacc);
    lua_pushtablenumber(L, "yacc", payload->yacc);
    lua_pushtablenumber(L, "zacc", payload->zacc);
    return;
    }
  case FASTMAVLINK_MSG_ID_HIL_CONTROLS: { // #91
    fmav_hil_controls_t* payload = (fmav_hil_controls_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "time_usec", payload->time_usec);
    lua_pushtablenumber(L, "roll_ailerons", payload->roll_ailerons);
    lua_pushtablenumber(L, "pitch_elevator", payload->pitch_elevator);
    lua_pushtablenumber(L, "yaw_rudder", payload->yaw_rudder);
    lua_pushtablenumber(L, "throttle", payload->throttle);
    lua_pushtablenumber(L, "aux1", payload->aux1);
    lua_pushtablenumber(L, "aux2", payload->aux2);
    lua_pushtablenumber(L, "aux3", payload->aux3);
    lua_pushtablenumber(L, "aux4", payload->aux4);
    lua_pushtablenumber(L, "mode", payload->mode);
    lua_pushtablenumber(L, "nav_mode", payload->nav_mode);
    return;
    }
  case FASTMAVLINK_MSG_ID_HIL_RC_INPUTS_RAW: { // #92
    fmav_hil_rc_inputs_raw_t* payload = (fmav_hil_rc_inputs_raw_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "time_usec", payload->time_usec);
    lua_pushtablenumber(L, "chan1_raw", payload->chan1_raw);
    lua_pushtablenumber(L, "chan2_raw", payload->chan2_raw);
    lua_pushtablenumber(L, "chan3_raw", payload->chan3_raw);
    lua_pushtablenumber(L, "chan4_raw", payload->chan4_raw);
    lua_pushtablenumber(L, "chan5_raw", payload->chan5_raw);
    lua_pushtablenumber(L, "chan6_raw", payload->chan6_raw);
    lua_pushtablenumber(L, "chan7_raw", payload->chan7_raw);
    lua_pushtablenumber(L, "chan8_raw", payload->chan8_raw);
    lua_pushtablenumber(L, "chan9_raw", payload->chan9_raw);
    lua_pushtablenumber(L, "chan10_raw", payload->chan10_raw);
    lua_pushtablenumber(L, "chan11_raw", payload->chan11_raw);
    lua_pushtablenumber(L, "chan12_raw", payload->chan12_raw);
    lua_pushtablenumber(L, "rssi", payload->rssi);
    return;
    }
  case FASTMAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS: { // #93
    fmav_hil_actuator_controls_t* payload = (fmav_hil_actuator_controls_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "time_usec", payload->time_usec);
    lua_pushstring(L, "controls"); // array controls[16]
    lua_newtable(L);
    for (int i = 0; i < 16; i++) { 
      lua_pushtableinumber(L, i+1, payload->controls[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushtablenumber(L, "mode", payload->mode);
    lua_pushtablenumber(L, "flags", payload->flags);
    return;
    }
  case FASTMAVLINK_MSG_ID_OPTICAL_FLOW: { // #100
    fmav_optical_flow_t* payload = (fmav_optical_flow_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "time_usec", payload->time_usec);
    lua_pushtablenumber(L, "sensor_id", payload->sensor_id);
    lua_pushtablenumber(L, "flow_x", payload->flow_x);
    lua_pushtablenumber(L, "flow_y", payload->flow_y);
    lua_pushtablenumber(L, "flow_comp_m_x", payload->flow_comp_m_x);
    lua_pushtablenumber(L, "flow_comp_m_y", payload->flow_comp_m_y);
    lua_pushtablenumber(L, "quality", payload->quality);
    lua_pushtablenumber(L, "ground_distance", payload->ground_distance);
    lua_pushtablenumber(L, "flow_rate_x", payload->flow_rate_x);
    lua_pushtablenumber(L, "flow_rate_y", payload->flow_rate_y);
    return;
    }
  case FASTMAVLINK_MSG_ID_GLOBAL_VISION_POSITION_ESTIMATE: { // #101
    fmav_global_vision_position_estimate_t* payload = (fmav_global_vision_position_estimate_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "usec", payload->usec);
    lua_pushtablenumber(L, "x", payload->x);
    lua_pushtablenumber(L, "y", payload->y);
    lua_pushtablenumber(L, "z", payload->z);
    lua_pushtablenumber(L, "roll", payload->roll);
    lua_pushtablenumber(L, "pitch", payload->pitch);
    lua_pushtablenumber(L, "yaw", payload->yaw);
    lua_pushstring(L, "covariance"); // array covariance[21]
    lua_newtable(L);
    for (int i = 0; i < 21; i++) { 
      lua_pushtableinumber(L, i+1, payload->covariance[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushtablenumber(L, "reset_counter", payload->reset_counter);
    return;
    }
  case FASTMAVLINK_MSG_ID_VISION_POSITION_ESTIMATE: { // #102
    fmav_vision_position_estimate_t* payload = (fmav_vision_position_estimate_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "usec", payload->usec);
    lua_pushtablenumber(L, "x", payload->x);
    lua_pushtablenumber(L, "y", payload->y);
    lua_pushtablenumber(L, "z", payload->z);
    lua_pushtablenumber(L, "roll", payload->roll);
    lua_pushtablenumber(L, "pitch", payload->pitch);
    lua_pushtablenumber(L, "yaw", payload->yaw);
    lua_pushstring(L, "covariance"); // array covariance[21]
    lua_newtable(L);
    for (int i = 0; i < 21; i++) { 
      lua_pushtableinumber(L, i+1, payload->covariance[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushtablenumber(L, "reset_counter", payload->reset_counter);
    return;
    }
  case FASTMAVLINK_MSG_ID_VISION_SPEED_ESTIMATE: { // #103
    fmav_vision_speed_estimate_t* payload = (fmav_vision_speed_estimate_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "usec", payload->usec);
    lua_pushtablenumber(L, "x", payload->x);
    lua_pushtablenumber(L, "y", payload->y);
    lua_pushtablenumber(L, "z", payload->z);
    lua_pushstring(L, "covariance"); // array covariance[9]
    lua_newtable(L);
    for (int i = 0; i < 9; i++) { 
      lua_pushtableinumber(L, i+1, payload->covariance[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushtablenumber(L, "reset_counter", payload->reset_counter);
    return;
    }
  case FASTMAVLINK_MSG_ID_VICON_POSITION_ESTIMATE: { // #104
    fmav_vicon_position_estimate_t* payload = (fmav_vicon_position_estimate_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "usec", payload->usec);
    lua_pushtablenumber(L, "x", payload->x);
    lua_pushtablenumber(L, "y", payload->y);
    lua_pushtablenumber(L, "z", payload->z);
    lua_pushtablenumber(L, "roll", payload->roll);
    lua_pushtablenumber(L, "pitch", payload->pitch);
    lua_pushtablenumber(L, "yaw", payload->yaw);
    lua_pushstring(L, "covariance"); // array covariance[21]
    lua_newtable(L);
    for (int i = 0; i < 21; i++) { 
      lua_pushtableinumber(L, i+1, payload->covariance[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    return;
    }
  case FASTMAVLINK_MSG_ID_HIGHRES_IMU: { // #105
    fmav_highres_imu_t* payload = (fmav_highres_imu_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "time_usec", payload->time_usec);
    lua_pushtablenumber(L, "xacc", payload->xacc);
    lua_pushtablenumber(L, "yacc", payload->yacc);
    lua_pushtablenumber(L, "zacc", payload->zacc);
    lua_pushtablenumber(L, "xgyro", payload->xgyro);
    lua_pushtablenumber(L, "ygyro", payload->ygyro);
    lua_pushtablenumber(L, "zgyro", payload->zgyro);
    lua_pushtablenumber(L, "xmag", payload->xmag);
    lua_pushtablenumber(L, "ymag", payload->ymag);
    lua_pushtablenumber(L, "zmag", payload->zmag);
    lua_pushtablenumber(L, "abs_pressure", payload->abs_pressure);
    lua_pushtablenumber(L, "diff_pressure", payload->diff_pressure);
    lua_pushtablenumber(L, "pressure_alt", payload->pressure_alt);
    lua_pushtablenumber(L, "temperature", payload->temperature);
    lua_pushtablenumber(L, "fields_updated", payload->fields_updated);
    lua_pushtablenumber(L, "id", payload->id);
    return;
    }
  case FASTMAVLINK_MSG_ID_OPTICAL_FLOW_RAD: { // #106
    fmav_optical_flow_rad_t* payload = (fmav_optical_flow_rad_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "time_usec", payload->time_usec);
    lua_pushtablenumber(L, "sensor_id", payload->sensor_id);
    lua_pushtablenumber(L, "integration_time_us", payload->integration_time_us);
    lua_pushtablenumber(L, "integrated_x", payload->integrated_x);
    lua_pushtablenumber(L, "integrated_y", payload->integrated_y);
    lua_pushtablenumber(L, "integrated_xgyro", payload->integrated_xgyro);
    lua_pushtablenumber(L, "integrated_ygyro", payload->integrated_ygyro);
    lua_pushtablenumber(L, "integrated_zgyro", payload->integrated_zgyro);
    lua_pushtablenumber(L, "temperature", payload->temperature);
    lua_pushtablenumber(L, "quality", payload->quality);
    lua_pushtablenumber(L, "time_delta_distance_us", payload->time_delta_distance_us);
    lua_pushtablenumber(L, "distance", payload->distance);
    return;
    }
  case FASTMAVLINK_MSG_ID_HIL_SENSOR: { // #107
    fmav_hil_sensor_t* payload = (fmav_hil_sensor_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "time_usec", payload->time_usec);
    lua_pushtablenumber(L, "xacc", payload->xacc);
    lua_pushtablenumber(L, "yacc", payload->yacc);
    lua_pushtablenumber(L, "zacc", payload->zacc);
    lua_pushtablenumber(L, "xgyro", payload->xgyro);
    lua_pushtablenumber(L, "ygyro", payload->ygyro);
    lua_pushtablenumber(L, "zgyro", payload->zgyro);
    lua_pushtablenumber(L, "xmag", payload->xmag);
    lua_pushtablenumber(L, "ymag", payload->ymag);
    lua_pushtablenumber(L, "zmag", payload->zmag);
    lua_pushtablenumber(L, "abs_pressure", payload->abs_pressure);
    lua_pushtablenumber(L, "diff_pressure", payload->diff_pressure);
    lua_pushtablenumber(L, "pressure_alt", payload->pressure_alt);
    lua_pushtablenumber(L, "temperature", payload->temperature);
    lua_pushtablenumber(L, "fields_updated", payload->fields_updated);
    lua_pushtablenumber(L, "id", payload->id);
    return;
    }
  case FASTMAVLINK_MSG_ID_SIM_STATE: { // #108
    fmav_sim_state_t* payload = (fmav_sim_state_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "q1", payload->q1);
    lua_pushtablenumber(L, "q2", payload->q2);
    lua_pushtablenumber(L, "q3", payload->q3);
    lua_pushtablenumber(L, "q4", payload->q4);
    lua_pushtablenumber(L, "roll", payload->roll);
    lua_pushtablenumber(L, "pitch", payload->pitch);
    lua_pushtablenumber(L, "yaw", payload->yaw);
    lua_pushtablenumber(L, "xacc", payload->xacc);
    lua_pushtablenumber(L, "yacc", payload->yacc);
    lua_pushtablenumber(L, "zacc", payload->zacc);
    lua_pushtablenumber(L, "xgyro", payload->xgyro);
    lua_pushtablenumber(L, "ygyro", payload->ygyro);
    lua_pushtablenumber(L, "zgyro", payload->zgyro);
    lua_pushtablenumber(L, "lat", payload->lat);
    lua_pushtablenumber(L, "lon", payload->lon);
    lua_pushtablenumber(L, "alt", payload->alt);
    lua_pushtablenumber(L, "std_dev_horz", payload->std_dev_horz);
    lua_pushtablenumber(L, "std_dev_vert", payload->std_dev_vert);
    lua_pushtablenumber(L, "vn", payload->vn);
    lua_pushtablenumber(L, "ve", payload->ve);
    lua_pushtablenumber(L, "vd", payload->vd);
    lua_pushtablenumber(L, "lat_int", payload->lat_int);
    lua_pushtablenumber(L, "lon_int", payload->lon_int);
    return;
    }
  case FASTMAVLINK_MSG_ID_RADIO_STATUS: { // #109
    fmav_radio_status_t* payload = (fmav_radio_status_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "rssi", payload->rssi);
    lua_pushtablenumber(L, "remrssi", payload->remrssi);
    lua_pushtablenumber(L, "txbuf", payload->txbuf);
    lua_pushtablenumber(L, "noise", payload->noise);
    lua_pushtablenumber(L, "remnoise", payload->remnoise);
    lua_pushtablenumber(L, "rxerrors", payload->rxerrors);
    lua_pushtablenumber(L, "fixed", payload->fixed);
    return;
    }
  case FASTMAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL: { // #110
    fmav_file_transfer_protocol_t* payload = (fmav_file_transfer_protocol_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "target_network", payload->target_network);
    lua_pushstring(L, "payload"); // array payload[251]
    lua_newtable(L);
    for (int i = 0; i < 251; i++) { 
      lua_pushtableinumber(L, i+1, payload->payload[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    return;
    }
  case FASTMAVLINK_MSG_ID_TIMESYNC: { // #111
    fmav_timesync_t* payload = (fmav_timesync_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "tc1", payload->tc1);
    lua_pushtablenumber(L, "ts1", payload->ts1);
    return;
    }
  case FASTMAVLINK_MSG_ID_CAMERA_TRIGGER: { // #112
    fmav_camera_trigger_t* payload = (fmav_camera_trigger_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "time_usec", payload->time_usec);
    lua_pushtablenumber(L, "seq", payload->seq);
    return;
    }
  case FASTMAVLINK_MSG_ID_HIL_GPS: { // #113
    fmav_hil_gps_t* payload = (fmav_hil_gps_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "time_usec", payload->time_usec);
    lua_pushtablenumber(L, "fix_type", payload->fix_type);
    lua_pushtablenumber(L, "lat", payload->lat);
    lua_pushtablenumber(L, "lon", payload->lon);
    lua_pushtablenumber(L, "alt", payload->alt);
    lua_pushtablenumber(L, "eph", payload->eph);
    lua_pushtablenumber(L, "epv", payload->epv);
    lua_pushtablenumber(L, "vel", payload->vel);
    lua_pushtablenumber(L, "vn", payload->vn);
    lua_pushtablenumber(L, "ve", payload->ve);
    lua_pushtablenumber(L, "vd", payload->vd);
    lua_pushtablenumber(L, "cog", payload->cog);
    lua_pushtablenumber(L, "satellites_visible", payload->satellites_visible);
    lua_pushtablenumber(L, "id", payload->id);
    lua_pushtablenumber(L, "yaw", payload->yaw);
    return;
    }
  case FASTMAVLINK_MSG_ID_HIL_OPTICAL_FLOW: { // #114
    fmav_hil_optical_flow_t* payload = (fmav_hil_optical_flow_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "time_usec", payload->time_usec);
    lua_pushtablenumber(L, "sensor_id", payload->sensor_id);
    lua_pushtablenumber(L, "integration_time_us", payload->integration_time_us);
    lua_pushtablenumber(L, "integrated_x", payload->integrated_x);
    lua_pushtablenumber(L, "integrated_y", payload->integrated_y);
    lua_pushtablenumber(L, "integrated_xgyro", payload->integrated_xgyro);
    lua_pushtablenumber(L, "integrated_ygyro", payload->integrated_ygyro);
    lua_pushtablenumber(L, "integrated_zgyro", payload->integrated_zgyro);
    lua_pushtablenumber(L, "temperature", payload->temperature);
    lua_pushtablenumber(L, "quality", payload->quality);
    lua_pushtablenumber(L, "time_delta_distance_us", payload->time_delta_distance_us);
    lua_pushtablenumber(L, "distance", payload->distance);
    return;
    }
  case FASTMAVLINK_MSG_ID_HIL_STATE_QUATERNION: { // #115
    fmav_hil_state_quaternion_t* payload = (fmav_hil_state_quaternion_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "time_usec", payload->time_usec);
    lua_pushstring(L, "attitude_quaternion"); // array attitude_quaternion[4]
    lua_newtable(L);
    for (int i = 0; i < 4; i++) { 
      lua_pushtableinumber(L, i+1, payload->attitude_quaternion[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushtablenumber(L, "rollspeed", payload->rollspeed);
    lua_pushtablenumber(L, "pitchspeed", payload->pitchspeed);
    lua_pushtablenumber(L, "yawspeed", payload->yawspeed);
    lua_pushtablenumber(L, "lat", payload->lat);
    lua_pushtablenumber(L, "lon", payload->lon);
    lua_pushtablenumber(L, "alt", payload->alt);
    lua_pushtablenumber(L, "vx", payload->vx);
    lua_pushtablenumber(L, "vy", payload->vy);
    lua_pushtablenumber(L, "vz", payload->vz);
    lua_pushtablenumber(L, "ind_airspeed", payload->ind_airspeed);
    lua_pushtablenumber(L, "true_airspeed", payload->true_airspeed);
    lua_pushtablenumber(L, "xacc", payload->xacc);
    lua_pushtablenumber(L, "yacc", payload->yacc);
    lua_pushtablenumber(L, "zacc", payload->zacc);
    return;
    }
  case FASTMAVLINK_MSG_ID_SCALED_IMU2: { // #116
    fmav_scaled_imu2_t* payload = (fmav_scaled_imu2_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "time_boot_ms", payload->time_boot_ms);
    lua_pushtablenumber(L, "xacc", payload->xacc);
    lua_pushtablenumber(L, "yacc", payload->yacc);
    lua_pushtablenumber(L, "zacc", payload->zacc);
    lua_pushtablenumber(L, "xgyro", payload->xgyro);
    lua_pushtablenumber(L, "ygyro", payload->ygyro);
    lua_pushtablenumber(L, "zgyro", payload->zgyro);
    lua_pushtablenumber(L, "xmag", payload->xmag);
    lua_pushtablenumber(L, "ymag", payload->ymag);
    lua_pushtablenumber(L, "zmag", payload->zmag);
    lua_pushtablenumber(L, "temperature", payload->temperature);
    return;
    }
  case FASTMAVLINK_MSG_ID_LOG_REQUEST_LIST: { // #117
    fmav_log_request_list_t* payload = (fmav_log_request_list_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "start", payload->start);
    lua_pushtablenumber(L, "end", payload->end);
    return;
    }
  case FASTMAVLINK_MSG_ID_LOG_ENTRY: { // #118
    fmav_log_entry_t* payload = (fmav_log_entry_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "id", payload->id);
    lua_pushtablenumber(L, "num_logs", payload->num_logs);
    lua_pushtablenumber(L, "last_log_num", payload->last_log_num);
    lua_pushtablenumber(L, "time_utc", payload->time_utc);
    lua_pushtablenumber(L, "size", payload->size);
    return;
    }
  case FASTMAVLINK_MSG_ID_LOG_REQUEST_DATA: { // #119
    fmav_log_request_data_t* payload = (fmav_log_request_data_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "id", payload->id);
    lua_pushtablenumber(L, "ofs", payload->ofs);
    lua_pushtablenumber(L, "count", payload->count);
    return;
    }
  case FASTMAVLINK_MSG_ID_LOG_DATA: { // #120
    fmav_log_data_t* payload = (fmav_log_data_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "id", payload->id);
    lua_pushtablenumber(L, "ofs", payload->ofs);
    lua_pushtablenumber(L, "count", payload->count);
    lua_pushstring(L, "data"); // array data[90]
    lua_newtable(L);
    for (int i = 0; i < 90; i++) { 
      lua_pushtableinumber(L, i+1, payload->data[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    return;
    }
  case FASTMAVLINK_MSG_ID_LOG_ERASE: { // #121
    return;
    }
  case FASTMAVLINK_MSG_ID_LOG_REQUEST_END: { // #122
    return;
    }
  case FASTMAVLINK_MSG_ID_GPS_INJECT_DATA: { // #123
    fmav_gps_inject_data_t* payload = (fmav_gps_inject_data_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "len", payload->len);
    lua_pushstring(L, "data"); // array data[110]
    lua_newtable(L);
    for (int i = 0; i < 110; i++) { 
      lua_pushtableinumber(L, i+1, payload->data[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    return;
    }
  case FASTMAVLINK_MSG_ID_GPS2_RAW: { // #124
    fmav_gps2_raw_t* payload = (fmav_gps2_raw_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "time_usec", payload->time_usec);
    lua_pushtablenumber(L, "fix_type", payload->fix_type);
    lua_pushtablenumber(L, "lat", payload->lat);
    lua_pushtablenumber(L, "lon", payload->lon);
    lua_pushtablenumber(L, "alt", payload->alt);
    lua_pushtablenumber(L, "eph", payload->eph);
    lua_pushtablenumber(L, "epv", payload->epv);
    lua_pushtablenumber(L, "vel", payload->vel);
    lua_pushtablenumber(L, "cog", payload->cog);
    lua_pushtablenumber(L, "satellites_visible", payload->satellites_visible);
    lua_pushtablenumber(L, "dgps_numch", payload->dgps_numch);
    lua_pushtablenumber(L, "dgps_age", payload->dgps_age);
    lua_pushtablenumber(L, "yaw", payload->yaw);
    lua_pushtablenumber(L, "alt_ellipsoid", payload->alt_ellipsoid);
    lua_pushtablenumber(L, "h_acc", payload->h_acc);
    lua_pushtablenumber(L, "v_acc", payload->v_acc);
    lua_pushtablenumber(L, "vel_acc", payload->vel_acc);
    lua_pushtablenumber(L, "hdg_acc", payload->hdg_acc);
    return;
    }
  case FASTMAVLINK_MSG_ID_POWER_STATUS: { // #125
    fmav_power_status_t* payload = (fmav_power_status_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "Vcc", payload->Vcc);
    lua_pushtablenumber(L, "Vservo", payload->Vservo);
    lua_pushtablenumber(L, "flags", payload->flags);
    return;
    }
  case FASTMAVLINK_MSG_ID_SERIAL_CONTROL: { // #126
    fmav_serial_control_t* payload = (fmav_serial_control_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "device", payload->device);
    lua_pushtablenumber(L, "flags", payload->flags);
    lua_pushtablenumber(L, "timeout", payload->timeout);
    lua_pushtablenumber(L, "baudrate", payload->baudrate);
    lua_pushtablenumber(L, "count", payload->count);
    lua_pushstring(L, "data"); // array data[70]
    lua_newtable(L);
    for (int i = 0; i < 70; i++) { 
      lua_pushtableinumber(L, i+1, payload->data[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    return;
    }
  case FASTMAVLINK_MSG_ID_GPS_RTK: { // #127
    fmav_gps_rtk_t* payload = (fmav_gps_rtk_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "time_last_baseline_ms", payload->time_last_baseline_ms);
    lua_pushtablenumber(L, "rtk_receiver_id", payload->rtk_receiver_id);
    lua_pushtablenumber(L, "wn", payload->wn);
    lua_pushtablenumber(L, "tow", payload->tow);
    lua_pushtablenumber(L, "rtk_health", payload->rtk_health);
    lua_pushtablenumber(L, "rtk_rate", payload->rtk_rate);
    lua_pushtablenumber(L, "nsats", payload->nsats);
    lua_pushtablenumber(L, "baseline_coords_type", payload->baseline_coords_type);
    lua_pushtablenumber(L, "baseline_a_mm", payload->baseline_a_mm);
    lua_pushtablenumber(L, "baseline_b_mm", payload->baseline_b_mm);
    lua_pushtablenumber(L, "baseline_c_mm", payload->baseline_c_mm);
    lua_pushtablenumber(L, "accuracy", payload->accuracy);
    lua_pushtablenumber(L, "iar_num_hypotheses", payload->iar_num_hypotheses);
    return;
    }
  case FASTMAVLINK_MSG_ID_GPS2_RTK: { // #128
    fmav_gps2_rtk_t* payload = (fmav_gps2_rtk_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "time_last_baseline_ms", payload->time_last_baseline_ms);
    lua_pushtablenumber(L, "rtk_receiver_id", payload->rtk_receiver_id);
    lua_pushtablenumber(L, "wn", payload->wn);
    lua_pushtablenumber(L, "tow", payload->tow);
    lua_pushtablenumber(L, "rtk_health", payload->rtk_health);
    lua_pushtablenumber(L, "rtk_rate", payload->rtk_rate);
    lua_pushtablenumber(L, "nsats", payload->nsats);
    lua_pushtablenumber(L, "baseline_coords_type", payload->baseline_coords_type);
    lua_pushtablenumber(L, "baseline_a_mm", payload->baseline_a_mm);
    lua_pushtablenumber(L, "baseline_b_mm", payload->baseline_b_mm);
    lua_pushtablenumber(L, "baseline_c_mm", payload->baseline_c_mm);
    lua_pushtablenumber(L, "accuracy", payload->accuracy);
    lua_pushtablenumber(L, "iar_num_hypotheses", payload->iar_num_hypotheses);
    return;
    }
  case FASTMAVLINK_MSG_ID_SCALED_IMU3: { // #129
    fmav_scaled_imu3_t* payload = (fmav_scaled_imu3_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "time_boot_ms", payload->time_boot_ms);
    lua_pushtablenumber(L, "xacc", payload->xacc);
    lua_pushtablenumber(L, "yacc", payload->yacc);
    lua_pushtablenumber(L, "zacc", payload->zacc);
    lua_pushtablenumber(L, "xgyro", payload->xgyro);
    lua_pushtablenumber(L, "ygyro", payload->ygyro);
    lua_pushtablenumber(L, "zgyro", payload->zgyro);
    lua_pushtablenumber(L, "xmag", payload->xmag);
    lua_pushtablenumber(L, "ymag", payload->ymag);
    lua_pushtablenumber(L, "zmag", payload->zmag);
    lua_pushtablenumber(L, "temperature", payload->temperature);
    return;
    }
  case FASTMAVLINK_MSG_ID_DATA_TRANSMISSION_HANDSHAKE: { // #130
    fmav_data_transmission_handshake_t* payload = (fmav_data_transmission_handshake_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "type", payload->type);
    lua_pushtablenumber(L, "size", payload->size);
    lua_pushtablenumber(L, "width", payload->width);
    lua_pushtablenumber(L, "height", payload->height);
    lua_pushtablenumber(L, "packets", payload->packets);
    lua_pushtablenumber(L, "payload", payload->payload);
    lua_pushtablenumber(L, "jpg_quality", payload->jpg_quality);
    return;
    }
  case FASTMAVLINK_MSG_ID_ENCAPSULATED_DATA: { // #131
    fmav_encapsulated_data_t* payload = (fmav_encapsulated_data_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "seqnr", payload->seqnr);
    lua_pushstring(L, "data"); // array data[253]
    lua_newtable(L);
    for (int i = 0; i < 253; i++) { 
      lua_pushtableinumber(L, i+1, payload->data[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    return;
    }
  case FASTMAVLINK_MSG_ID_DISTANCE_SENSOR: { // #132
    fmav_distance_sensor_t* payload = (fmav_distance_sensor_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "time_boot_ms", payload->time_boot_ms);
    lua_pushtablenumber(L, "min_distance", payload->min_distance);
    lua_pushtablenumber(L, "max_distance", payload->max_distance);
    lua_pushtablenumber(L, "current_distance", payload->current_distance);
    lua_pushtablenumber(L, "type", payload->type);
    lua_pushtablenumber(L, "id", payload->id);
    lua_pushtablenumber(L, "orientation", payload->orientation);
    lua_pushtablenumber(L, "covariance", payload->covariance);
    lua_pushtablenumber(L, "horizontal_fov", payload->horizontal_fov);
    lua_pushtablenumber(L, "vertical_fov", payload->vertical_fov);
    lua_pushstring(L, "quaternion"); // array quaternion[4]
    lua_newtable(L);
    for (int i = 0; i < 4; i++) { 
      lua_pushtableinumber(L, i+1, payload->quaternion[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushtablenumber(L, "signal_quality", payload->signal_quality);
    return;
    }
  case FASTMAVLINK_MSG_ID_TERRAIN_REQUEST: { // #133
    fmav_terrain_request_t* payload = (fmav_terrain_request_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "lat", payload->lat);
    lua_pushtablenumber(L, "lon", payload->lon);
    lua_pushtablenumber(L, "grid_spacing", payload->grid_spacing);
    lua_pushtablenumber(L, "mask", payload->mask);
    return;
    }
  case FASTMAVLINK_MSG_ID_TERRAIN_DATA: { // #134
    fmav_terrain_data_t* payload = (fmav_terrain_data_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "lat", payload->lat);
    lua_pushtablenumber(L, "lon", payload->lon);
    lua_pushtablenumber(L, "grid_spacing", payload->grid_spacing);
    lua_pushtablenumber(L, "gridbit", payload->gridbit);
    lua_pushstring(L, "data"); // array data[16]
    lua_newtable(L);
    for (int i = 0; i < 16; i++) { 
      lua_pushtableinumber(L, i+1, payload->data[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    return;
    }
  case FASTMAVLINK_MSG_ID_TERRAIN_CHECK: { // #135
    fmav_terrain_check_t* payload = (fmav_terrain_check_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "lat", payload->lat);
    lua_pushtablenumber(L, "lon", payload->lon);
    return;
    }
  case FASTMAVLINK_MSG_ID_TERRAIN_REPORT: { // #136
    fmav_terrain_report_t* payload = (fmav_terrain_report_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "lat", payload->lat);
    lua_pushtablenumber(L, "lon", payload->lon);
    lua_pushtablenumber(L, "spacing", payload->spacing);
    lua_pushtablenumber(L, "terrain_height", payload->terrain_height);
    lua_pushtablenumber(L, "current_height", payload->current_height);
    lua_pushtablenumber(L, "pending", payload->pending);
    lua_pushtablenumber(L, "loaded", payload->loaded);
    return;
    }
  case FASTMAVLINK_MSG_ID_SCALED_PRESSURE2: { // #137
    fmav_scaled_pressure2_t* payload = (fmav_scaled_pressure2_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "time_boot_ms", payload->time_boot_ms);
    lua_pushtablenumber(L, "press_abs", payload->press_abs);
    lua_pushtablenumber(L, "press_diff", payload->press_diff);
    lua_pushtablenumber(L, "temperature", payload->temperature);
    lua_pushtablenumber(L, "temperature_press_diff", payload->temperature_press_diff);
    return;
    }
  case FASTMAVLINK_MSG_ID_ATT_POS_MOCAP: { // #138
    fmav_att_pos_mocap_t* payload = (fmav_att_pos_mocap_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "time_usec", payload->time_usec);
    lua_pushstring(L, "q"); // array q[4]
    lua_newtable(L);
    for (int i = 0; i < 4; i++) { 
      lua_pushtableinumber(L, i+1, payload->q[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushtablenumber(L, "x", payload->x);
    lua_pushtablenumber(L, "y", payload->y);
    lua_pushtablenumber(L, "z", payload->z);
    lua_pushstring(L, "covariance"); // array covariance[21]
    lua_newtable(L);
    for (int i = 0; i < 21; i++) { 
      lua_pushtableinumber(L, i+1, payload->covariance[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    return;
    }
  case FASTMAVLINK_MSG_ID_SET_ACTUATOR_CONTROL_TARGET: { // #139
    fmav_set_actuator_control_target_t* payload = (fmav_set_actuator_control_target_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "time_usec", payload->time_usec);
    lua_pushtablenumber(L, "group_mlx", payload->group_mlx);
    lua_pushstring(L, "controls"); // array controls[8]
    lua_newtable(L);
    for (int i = 0; i < 8; i++) { 
      lua_pushtableinumber(L, i+1, payload->controls[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    return;
    }
  case FASTMAVLINK_MSG_ID_ACTUATOR_CONTROL_TARGET: { // #140
    fmav_actuator_control_target_t* payload = (fmav_actuator_control_target_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "time_usec", payload->time_usec);
    lua_pushtablenumber(L, "group_mlx", payload->group_mlx);
    lua_pushstring(L, "controls"); // array controls[8]
    lua_newtable(L);
    for (int i = 0; i < 8; i++) { 
      lua_pushtableinumber(L, i+1, payload->controls[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    return;
    }
  case FASTMAVLINK_MSG_ID_ALTITUDE: { // #141
    fmav_altitude_t* payload = (fmav_altitude_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "time_usec", payload->time_usec);
    lua_pushtablenumber(L, "altitude_monotonic", payload->altitude_monotonic);
    lua_pushtablenumber(L, "altitude_amsl", payload->altitude_amsl);
    lua_pushtablenumber(L, "altitude_local", payload->altitude_local);
    lua_pushtablenumber(L, "altitude_relative", payload->altitude_relative);
    lua_pushtablenumber(L, "altitude_terrain", payload->altitude_terrain);
    lua_pushtablenumber(L, "bottom_clearance", payload->bottom_clearance);
    return;
    }
  case FASTMAVLINK_MSG_ID_RESOURCE_REQUEST: { // #142
    fmav_resource_request_t* payload = (fmav_resource_request_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "request_id", payload->request_id);
    lua_pushtablenumber(L, "uri_type", payload->uri_type);
    lua_pushstring(L, "uri"); // array uri[120]
    lua_newtable(L);
    for (int i = 0; i < 120; i++) { 
      lua_pushtableinumber(L, i+1, payload->uri[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushtablenumber(L, "transfer_type", payload->transfer_type);
    lua_pushstring(L, "storage"); // array storage[120]
    lua_newtable(L);
    for (int i = 0; i < 120; i++) { 
      lua_pushtableinumber(L, i+1, payload->storage[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    return;
    }
  case FASTMAVLINK_MSG_ID_SCALED_PRESSURE3: { // #143
    fmav_scaled_pressure3_t* payload = (fmav_scaled_pressure3_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "time_boot_ms", payload->time_boot_ms);
    lua_pushtablenumber(L, "press_abs", payload->press_abs);
    lua_pushtablenumber(L, "press_diff", payload->press_diff);
    lua_pushtablenumber(L, "temperature", payload->temperature);
    lua_pushtablenumber(L, "temperature_press_diff", payload->temperature_press_diff);
    return;
    }
  case FASTMAVLINK_MSG_ID_FOLLOW_TARGET: { // #144
    fmav_follow_target_t* payload = (fmav_follow_target_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "timestamp", payload->timestamp);
    lua_pushtablenumber(L, "est_capabilities", payload->est_capabilities);
    lua_pushtablenumber(L, "lat", payload->lat);
    lua_pushtablenumber(L, "lon", payload->lon);
    lua_pushtablenumber(L, "alt", payload->alt);
    lua_pushstring(L, "vel"); // array vel[3]
    lua_newtable(L);
    for (int i = 0; i < 3; i++) { 
      lua_pushtableinumber(L, i+1, payload->vel[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushstring(L, "acc"); // array acc[3]
    lua_newtable(L);
    for (int i = 0; i < 3; i++) { 
      lua_pushtableinumber(L, i+1, payload->acc[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushstring(L, "attitude_q"); // array attitude_q[4]
    lua_newtable(L);
    for (int i = 0; i < 4; i++) { 
      lua_pushtableinumber(L, i+1, payload->attitude_q[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushstring(L, "rates"); // array rates[3]
    lua_newtable(L);
    for (int i = 0; i < 3; i++) { 
      lua_pushtableinumber(L, i+1, payload->rates[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushstring(L, "position_cov"); // array position_cov[3]
    lua_newtable(L);
    for (int i = 0; i < 3; i++) { 
      lua_pushtableinumber(L, i+1, payload->position_cov[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushtablenumber(L, "custom_state", payload->custom_state);
    return;
    }
  case FASTMAVLINK_MSG_ID_CONTROL_SYSTEM_STATE: { // #146
    fmav_control_system_state_t* payload = (fmav_control_system_state_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "time_usec", payload->time_usec);
    lua_pushtablenumber(L, "x_acc", payload->x_acc);
    lua_pushtablenumber(L, "y_acc", payload->y_acc);
    lua_pushtablenumber(L, "z_acc", payload->z_acc);
    lua_pushtablenumber(L, "x_vel", payload->x_vel);
    lua_pushtablenumber(L, "y_vel", payload->y_vel);
    lua_pushtablenumber(L, "z_vel", payload->z_vel);
    lua_pushtablenumber(L, "x_pos", payload->x_pos);
    lua_pushtablenumber(L, "y_pos", payload->y_pos);
    lua_pushtablenumber(L, "z_pos", payload->z_pos);
    lua_pushtablenumber(L, "airspeed", payload->airspeed);
    lua_pushstring(L, "vel_variance"); // array vel_variance[3]
    lua_newtable(L);
    for (int i = 0; i < 3; i++) { 
      lua_pushtableinumber(L, i+1, payload->vel_variance[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushstring(L, "pos_variance"); // array pos_variance[3]
    lua_newtable(L);
    for (int i = 0; i < 3; i++) { 
      lua_pushtableinumber(L, i+1, payload->pos_variance[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushstring(L, "q"); // array q[4]
    lua_newtable(L);
    for (int i = 0; i < 4; i++) { 
      lua_pushtableinumber(L, i+1, payload->q[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushtablenumber(L, "roll_rate", payload->roll_rate);
    lua_pushtablenumber(L, "pitch_rate", payload->pitch_rate);
    lua_pushtablenumber(L, "yaw_rate", payload->yaw_rate);
    return;
    }
  case FASTMAVLINK_MSG_ID_BATTERY_STATUS: { // #147
    fmav_battery_status_t* payload = (fmav_battery_status_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "id", payload->id);
    lua_pushtablenumber(L, "battery_function", payload->battery_function);
    lua_pushtablenumber(L, "type", payload->type);
    lua_pushtablenumber(L, "temperature", payload->temperature);
    lua_pushstring(L, "voltages"); // array voltages[10]
    lua_newtable(L);
    for (int i = 0; i < 10; i++) { 
      lua_pushtableinumber(L, i+1, payload->voltages[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushtablenumber(L, "current_battery", payload->current_battery);
    lua_pushtablenumber(L, "current_consumed", payload->current_consumed);
    lua_pushtablenumber(L, "energy_consumed", payload->energy_consumed);
    lua_pushtablenumber(L, "battery_remaining", payload->battery_remaining);
    lua_pushtablenumber(L, "time_remaining", payload->time_remaining);
    lua_pushtablenumber(L, "charge_state", payload->charge_state);
    lua_pushstring(L, "voltages_ext"); // array voltages_ext[4]
    lua_newtable(L);
    for (int i = 0; i < 4; i++) { 
      lua_pushtableinumber(L, i+1, payload->voltages_ext[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushtablenumber(L, "mode", payload->mode);
    lua_pushtablenumber(L, "fault_bitmask", payload->fault_bitmask);
    return;
    }
  case FASTMAVLINK_MSG_ID_AUTOPILOT_VERSION: { // #148
    fmav_autopilot_version_t* payload = (fmav_autopilot_version_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "capabilities", payload->capabilities);
    lua_pushtablenumber(L, "flight_sw_version", payload->flight_sw_version);
    lua_pushtablenumber(L, "middleware_sw_version", payload->middleware_sw_version);
    lua_pushtablenumber(L, "os_sw_version", payload->os_sw_version);
    lua_pushtablenumber(L, "board_version", payload->board_version);
    lua_pushstring(L, "flight_custom_version"); // array flight_custom_version[8]
    lua_newtable(L);
    for (int i = 0; i < 8; i++) { 
      lua_pushtableinumber(L, i+1, payload->flight_custom_version[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushstring(L, "middleware_custom_version"); // array middleware_custom_version[8]
    lua_newtable(L);
    for (int i = 0; i < 8; i++) { 
      lua_pushtableinumber(L, i+1, payload->middleware_custom_version[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushstring(L, "os_custom_version"); // array os_custom_version[8]
    lua_newtable(L);
    for (int i = 0; i < 8; i++) { 
      lua_pushtableinumber(L, i+1, payload->os_custom_version[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushtablenumber(L, "vendor_id", payload->vendor_id);
    lua_pushtablenumber(L, "product_id", payload->product_id);
    lua_pushtablenumber(L, "uid", payload->uid);
    lua_pushstring(L, "uid2"); // array uid2[18]
    lua_newtable(L);
    for (int i = 0; i < 18; i++) { 
      lua_pushtableinumber(L, i+1, payload->uid2[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    return;
    }
  case FASTMAVLINK_MSG_ID_LANDING_TARGET: { // #149
    fmav_landing_target_t* payload = (fmav_landing_target_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "time_usec", payload->time_usec);
    lua_pushtablenumber(L, "target_num", payload->target_num);
    lua_pushtablenumber(L, "frame", payload->frame);
    lua_pushtablenumber(L, "angle_x", payload->angle_x);
    lua_pushtablenumber(L, "angle_y", payload->angle_y);
    lua_pushtablenumber(L, "distance", payload->distance);
    lua_pushtablenumber(L, "size_x", payload->size_x);
    lua_pushtablenumber(L, "size_y", payload->size_y);
    lua_pushtablenumber(L, "x", payload->x);
    lua_pushtablenumber(L, "y", payload->y);
    lua_pushtablenumber(L, "z", payload->z);
    lua_pushstring(L, "q"); // array q[4]
    lua_newtable(L);
    for (int i = 0; i < 4; i++) { 
      lua_pushtableinumber(L, i+1, payload->q[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushtablenumber(L, "type", payload->type);
    lua_pushtablenumber(L, "position_valid", payload->position_valid);
    return;
    }
  case FASTMAVLINK_MSG_ID_SENSOR_OFFSETS: { // #150
    fmav_sensor_offsets_t* payload = (fmav_sensor_offsets_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "mag_ofs_x", payload->mag_ofs_x);
    lua_pushtablenumber(L, "mag_ofs_y", payload->mag_ofs_y);
    lua_pushtablenumber(L, "mag_ofs_z", payload->mag_ofs_z);
    lua_pushtablenumber(L, "mag_declination", payload->mag_declination);
    lua_pushtablenumber(L, "raw_press", payload->raw_press);
    lua_pushtablenumber(L, "raw_temp", payload->raw_temp);
    lua_pushtablenumber(L, "gyro_cal_x", payload->gyro_cal_x);
    lua_pushtablenumber(L, "gyro_cal_y", payload->gyro_cal_y);
    lua_pushtablenumber(L, "gyro_cal_z", payload->gyro_cal_z);
    lua_pushtablenumber(L, "accel_cal_x", payload->accel_cal_x);
    lua_pushtablenumber(L, "accel_cal_y", payload->accel_cal_y);
    lua_pushtablenumber(L, "accel_cal_z", payload->accel_cal_z);
    return;
    }
  case FASTMAVLINK_MSG_ID_SET_MAG_OFFSETS: { // #151
    fmav_set_mag_offsets_t* payload = (fmav_set_mag_offsets_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "mag_ofs_x", payload->mag_ofs_x);
    lua_pushtablenumber(L, "mag_ofs_y", payload->mag_ofs_y);
    lua_pushtablenumber(L, "mag_ofs_z", payload->mag_ofs_z);
    return;
    }
  case FASTMAVLINK_MSG_ID_MEMINFO: { // #152
    fmav_meminfo_t* payload = (fmav_meminfo_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "brkval", payload->brkval);
    lua_pushtablenumber(L, "freemem", payload->freemem);
    lua_pushtablenumber(L, "freemem32", payload->freemem32);
    return;
    }
  case FASTMAVLINK_MSG_ID_AP_ADC: { // #153
    fmav_ap_adc_t* payload = (fmav_ap_adc_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "adc1", payload->adc1);
    lua_pushtablenumber(L, "adc2", payload->adc2);
    lua_pushtablenumber(L, "adc3", payload->adc3);
    lua_pushtablenumber(L, "adc4", payload->adc4);
    lua_pushtablenumber(L, "adc5", payload->adc5);
    lua_pushtablenumber(L, "adc6", payload->adc6);
    return;
    }
  case FASTMAVLINK_MSG_ID_DIGICAM_CONFIGURE: { // #154
    fmav_digicam_configure_t* payload = (fmav_digicam_configure_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "mode", payload->mode);
    lua_pushtablenumber(L, "shutter_speed", payload->shutter_speed);
    lua_pushtablenumber(L, "aperture", payload->aperture);
    lua_pushtablenumber(L, "iso", payload->iso);
    lua_pushtablenumber(L, "exposure_type", payload->exposure_type);
    lua_pushtablenumber(L, "command_id", payload->command_id);
    lua_pushtablenumber(L, "engine_cut_off", payload->engine_cut_off);
    lua_pushtablenumber(L, "extra_param", payload->extra_param);
    lua_pushtablenumber(L, "extra_value", payload->extra_value);
    return;
    }
  case FASTMAVLINK_MSG_ID_DIGICAM_CONTROL: { // #155
    fmav_digicam_control_t* payload = (fmav_digicam_control_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "session", payload->session);
    lua_pushtablenumber(L, "zoom_pos", payload->zoom_pos);
    lua_pushtablenumber(L, "zoom_step", payload->zoom_step);
    lua_pushtablenumber(L, "focus_lock", payload->focus_lock);
    lua_pushtablenumber(L, "shot", payload->shot);
    lua_pushtablenumber(L, "command_id", payload->command_id);
    lua_pushtablenumber(L, "extra_param", payload->extra_param);
    lua_pushtablenumber(L, "extra_value", payload->extra_value);
    return;
    }
  case FASTMAVLINK_MSG_ID_MOUNT_CONFIGURE: { // #156
    fmav_mount_configure_t* payload = (fmav_mount_configure_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "mount_mode", payload->mount_mode);
    lua_pushtablenumber(L, "stab_roll", payload->stab_roll);
    lua_pushtablenumber(L, "stab_pitch", payload->stab_pitch);
    lua_pushtablenumber(L, "stab_yaw", payload->stab_yaw);
    return;
    }
  case FASTMAVLINK_MSG_ID_MOUNT_CONTROL: { // #157
    fmav_mount_control_t* payload = (fmav_mount_control_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "input_a", payload->input_a);
    lua_pushtablenumber(L, "input_b", payload->input_b);
    lua_pushtablenumber(L, "input_c", payload->input_c);
    lua_pushtablenumber(L, "save_position", payload->save_position);
    return;
    }
  case FASTMAVLINK_MSG_ID_MOUNT_STATUS: { // #158
    fmav_mount_status_t* payload = (fmav_mount_status_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "pointing_a", payload->pointing_a);
    lua_pushtablenumber(L, "pointing_b", payload->pointing_b);
    lua_pushtablenumber(L, "pointing_c", payload->pointing_c);
    lua_pushtablenumber(L, "mount_mode", payload->mount_mode);
    return;
    }
  case FASTMAVLINK_MSG_ID_FENCE_POINT: { // #160
    fmav_fence_point_t* payload = (fmav_fence_point_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "idx", payload->idx);
    lua_pushtablenumber(L, "count", payload->count);
    lua_pushtablenumber(L, "lat", payload->lat);
    lua_pushtablenumber(L, "lng", payload->lng);
    return;
    }
  case FASTMAVLINK_MSG_ID_FENCE_FETCH_POINT: { // #161
    fmav_fence_fetch_point_t* payload = (fmav_fence_fetch_point_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "idx", payload->idx);
    return;
    }
  case FASTMAVLINK_MSG_ID_FENCE_STATUS: { // #162
    fmav_fence_status_t* payload = (fmav_fence_status_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "breach_status", payload->breach_status);
    lua_pushtablenumber(L, "breach_count", payload->breach_count);
    lua_pushtablenumber(L, "breach_type", payload->breach_type);
    lua_pushtablenumber(L, "breach_time", payload->breach_time);
    lua_pushtablenumber(L, "breach_mitigation", payload->breach_mitigation);
    return;
    }
  case FASTMAVLINK_MSG_ID_AHRS: { // #163
    fmav_ahrs_t* payload = (fmav_ahrs_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "omegaIx", payload->omegaIx);
    lua_pushtablenumber(L, "omegaIy", payload->omegaIy);
    lua_pushtablenumber(L, "omegaIz", payload->omegaIz);
    lua_pushtablenumber(L, "accel_weight", payload->accel_weight);
    lua_pushtablenumber(L, "renorm_val", payload->renorm_val);
    lua_pushtablenumber(L, "error_rp", payload->error_rp);
    lua_pushtablenumber(L, "error_yaw", payload->error_yaw);
    return;
    }
  case FASTMAVLINK_MSG_ID_SIMSTATE: { // #164
    fmav_simstate_t* payload = (fmav_simstate_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "roll", payload->roll);
    lua_pushtablenumber(L, "pitch", payload->pitch);
    lua_pushtablenumber(L, "yaw", payload->yaw);
    lua_pushtablenumber(L, "xacc", payload->xacc);
    lua_pushtablenumber(L, "yacc", payload->yacc);
    lua_pushtablenumber(L, "zacc", payload->zacc);
    lua_pushtablenumber(L, "xgyro", payload->xgyro);
    lua_pushtablenumber(L, "ygyro", payload->ygyro);
    lua_pushtablenumber(L, "zgyro", payload->zgyro);
    lua_pushtablenumber(L, "lat", payload->lat);
    lua_pushtablenumber(L, "lng", payload->lng);
    return;
    }
  case FASTMAVLINK_MSG_ID_HWSTATUS: { // #165
    fmav_hwstatus_t* payload = (fmav_hwstatus_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "Vcc", payload->Vcc);
    lua_pushtablenumber(L, "I2Cerr", payload->I2Cerr);
    return;
    }
  case FASTMAVLINK_MSG_ID_RADIO: { // #166
    fmav_radio_t* payload = (fmav_radio_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "rssi", payload->rssi);
    lua_pushtablenumber(L, "remrssi", payload->remrssi);
    lua_pushtablenumber(L, "txbuf", payload->txbuf);
    lua_pushtablenumber(L, "noise", payload->noise);
    lua_pushtablenumber(L, "remnoise", payload->remnoise);
    lua_pushtablenumber(L, "rxerrors", payload->rxerrors);
    lua_pushtablenumber(L, "fixed", payload->fixed);
    return;
    }
  case FASTMAVLINK_MSG_ID_LIMITS_STATUS: { // #167
    fmav_limits_status_t* payload = (fmav_limits_status_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "limits_state", payload->limits_state);
    lua_pushtablenumber(L, "last_trigger", payload->last_trigger);
    lua_pushtablenumber(L, "last_action", payload->last_action);
    lua_pushtablenumber(L, "last_recovery", payload->last_recovery);
    lua_pushtablenumber(L, "last_clear", payload->last_clear);
    lua_pushtablenumber(L, "breach_count", payload->breach_count);
    lua_pushtablenumber(L, "mods_enabled", payload->mods_enabled);
    lua_pushtablenumber(L, "mods_required", payload->mods_required);
    lua_pushtablenumber(L, "mods_triggered", payload->mods_triggered);
    return;
    }
  case FASTMAVLINK_MSG_ID_WIND: { // #168
    fmav_wind_t* payload = (fmav_wind_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "direction", payload->direction);
    lua_pushtablenumber(L, "speed", payload->speed);
    lua_pushtablenumber(L, "speed_z", payload->speed_z);
    return;
    }
  case FASTMAVLINK_MSG_ID_DATA16: { // #169
    fmav_data16_t* payload = (fmav_data16_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "type", payload->type);
    lua_pushtablenumber(L, "len", payload->len);
    lua_pushstring(L, "data"); // array data[16]
    lua_newtable(L);
    for (int i = 0; i < 16; i++) { 
      lua_pushtableinumber(L, i+1, payload->data[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    return;
    }
  case FASTMAVLINK_MSG_ID_DATA32: { // #170
    fmav_data32_t* payload = (fmav_data32_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "type", payload->type);
    lua_pushtablenumber(L, "len", payload->len);
    lua_pushstring(L, "data"); // array data[32]
    lua_newtable(L);
    for (int i = 0; i < 32; i++) { 
      lua_pushtableinumber(L, i+1, payload->data[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    return;
    }
  case FASTMAVLINK_MSG_ID_DATA64: { // #171
    fmav_data64_t* payload = (fmav_data64_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "type", payload->type);
    lua_pushtablenumber(L, "len", payload->len);
    lua_pushstring(L, "data"); // array data[64]
    lua_newtable(L);
    for (int i = 0; i < 64; i++) { 
      lua_pushtableinumber(L, i+1, payload->data[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    return;
    }
  case FASTMAVLINK_MSG_ID_DATA96: { // #172
    fmav_data96_t* payload = (fmav_data96_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "type", payload->type);
    lua_pushtablenumber(L, "len", payload->len);
    lua_pushstring(L, "data"); // array data[96]
    lua_newtable(L);
    for (int i = 0; i < 96; i++) { 
      lua_pushtableinumber(L, i+1, payload->data[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    return;
    }
  case FASTMAVLINK_MSG_ID_RANGEFINDER: { // #173
    fmav_rangefinder_t* payload = (fmav_rangefinder_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "distance", payload->distance);
    lua_pushtablenumber(L, "voltage", payload->voltage);
    return;
    }
  case FASTMAVLINK_MSG_ID_AIRSPEED_AUTOCAL: { // #174
    fmav_airspeed_autocal_t* payload = (fmav_airspeed_autocal_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "vx", payload->vx);
    lua_pushtablenumber(L, "vy", payload->vy);
    lua_pushtablenumber(L, "vz", payload->vz);
    lua_pushtablenumber(L, "diff_pressure", payload->diff_pressure);
    lua_pushtablenumber(L, "EAS2TAS", payload->EAS2TAS);
    lua_pushtablenumber(L, "ratio", payload->ratio);
    lua_pushtablenumber(L, "state_x", payload->state_x);
    lua_pushtablenumber(L, "state_y", payload->state_y);
    lua_pushtablenumber(L, "state_z", payload->state_z);
    lua_pushtablenumber(L, "Pax", payload->Pax);
    lua_pushtablenumber(L, "Pby", payload->Pby);
    lua_pushtablenumber(L, "Pcz", payload->Pcz);
    return;
    }
  case FASTMAVLINK_MSG_ID_RALLY_POINT: { // #175
    fmav_rally_point_t* payload = (fmav_rally_point_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "idx", payload->idx);
    lua_pushtablenumber(L, "count", payload->count);
    lua_pushtablenumber(L, "lat", payload->lat);
    lua_pushtablenumber(L, "lng", payload->lng);
    lua_pushtablenumber(L, "alt", payload->alt);
    lua_pushtablenumber(L, "break_alt", payload->break_alt);
    lua_pushtablenumber(L, "land_dir", payload->land_dir);
    lua_pushtablenumber(L, "flags", payload->flags);
    return;
    }
  case FASTMAVLINK_MSG_ID_RALLY_FETCH_POINT: { // #176
    fmav_rally_fetch_point_t* payload = (fmav_rally_fetch_point_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "idx", payload->idx);
    return;
    }
  case FASTMAVLINK_MSG_ID_COMPASSMOT_STATUS: { // #177
    fmav_compassmot_status_t* payload = (fmav_compassmot_status_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "throttle", payload->throttle);
    lua_pushtablenumber(L, "current", payload->current);
    lua_pushtablenumber(L, "interference", payload->interference);
    lua_pushtablenumber(L, "CompensationX", payload->CompensationX);
    lua_pushtablenumber(L, "CompensationY", payload->CompensationY);
    lua_pushtablenumber(L, "CompensationZ", payload->CompensationZ);
    return;
    }
  case FASTMAVLINK_MSG_ID_AHRS2: { // #178
    fmav_ahrs2_t* payload = (fmav_ahrs2_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "roll", payload->roll);
    lua_pushtablenumber(L, "pitch", payload->pitch);
    lua_pushtablenumber(L, "yaw", payload->yaw);
    lua_pushtablenumber(L, "altitude", payload->altitude);
    lua_pushtablenumber(L, "lat", payload->lat);
    lua_pushtablenumber(L, "lng", payload->lng);
    return;
    }
  case FASTMAVLINK_MSG_ID_CAMERA_STATUS: { // #179
    fmav_camera_status_t* payload = (fmav_camera_status_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "time_usec", payload->time_usec);
    lua_pushtablenumber(L, "cam_idx", payload->cam_idx);
    lua_pushtablenumber(L, "img_idx", payload->img_idx);
    lua_pushtablenumber(L, "event_id", payload->event_id);
    lua_pushtablenumber(L, "p1", payload->p1);
    lua_pushtablenumber(L, "p2", payload->p2);
    lua_pushtablenumber(L, "p3", payload->p3);
    lua_pushtablenumber(L, "p4", payload->p4);
    return;
    }
  case FASTMAVLINK_MSG_ID_CAMERA_FEEDBACK: { // #180
    fmav_camera_feedback_t* payload = (fmav_camera_feedback_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "time_usec", payload->time_usec);
    lua_pushtablenumber(L, "cam_idx", payload->cam_idx);
    lua_pushtablenumber(L, "img_idx", payload->img_idx);
    lua_pushtablenumber(L, "lat", payload->lat);
    lua_pushtablenumber(L, "lng", payload->lng);
    lua_pushtablenumber(L, "alt_msl", payload->alt_msl);
    lua_pushtablenumber(L, "alt_rel", payload->alt_rel);
    lua_pushtablenumber(L, "roll", payload->roll);
    lua_pushtablenumber(L, "pitch", payload->pitch);
    lua_pushtablenumber(L, "yaw", payload->yaw);
    lua_pushtablenumber(L, "foc_len", payload->foc_len);
    lua_pushtablenumber(L, "flags", payload->flags);
    lua_pushtablenumber(L, "completed_captures", payload->completed_captures);
    return;
    }
  case FASTMAVLINK_MSG_ID_BATTERY2: { // #181
    fmav_battery2_t* payload = (fmav_battery2_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "voltage", payload->voltage);
    lua_pushtablenumber(L, "current_battery", payload->current_battery);
    return;
    }
  case FASTMAVLINK_MSG_ID_AHRS3: { // #182
    fmav_ahrs3_t* payload = (fmav_ahrs3_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "roll", payload->roll);
    lua_pushtablenumber(L, "pitch", payload->pitch);
    lua_pushtablenumber(L, "yaw", payload->yaw);
    lua_pushtablenumber(L, "altitude", payload->altitude);
    lua_pushtablenumber(L, "lat", payload->lat);
    lua_pushtablenumber(L, "lng", payload->lng);
    lua_pushtablenumber(L, "v1", payload->v1);
    lua_pushtablenumber(L, "v2", payload->v2);
    lua_pushtablenumber(L, "v3", payload->v3);
    lua_pushtablenumber(L, "v4", payload->v4);
    return;
    }
  case FASTMAVLINK_MSG_ID_AUTOPILOT_VERSION_REQUEST: { // #183
    return;
    }
  case FASTMAVLINK_MSG_ID_REMOTE_LOG_DATA_BLOCK: { // #184
    fmav_remote_log_data_block_t* payload = (fmav_remote_log_data_block_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "seqno", payload->seqno);
    lua_pushstring(L, "data"); // array data[200]
    lua_newtable(L);
    for (int i = 0; i < 200; i++) { 
      lua_pushtableinumber(L, i+1, payload->data[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    return;
    }
  case FASTMAVLINK_MSG_ID_REMOTE_LOG_BLOCK_STATUS: { // #185
    fmav_remote_log_block_status_t* payload = (fmav_remote_log_block_status_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "seqno", payload->seqno);
    lua_pushtablenumber(L, "status", payload->status);
    return;
    }
  case FASTMAVLINK_MSG_ID_LED_CONTROL: { // #186
    fmav_led_control_t* payload = (fmav_led_control_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "instance", payload->instance);
    lua_pushtablenumber(L, "pattern", payload->pattern);
    lua_pushtablenumber(L, "custom_len", payload->custom_len);
    lua_pushstring(L, "custom_bytes"); // array custom_bytes[24]
    lua_newtable(L);
    for (int i = 0; i < 24; i++) { 
      lua_pushtableinumber(L, i+1, payload->custom_bytes[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    return;
    }
  case FASTMAVLINK_MSG_ID_MAG_CAL_PROGRESS: { // #191
    fmav_mag_cal_progress_t* payload = (fmav_mag_cal_progress_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "compass_id", payload->compass_id);
    lua_pushtablenumber(L, "cal_mask", payload->cal_mask);
    lua_pushtablenumber(L, "cal_status", payload->cal_status);
    lua_pushtablenumber(L, "attempt", payload->attempt);
    lua_pushtablenumber(L, "completion_pct", payload->completion_pct);
    lua_pushstring(L, "completion_mask"); // array completion_mask[10]
    lua_newtable(L);
    for (int i = 0; i < 10; i++) { 
      lua_pushtableinumber(L, i+1, payload->completion_mask[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushtablenumber(L, "direction_x", payload->direction_x);
    lua_pushtablenumber(L, "direction_y", payload->direction_y);
    lua_pushtablenumber(L, "direction_z", payload->direction_z);
    return;
    }
  case FASTMAVLINK_MSG_ID_MAG_CAL_REPORT: { // #192
    fmav_mag_cal_report_t* payload = (fmav_mag_cal_report_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "compass_id", payload->compass_id);
    lua_pushtablenumber(L, "cal_mask", payload->cal_mask);
    lua_pushtablenumber(L, "cal_status", payload->cal_status);
    lua_pushtablenumber(L, "autosaved", payload->autosaved);
    lua_pushtablenumber(L, "fitness", payload->fitness);
    lua_pushtablenumber(L, "ofs_x", payload->ofs_x);
    lua_pushtablenumber(L, "ofs_y", payload->ofs_y);
    lua_pushtablenumber(L, "ofs_z", payload->ofs_z);
    lua_pushtablenumber(L, "diag_x", payload->diag_x);
    lua_pushtablenumber(L, "diag_y", payload->diag_y);
    lua_pushtablenumber(L, "diag_z", payload->diag_z);
    lua_pushtablenumber(L, "offdiag_x", payload->offdiag_x);
    lua_pushtablenumber(L, "offdiag_y", payload->offdiag_y);
    lua_pushtablenumber(L, "offdiag_z", payload->offdiag_z);
    lua_pushtablenumber(L, "orientation_confidence", payload->orientation_confidence);
    lua_pushtablenumber(L, "old_orientation", payload->old_orientation);
    lua_pushtablenumber(L, "new_orientation", payload->new_orientation);
    lua_pushtablenumber(L, "scale_factor", payload->scale_factor);
    return;
    }
  case FASTMAVLINK_MSG_ID_EKF_STATUS_REPORT: { // #193
    fmav_ekf_status_report_t* payload = (fmav_ekf_status_report_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "flags", payload->flags);
    lua_pushtablenumber(L, "velocity_variance", payload->velocity_variance);
    lua_pushtablenumber(L, "pos_horiz_variance", payload->pos_horiz_variance);
    lua_pushtablenumber(L, "pos_vert_variance", payload->pos_vert_variance);
    lua_pushtablenumber(L, "compass_variance", payload->compass_variance);
    lua_pushtablenumber(L, "terrain_alt_variance", payload->terrain_alt_variance);
    lua_pushtablenumber(L, "airspeed_variance", payload->airspeed_variance);
    return;
    }
  case FASTMAVLINK_MSG_ID_PID_TUNING: { // #194
    fmav_pid_tuning_t* payload = (fmav_pid_tuning_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "axis", payload->axis);
    lua_pushtablenumber(L, "desired", payload->desired);
    lua_pushtablenumber(L, "achieved", payload->achieved);
    lua_pushtablenumber(L, "FF", payload->FF);
    lua_pushtablenumber(L, "P", payload->P);
    lua_pushtablenumber(L, "I", payload->I);
    lua_pushtablenumber(L, "D", payload->D);
    lua_pushtablenumber(L, "SRate", payload->SRate);
    lua_pushtablenumber(L, "PDmod", payload->PDmod);
    return;
    }
  case FASTMAVLINK_MSG_ID_DEEPSTALL: { // #195
    fmav_deepstall_t* payload = (fmav_deepstall_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "landing_lat", payload->landing_lat);
    lua_pushtablenumber(L, "landing_lon", payload->landing_lon);
    lua_pushtablenumber(L, "path_lat", payload->path_lat);
    lua_pushtablenumber(L, "path_lon", payload->path_lon);
    lua_pushtablenumber(L, "arc_entry_lat", payload->arc_entry_lat);
    lua_pushtablenumber(L, "arc_entry_lon", payload->arc_entry_lon);
    lua_pushtablenumber(L, "altitude", payload->altitude);
    lua_pushtablenumber(L, "expected_travel_distance", payload->expected_travel_distance);
    lua_pushtablenumber(L, "cross_track_error", payload->cross_track_error);
    lua_pushtablenumber(L, "stage", payload->stage);
    return;
    }
  case FASTMAVLINK_MSG_ID_GIMBAL_REPORT: { // #200
    fmav_gimbal_report_t* payload = (fmav_gimbal_report_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "delta_time", payload->delta_time);
    lua_pushtablenumber(L, "delta_angle_x", payload->delta_angle_x);
    lua_pushtablenumber(L, "delta_angle_y", payload->delta_angle_y);
    lua_pushtablenumber(L, "delta_angle_z", payload->delta_angle_z);
    lua_pushtablenumber(L, "delta_velocity_x", payload->delta_velocity_x);
    lua_pushtablenumber(L, "delta_velocity_y", payload->delta_velocity_y);
    lua_pushtablenumber(L, "delta_velocity_z", payload->delta_velocity_z);
    lua_pushtablenumber(L, "joint_roll", payload->joint_roll);
    lua_pushtablenumber(L, "joint_el", payload->joint_el);
    lua_pushtablenumber(L, "joint_az", payload->joint_az);
    return;
    }
  case FASTMAVLINK_MSG_ID_GIMBAL_CONTROL: { // #201
    fmav_gimbal_control_t* payload = (fmav_gimbal_control_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "demanded_rate_x", payload->demanded_rate_x);
    lua_pushtablenumber(L, "demanded_rate_y", payload->demanded_rate_y);
    lua_pushtablenumber(L, "demanded_rate_z", payload->demanded_rate_z);
    return;
    }
  case FASTMAVLINK_MSG_ID_GIMBAL_TORQUE_CMD_REPORT: { // #214
    fmav_gimbal_torque_cmd_report_t* payload = (fmav_gimbal_torque_cmd_report_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "rl_torque_cmd", payload->rl_torque_cmd);
    lua_pushtablenumber(L, "el_torque_cmd", payload->el_torque_cmd);
    lua_pushtablenumber(L, "az_torque_cmd", payload->az_torque_cmd);
    return;
    }
  case FASTMAVLINK_MSG_ID_GOPRO_HEARTBEAT: { // #215
    fmav_gopro_heartbeat_t* payload = (fmav_gopro_heartbeat_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "status", payload->status);
    lua_pushtablenumber(L, "capture_mode", payload->capture_mode);
    lua_pushtablenumber(L, "flags", payload->flags);
    return;
    }
  case FASTMAVLINK_MSG_ID_GOPRO_GET_REQUEST: { // #216
    fmav_gopro_get_request_t* payload = (fmav_gopro_get_request_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "cmd_id", payload->cmd_id);
    return;
    }
  case FASTMAVLINK_MSG_ID_GOPRO_GET_RESPONSE: { // #217
    fmav_gopro_get_response_t* payload = (fmav_gopro_get_response_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "cmd_id", payload->cmd_id);
    lua_pushtablenumber(L, "status", payload->status);
    lua_pushstring(L, "value"); // array value[4]
    lua_newtable(L);
    for (int i = 0; i < 4; i++) { 
      lua_pushtableinumber(L, i+1, payload->value[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    return;
    }
  case FASTMAVLINK_MSG_ID_GOPRO_SET_REQUEST: { // #218
    fmav_gopro_set_request_t* payload = (fmav_gopro_set_request_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "cmd_id", payload->cmd_id);
    lua_pushstring(L, "value"); // array value[4]
    lua_newtable(L);
    for (int i = 0; i < 4; i++) { 
      lua_pushtableinumber(L, i+1, payload->value[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    return;
    }
  case FASTMAVLINK_MSG_ID_GOPRO_SET_RESPONSE: { // #219
    fmav_gopro_set_response_t* payload = (fmav_gopro_set_response_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "cmd_id", payload->cmd_id);
    lua_pushtablenumber(L, "status", payload->status);
    return;
    }
  case FASTMAVLINK_MSG_ID_EFI_STATUS: { // #225
    fmav_efi_status_t* payload = (fmav_efi_status_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "health", payload->health);
    lua_pushtablenumber(L, "ecu_index", payload->ecu_index);
    lua_pushtablenumber(L, "rpm", payload->rpm);
    lua_pushtablenumber(L, "fuel_consumed", payload->fuel_consumed);
    lua_pushtablenumber(L, "fuel_flow", payload->fuel_flow);
    lua_pushtablenumber(L, "engine_load", payload->engine_load);
    lua_pushtablenumber(L, "throttle_position", payload->throttle_position);
    lua_pushtablenumber(L, "spark_dwell_time", payload->spark_dwell_time);
    lua_pushtablenumber(L, "barometric_pressure", payload->barometric_pressure);
    lua_pushtablenumber(L, "intake_manifold_pressure", payload->intake_manifold_pressure);
    lua_pushtablenumber(L, "intake_manifold_temperature", payload->intake_manifold_temperature);
    lua_pushtablenumber(L, "cylinder_head_temperature", payload->cylinder_head_temperature);
    lua_pushtablenumber(L, "ignition_timing", payload->ignition_timing);
    lua_pushtablenumber(L, "injection_time", payload->injection_time);
    lua_pushtablenumber(L, "exhaust_gas_temperature", payload->exhaust_gas_temperature);
    lua_pushtablenumber(L, "throttle_out", payload->throttle_out);
    lua_pushtablenumber(L, "pt_compensation", payload->pt_compensation);
    lua_pushtablenumber(L, "ignition_voltage", payload->ignition_voltage);
    lua_pushtablenumber(L, "fuel_pressure", payload->fuel_pressure);
    return;
    }
  case FASTMAVLINK_MSG_ID_RPM: { // #226
    fmav_rpm_t* payload = (fmav_rpm_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "rpm1", payload->rpm1);
    lua_pushtablenumber(L, "rpm2", payload->rpm2);
    return;
    }
  case FASTMAVLINK_MSG_ID_ESTIMATOR_STATUS: { // #230
    fmav_estimator_status_t* payload = (fmav_estimator_status_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "time_usec", payload->time_usec);
    lua_pushtablenumber(L, "flags", payload->flags);
    lua_pushtablenumber(L, "vel_ratio", payload->vel_ratio);
    lua_pushtablenumber(L, "pos_horiz_ratio", payload->pos_horiz_ratio);
    lua_pushtablenumber(L, "pos_vert_ratio", payload->pos_vert_ratio);
    lua_pushtablenumber(L, "mag_ratio", payload->mag_ratio);
    lua_pushtablenumber(L, "hagl_ratio", payload->hagl_ratio);
    lua_pushtablenumber(L, "tas_ratio", payload->tas_ratio);
    lua_pushtablenumber(L, "pos_horiz_accuracy", payload->pos_horiz_accuracy);
    lua_pushtablenumber(L, "pos_vert_accuracy", payload->pos_vert_accuracy);
    return;
    }
  case FASTMAVLINK_MSG_ID_WIND_COV: { // #231
    fmav_wind_cov_t* payload = (fmav_wind_cov_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "time_usec", payload->time_usec);
    lua_pushtablenumber(L, "wind_x", payload->wind_x);
    lua_pushtablenumber(L, "wind_y", payload->wind_y);
    lua_pushtablenumber(L, "wind_z", payload->wind_z);
    lua_pushtablenumber(L, "var_horiz", payload->var_horiz);
    lua_pushtablenumber(L, "var_vert", payload->var_vert);
    lua_pushtablenumber(L, "wind_alt", payload->wind_alt);
    lua_pushtablenumber(L, "horiz_accuracy", payload->horiz_accuracy);
    lua_pushtablenumber(L, "vert_accuracy", payload->vert_accuracy);
    return;
    }
  case FASTMAVLINK_MSG_ID_GPS_INPUT: { // #232
    fmav_gps_input_t* payload = (fmav_gps_input_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "time_usec", payload->time_usec);
    lua_pushtablenumber(L, "gps_id", payload->gps_id);
    lua_pushtablenumber(L, "ignore_flags", payload->ignore_flags);
    lua_pushtablenumber(L, "time_week_ms", payload->time_week_ms);
    lua_pushtablenumber(L, "time_week", payload->time_week);
    lua_pushtablenumber(L, "fix_type", payload->fix_type);
    lua_pushtablenumber(L, "lat", payload->lat);
    lua_pushtablenumber(L, "lon", payload->lon);
    lua_pushtablenumber(L, "alt", payload->alt);
    lua_pushtablenumber(L, "hdop", payload->hdop);
    lua_pushtablenumber(L, "vdop", payload->vdop);
    lua_pushtablenumber(L, "vn", payload->vn);
    lua_pushtablenumber(L, "ve", payload->ve);
    lua_pushtablenumber(L, "vd", payload->vd);
    lua_pushtablenumber(L, "speed_accuracy", payload->speed_accuracy);
    lua_pushtablenumber(L, "horiz_accuracy", payload->horiz_accuracy);
    lua_pushtablenumber(L, "vert_accuracy", payload->vert_accuracy);
    lua_pushtablenumber(L, "satellites_visible", payload->satellites_visible);
    lua_pushtablenumber(L, "yaw", payload->yaw);
    return;
    }
  case FASTMAVLINK_MSG_ID_GPS_RTCM_DATA: { // #233
    fmav_gps_rtcm_data_t* payload = (fmav_gps_rtcm_data_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "flags", payload->flags);
    lua_pushtablenumber(L, "len", payload->len);
    lua_pushstring(L, "data"); // array data[180]
    lua_newtable(L);
    for (int i = 0; i < 180; i++) { 
      lua_pushtableinumber(L, i+1, payload->data[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    return;
    }
  case FASTMAVLINK_MSG_ID_HIGH_LATENCY: { // #234
    fmav_high_latency_t* payload = (fmav_high_latency_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "base_mode", payload->base_mode);
    lua_pushtablenumber(L, "custom_mode", payload->custom_mode);
    lua_pushtablenumber(L, "landed_state", payload->landed_state);
    lua_pushtablenumber(L, "roll", payload->roll);
    lua_pushtablenumber(L, "pitch", payload->pitch);
    lua_pushtablenumber(L, "heading", payload->heading);
    lua_pushtablenumber(L, "throttle", payload->throttle);
    lua_pushtablenumber(L, "heading_sp", payload->heading_sp);
    lua_pushtablenumber(L, "latitude", payload->latitude);
    lua_pushtablenumber(L, "longitude", payload->longitude);
    lua_pushtablenumber(L, "altitude_amsl", payload->altitude_amsl);
    lua_pushtablenumber(L, "altitude_sp", payload->altitude_sp);
    lua_pushtablenumber(L, "airspeed", payload->airspeed);
    lua_pushtablenumber(L, "airspeed_sp", payload->airspeed_sp);
    lua_pushtablenumber(L, "groundspeed", payload->groundspeed);
    lua_pushtablenumber(L, "climb_rate", payload->climb_rate);
    lua_pushtablenumber(L, "gps_nsat", payload->gps_nsat);
    lua_pushtablenumber(L, "gps_fix_type", payload->gps_fix_type);
    lua_pushtablenumber(L, "battery_remaining", payload->battery_remaining);
    lua_pushtablenumber(L, "temperature", payload->temperature);
    lua_pushtablenumber(L, "temperature_air", payload->temperature_air);
    lua_pushtablenumber(L, "failsafe", payload->failsafe);
    lua_pushtablenumber(L, "wp_num", payload->wp_num);
    lua_pushtablenumber(L, "wp_distance", payload->wp_distance);
    return;
    }
  case FASTMAVLINK_MSG_ID_HIGH_LATENCY2: { // #235
    fmav_high_latency2_t* payload = (fmav_high_latency2_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "timestamp", payload->timestamp);
    lua_pushtablenumber(L, "type", payload->type);
    lua_pushtablenumber(L, "autopilot", payload->autopilot);
    lua_pushtablenumber(L, "custom_mode", payload->custom_mode);
    lua_pushtablenumber(L, "latitude", payload->latitude);
    lua_pushtablenumber(L, "longitude", payload->longitude);
    lua_pushtablenumber(L, "altitude", payload->altitude);
    lua_pushtablenumber(L, "target_altitude", payload->target_altitude);
    lua_pushtablenumber(L, "heading", payload->heading);
    lua_pushtablenumber(L, "target_heading", payload->target_heading);
    lua_pushtablenumber(L, "target_distance", payload->target_distance);
    lua_pushtablenumber(L, "throttle", payload->throttle);
    lua_pushtablenumber(L, "airspeed", payload->airspeed);
    lua_pushtablenumber(L, "airspeed_sp", payload->airspeed_sp);
    lua_pushtablenumber(L, "groundspeed", payload->groundspeed);
    lua_pushtablenumber(L, "windspeed", payload->windspeed);
    lua_pushtablenumber(L, "wind_heading", payload->wind_heading);
    lua_pushtablenumber(L, "eph", payload->eph);
    lua_pushtablenumber(L, "epv", payload->epv);
    lua_pushtablenumber(L, "temperature_air", payload->temperature_air);
    lua_pushtablenumber(L, "climb_rate", payload->climb_rate);
    lua_pushtablenumber(L, "battery", payload->battery);
    lua_pushtablenumber(L, "wp_num", payload->wp_num);
    lua_pushtablenumber(L, "failure_flags", payload->failure_flags);
    lua_pushtablenumber(L, "custom0", payload->custom0);
    lua_pushtablenumber(L, "custom1", payload->custom1);
    lua_pushtablenumber(L, "custom2", payload->custom2);
    return;
    }
  case FASTMAVLINK_MSG_ID_VIBRATION: { // #241
    fmav_vibration_t* payload = (fmav_vibration_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "time_usec", payload->time_usec);
    lua_pushtablenumber(L, "vibration_x", payload->vibration_x);
    lua_pushtablenumber(L, "vibration_y", payload->vibration_y);
    lua_pushtablenumber(L, "vibration_z", payload->vibration_z);
    lua_pushtablenumber(L, "clipping_0", payload->clipping_0);
    lua_pushtablenumber(L, "clipping_1", payload->clipping_1);
    lua_pushtablenumber(L, "clipping_2", payload->clipping_2);
    return;
    }
  case FASTMAVLINK_MSG_ID_HOME_POSITION: { // #242
    fmav_home_position_t* payload = (fmav_home_position_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "latitude", payload->latitude);
    lua_pushtablenumber(L, "longitude", payload->longitude);
    lua_pushtablenumber(L, "altitude", payload->altitude);
    lua_pushtablenumber(L, "x", payload->x);
    lua_pushtablenumber(L, "y", payload->y);
    lua_pushtablenumber(L, "z", payload->z);
    lua_pushstring(L, "q"); // array q[4]
    lua_newtable(L);
    for (int i = 0; i < 4; i++) { 
      lua_pushtableinumber(L, i+1, payload->q[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushtablenumber(L, "approach_x", payload->approach_x);
    lua_pushtablenumber(L, "approach_y", payload->approach_y);
    lua_pushtablenumber(L, "approach_z", payload->approach_z);
    lua_pushtablenumber(L, "time_usec", payload->time_usec);
    return;
    }
  case FASTMAVLINK_MSG_ID_SET_HOME_POSITION: { // #243
    fmav_set_home_position_t* payload = (fmav_set_home_position_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "latitude", payload->latitude);
    lua_pushtablenumber(L, "longitude", payload->longitude);
    lua_pushtablenumber(L, "altitude", payload->altitude);
    lua_pushtablenumber(L, "x", payload->x);
    lua_pushtablenumber(L, "y", payload->y);
    lua_pushtablenumber(L, "z", payload->z);
    lua_pushstring(L, "q"); // array q[4]
    lua_newtable(L);
    for (int i = 0; i < 4; i++) { 
      lua_pushtableinumber(L, i+1, payload->q[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushtablenumber(L, "approach_x", payload->approach_x);
    lua_pushtablenumber(L, "approach_y", payload->approach_y);
    lua_pushtablenumber(L, "approach_z", payload->approach_z);
    lua_pushtablenumber(L, "time_usec", payload->time_usec);
    return;
    }
  case FASTMAVLINK_MSG_ID_MESSAGE_INTERVAL: { // #244
    fmav_message_interval_t* payload = (fmav_message_interval_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "message_id", payload->message_id);
    lua_pushtablenumber(L, "interval_us", payload->interval_us);
    return;
    }
  case FASTMAVLINK_MSG_ID_EXTENDED_SYS_STATE: { // #245
    fmav_extended_sys_state_t* payload = (fmav_extended_sys_state_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "vtol_state", payload->vtol_state);
    lua_pushtablenumber(L, "landed_state", payload->landed_state);
    return;
    }
  case FASTMAVLINK_MSG_ID_ADSB_VEHICLE: { // #246
    fmav_adsb_vehicle_t* payload = (fmav_adsb_vehicle_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "ICAO_address", payload->ICAO_address);
    lua_pushtablenumber(L, "lat", payload->lat);
    lua_pushtablenumber(L, "lon", payload->lon);
    lua_pushtablenumber(L, "altitude_type", payload->altitude_type);
    lua_pushtablenumber(L, "altitude", payload->altitude);
    lua_pushtablenumber(L, "heading", payload->heading);
    lua_pushtablenumber(L, "hor_velocity", payload->hor_velocity);
    lua_pushtablenumber(L, "ver_velocity", payload->ver_velocity);
    lua_pushstring(L, "callsign"); // array callsign[9]
    lua_newtable(L);
    for (int i = 0; i < 9; i++) { 
      lua_pushtableinumber(L, i+1, payload->callsign[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushtablenumber(L, "emitter_type", payload->emitter_type);
    lua_pushtablenumber(L, "tslc", payload->tslc);
    lua_pushtablenumber(L, "flags", payload->flags);
    lua_pushtablenumber(L, "squawk", payload->squawk);
    return;
    }
  case FASTMAVLINK_MSG_ID_COLLISION: { // #247
    fmav_collision_t* payload = (fmav_collision_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "src", payload->src);
    lua_pushtablenumber(L, "id", payload->id);
    lua_pushtablenumber(L, "action", payload->action);
    lua_pushtablenumber(L, "threat_level", payload->threat_level);
    lua_pushtablenumber(L, "time_to_minimum_delta", payload->time_to_minimum_delta);
    lua_pushtablenumber(L, "altitude_minimum_delta", payload->altitude_minimum_delta);
    lua_pushtablenumber(L, "horizontal_minimum_delta", payload->horizontal_minimum_delta);
    return;
    }
  case FASTMAVLINK_MSG_ID_V2_EXTENSION: { // #248
    fmav_v2_extension_t* payload = (fmav_v2_extension_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "target_network", payload->target_network);
    lua_pushtablenumber(L, "message_type", payload->message_type);
    lua_pushstring(L, "payload"); // array payload[249]
    lua_newtable(L);
    for (int i = 0; i < 249; i++) { 
      lua_pushtableinumber(L, i+1, payload->payload[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    return;
    }
  case FASTMAVLINK_MSG_ID_MEMORY_VECT: { // #249
    fmav_memory_vect_t* payload = (fmav_memory_vect_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "address", payload->address);
    lua_pushtablenumber(L, "ver", payload->ver);
    lua_pushtablenumber(L, "type", payload->type);
    lua_pushstring(L, "value"); // array value[32]
    lua_newtable(L);
    for (int i = 0; i < 32; i++) { 
      lua_pushtableinumber(L, i+1, payload->value[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    return;
    }
  case FASTMAVLINK_MSG_ID_DEBUG_VECT: { // #250
    fmav_debug_vect_t* payload = (fmav_debug_vect_t*)(mavmsg->payload_ptr);
    lua_pushstring(L, "name"); // array name[10]
    lua_newtable(L);
    for (int i = 0; i < 10; i++) { 
      lua_pushtableinumber(L, i+1, payload->name[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushtablenumber(L, "time_usec", payload->time_usec);
    lua_pushtablenumber(L, "x", payload->x);
    lua_pushtablenumber(L, "y", payload->y);
    lua_pushtablenumber(L, "z", payload->z);
    return;
    }
  case FASTMAVLINK_MSG_ID_NAMED_VALUE_FLOAT: { // #251
    fmav_named_value_float_t* payload = (fmav_named_value_float_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "time_boot_ms", payload->time_boot_ms);
    lua_pushstring(L, "name"); // array name[10]
    lua_newtable(L);
    for (int i = 0; i < 10; i++) { 
      lua_pushtableinumber(L, i+1, payload->name[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushtablenumber(L, "value", payload->value);
    return;
    }
  case FASTMAVLINK_MSG_ID_NAMED_VALUE_INT: { // #252
    fmav_named_value_int_t* payload = (fmav_named_value_int_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "time_boot_ms", payload->time_boot_ms);
    lua_pushstring(L, "name"); // array name[10]
    lua_newtable(L);
    for (int i = 0; i < 10; i++) { 
      lua_pushtableinumber(L, i+1, payload->name[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushtablenumber(L, "value", payload->value);
    return;
    }
  case FASTMAVLINK_MSG_ID_STATUSTEXT: { // #253
    fmav_statustext_t* payload = (fmav_statustext_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "severity", payload->severity);
    lua_pushstring(L, "text"); // array text[50]
    lua_newtable(L);
    for (int i = 0; i < 50; i++) { 
      lua_pushtableinumber(L, i+1, payload->text[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushtablenumber(L, "id", payload->id);
    lua_pushtablenumber(L, "chunk_seq", payload->chunk_seq);
    return;
    }
  case FASTMAVLINK_MSG_ID_DEBUG: { // #254
    fmav_debug_t* payload = (fmav_debug_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "time_boot_ms", payload->time_boot_ms);
    lua_pushtablenumber(L, "ind", payload->ind);
    lua_pushtablenumber(L, "value", payload->value);
    return;
    }
  case FASTMAVLINK_MSG_ID_SETUP_SIGNING: { // #256
    fmav_setup_signing_t* payload = (fmav_setup_signing_t*)(mavmsg->payload_ptr);
    lua_pushstring(L, "secret_key"); // array secret_key[32]
    lua_newtable(L);
    for (int i = 0; i < 32; i++) { 
      lua_pushtableinumber(L, i+1, payload->secret_key[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushtablenumber(L, "initial_timestamp", payload->initial_timestamp);
    return;
    }
  case FASTMAVLINK_MSG_ID_BUTTON_CHANGE: { // #257
    fmav_button_change_t* payload = (fmav_button_change_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "time_boot_ms", payload->time_boot_ms);
    lua_pushtablenumber(L, "last_change_ms", payload->last_change_ms);
    lua_pushtablenumber(L, "state", payload->state);
    return;
    }
  case FASTMAVLINK_MSG_ID_PLAY_TUNE: { // #258
    fmav_play_tune_t* payload = (fmav_play_tune_t*)(mavmsg->payload_ptr);
    lua_pushstring(L, "tune"); // array tune[30]
    lua_newtable(L);
    for (int i = 0; i < 30; i++) { 
      lua_pushtableinumber(L, i+1, payload->tune[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushstring(L, "tune2"); // array tune2[200]
    lua_newtable(L);
    for (int i = 0; i < 200; i++) { 
      lua_pushtableinumber(L, i+1, payload->tune2[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    return;
    }
  case FASTMAVLINK_MSG_ID_CAMERA_INFORMATION: { // #259
    fmav_camera_information_t* payload = (fmav_camera_information_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "time_boot_ms", payload->time_boot_ms);
    lua_pushstring(L, "vendor_name"); // array vendor_name[32]
    lua_newtable(L);
    for (int i = 0; i < 32; i++) { 
      lua_pushtableinumber(L, i+1, payload->vendor_name[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushstring(L, "model_name"); // array model_name[32]
    lua_newtable(L);
    for (int i = 0; i < 32; i++) { 
      lua_pushtableinumber(L, i+1, payload->model_name[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushtablenumber(L, "firmware_version", payload->firmware_version);
    lua_pushtablenumber(L, "focal_length", payload->focal_length);
    lua_pushtablenumber(L, "sensor_size_h", payload->sensor_size_h);
    lua_pushtablenumber(L, "sensor_size_v", payload->sensor_size_v);
    lua_pushtablenumber(L, "resolution_h", payload->resolution_h);
    lua_pushtablenumber(L, "resolution_v", payload->resolution_v);
    lua_pushtablenumber(L, "lens_id", payload->lens_id);
    lua_pushtablenumber(L, "flags", payload->flags);
    lua_pushtablenumber(L, "cam_definition_version", payload->cam_definition_version);
    lua_pushstring(L, "cam_definition_uri"); // array cam_definition_uri[140]
    lua_newtable(L);
    for (int i = 0; i < 140; i++) { 
      lua_pushtableinumber(L, i+1, payload->cam_definition_uri[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushtablenumber(L, "gimbal_device_id", payload->gimbal_device_id);
    return;
    }
  case FASTMAVLINK_MSG_ID_CAMERA_SETTINGS: { // #260
    fmav_camera_settings_t* payload = (fmav_camera_settings_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "time_boot_ms", payload->time_boot_ms);
    lua_pushtablenumber(L, "mode_id", payload->mode_id);
    lua_pushtablenumber(L, "zoomLevel", payload->zoomLevel);
    lua_pushtablenumber(L, "focusLevel", payload->focusLevel);
    return;
    }
  case FASTMAVLINK_MSG_ID_STORAGE_INFORMATION: { // #261
    fmav_storage_information_t* payload = (fmav_storage_information_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "time_boot_ms", payload->time_boot_ms);
    lua_pushtablenumber(L, "storage_id", payload->storage_id);
    lua_pushtablenumber(L, "storage_count", payload->storage_count);
    lua_pushtablenumber(L, "status", payload->status);
    lua_pushtablenumber(L, "total_capacity", payload->total_capacity);
    lua_pushtablenumber(L, "used_capacity", payload->used_capacity);
    lua_pushtablenumber(L, "available_capacity", payload->available_capacity);
    lua_pushtablenumber(L, "read_speed", payload->read_speed);
    lua_pushtablenumber(L, "write_speed", payload->write_speed);
    lua_pushtablenumber(L, "type", payload->type);
    lua_pushstring(L, "name"); // array name[32]
    lua_newtable(L);
    for (int i = 0; i < 32; i++) { 
      lua_pushtableinumber(L, i+1, payload->name[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushtablenumber(L, "storage_usage", payload->storage_usage);
    return;
    }
  case FASTMAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS: { // #262
    fmav_camera_capture_status_t* payload = (fmav_camera_capture_status_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "time_boot_ms", payload->time_boot_ms);
    lua_pushtablenumber(L, "image_status", payload->image_status);
    lua_pushtablenumber(L, "video_status", payload->video_status);
    lua_pushtablenumber(L, "image_interval", payload->image_interval);
    lua_pushtablenumber(L, "recording_time_ms", payload->recording_time_ms);
    lua_pushtablenumber(L, "available_capacity", payload->available_capacity);
    lua_pushtablenumber(L, "image_count", payload->image_count);
    return;
    }
  case FASTMAVLINK_MSG_ID_CAMERA_IMAGE_CAPTURED: { // #263
    fmav_camera_image_captured_t* payload = (fmav_camera_image_captured_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "time_boot_ms", payload->time_boot_ms);
    lua_pushtablenumber(L, "time_utc", payload->time_utc);
    lua_pushtablenumber(L, "camera_id", payload->camera_id);
    lua_pushtablenumber(L, "lat", payload->lat);
    lua_pushtablenumber(L, "lon", payload->lon);
    lua_pushtablenumber(L, "alt", payload->alt);
    lua_pushtablenumber(L, "relative_alt", payload->relative_alt);
    lua_pushstring(L, "q"); // array q[4]
    lua_newtable(L);
    for (int i = 0; i < 4; i++) { 
      lua_pushtableinumber(L, i+1, payload->q[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushtablenumber(L, "image_index", payload->image_index);
    lua_pushtablenumber(L, "capture_result", payload->capture_result);
    lua_pushstring(L, "file_url"); // array file_url[205]
    lua_newtable(L);
    for (int i = 0; i < 205; i++) { 
      lua_pushtableinumber(L, i+1, payload->file_url[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    return;
    }
  case FASTMAVLINK_MSG_ID_FLIGHT_INFORMATION: { // #264
    fmav_flight_information_t* payload = (fmav_flight_information_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "time_boot_ms", payload->time_boot_ms);
    lua_pushtablenumber(L, "arming_time_utc", payload->arming_time_utc);
    lua_pushtablenumber(L, "takeoff_time_utc", payload->takeoff_time_utc);
    lua_pushtablenumber(L, "flight_uuid", payload->flight_uuid);
    return;
    }
  case FASTMAVLINK_MSG_ID_MOUNT_ORIENTATION: { // #265
    fmav_mount_orientation_t* payload = (fmav_mount_orientation_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "time_boot_ms", payload->time_boot_ms);
    lua_pushtablenumber(L, "roll", payload->roll);
    lua_pushtablenumber(L, "pitch", payload->pitch);
    lua_pushtablenumber(L, "yaw", payload->yaw);
    lua_pushtablenumber(L, "yaw_absolute", payload->yaw_absolute);
    return;
    }
  case FASTMAVLINK_MSG_ID_LOGGING_DATA: { // #266
    fmav_logging_data_t* payload = (fmav_logging_data_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "sequence", payload->sequence);
    lua_pushtablenumber(L, "length", payload->length);
    lua_pushtablenumber(L, "first_message_offset", payload->first_message_offset);
    lua_pushstring(L, "data"); // array data[249]
    lua_newtable(L);
    for (int i = 0; i < 249; i++) { 
      lua_pushtableinumber(L, i+1, payload->data[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    return;
    }
  case FASTMAVLINK_MSG_ID_LOGGING_DATA_ACKED: { // #267
    fmav_logging_data_acked_t* payload = (fmav_logging_data_acked_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "sequence", payload->sequence);
    lua_pushtablenumber(L, "length", payload->length);
    lua_pushtablenumber(L, "first_message_offset", payload->first_message_offset);
    lua_pushstring(L, "data"); // array data[249]
    lua_newtable(L);
    for (int i = 0; i < 249; i++) { 
      lua_pushtableinumber(L, i+1, payload->data[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    return;
    }
  case FASTMAVLINK_MSG_ID_LOGGING_ACK: { // #268
    fmav_logging_ack_t* payload = (fmav_logging_ack_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "sequence", payload->sequence);
    return;
    }
  case FASTMAVLINK_MSG_ID_VIDEO_STREAM_INFORMATION: { // #269
    fmav_video_stream_information_t* payload = (fmav_video_stream_information_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "stream_id", payload->stream_id);
    lua_pushtablenumber(L, "count", payload->count);
    lua_pushtablenumber(L, "type", payload->type);
    lua_pushtablenumber(L, "flags", payload->flags);
    lua_pushtablenumber(L, "framerate", payload->framerate);
    lua_pushtablenumber(L, "resolution_h", payload->resolution_h);
    lua_pushtablenumber(L, "resolution_v", payload->resolution_v);
    lua_pushtablenumber(L, "bitrate", payload->bitrate);
    lua_pushtablenumber(L, "rotation", payload->rotation);
    lua_pushtablenumber(L, "hfov", payload->hfov);
    lua_pushstring(L, "name"); // array name[32]
    lua_newtable(L);
    for (int i = 0; i < 32; i++) { 
      lua_pushtableinumber(L, i+1, payload->name[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushstring(L, "uri"); // array uri[160]
    lua_newtable(L);
    for (int i = 0; i < 160; i++) { 
      lua_pushtableinumber(L, i+1, payload->uri[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    return;
    }
  case FASTMAVLINK_MSG_ID_VIDEO_STREAM_STATUS: { // #270
    fmav_video_stream_status_t* payload = (fmav_video_stream_status_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "stream_id", payload->stream_id);
    lua_pushtablenumber(L, "flags", payload->flags);
    lua_pushtablenumber(L, "framerate", payload->framerate);
    lua_pushtablenumber(L, "resolution_h", payload->resolution_h);
    lua_pushtablenumber(L, "resolution_v", payload->resolution_v);
    lua_pushtablenumber(L, "bitrate", payload->bitrate);
    lua_pushtablenumber(L, "rotation", payload->rotation);
    lua_pushtablenumber(L, "hfov", payload->hfov);
    return;
    }
  case FASTMAVLINK_MSG_ID_CAMERA_FOV_STATUS: { // #271
    fmav_camera_fov_status_t* payload = (fmav_camera_fov_status_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "time_boot_ms", payload->time_boot_ms);
    lua_pushtablenumber(L, "lat_camera", payload->lat_camera);
    lua_pushtablenumber(L, "lon_camera", payload->lon_camera);
    lua_pushtablenumber(L, "alt_camera", payload->alt_camera);
    lua_pushtablenumber(L, "lat_image", payload->lat_image);
    lua_pushtablenumber(L, "lon_image", payload->lon_image);
    lua_pushtablenumber(L, "alt_image", payload->alt_image);
    lua_pushstring(L, "q"); // array q[4]
    lua_newtable(L);
    for (int i = 0; i < 4; i++) { 
      lua_pushtableinumber(L, i+1, payload->q[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushtablenumber(L, "hfov", payload->hfov);
    lua_pushtablenumber(L, "vfov", payload->vfov);
    return;
    }
  case FASTMAVLINK_MSG_ID_CAMERA_TRACKING_IMAGE_STATUS: { // #275
    fmav_camera_tracking_image_status_t* payload = (fmav_camera_tracking_image_status_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "tracking_status", payload->tracking_status);
    lua_pushtablenumber(L, "tracking_mode", payload->tracking_mode);
    lua_pushtablenumber(L, "target_data", payload->target_data);
    lua_pushtablenumber(L, "point_x", payload->point_x);
    lua_pushtablenumber(L, "point_y", payload->point_y);
    lua_pushtablenumber(L, "radius", payload->radius);
    lua_pushtablenumber(L, "rec_top_x", payload->rec_top_x);
    lua_pushtablenumber(L, "rec_top_y", payload->rec_top_y);
    lua_pushtablenumber(L, "rec_bottom_x", payload->rec_bottom_x);
    lua_pushtablenumber(L, "rec_bottom_y", payload->rec_bottom_y);
    return;
    }
  case FASTMAVLINK_MSG_ID_CAMERA_TRACKING_GEO_STATUS: { // #276
    fmav_camera_tracking_geo_status_t* payload = (fmav_camera_tracking_geo_status_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "tracking_status", payload->tracking_status);
    lua_pushtablenumber(L, "lat", payload->lat);
    lua_pushtablenumber(L, "lon", payload->lon);
    lua_pushtablenumber(L, "alt", payload->alt);
    lua_pushtablenumber(L, "h_acc", payload->h_acc);
    lua_pushtablenumber(L, "v_acc", payload->v_acc);
    lua_pushtablenumber(L, "vel_n", payload->vel_n);
    lua_pushtablenumber(L, "vel_e", payload->vel_e);
    lua_pushtablenumber(L, "vel_d", payload->vel_d);
    lua_pushtablenumber(L, "vel_acc", payload->vel_acc);
    lua_pushtablenumber(L, "dist", payload->dist);
    lua_pushtablenumber(L, "hdg", payload->hdg);
    lua_pushtablenumber(L, "hdg_acc", payload->hdg_acc);
    return;
    }
  case FASTMAVLINK_MSG_ID_GIMBAL_MANAGER_INFORMATION: { // #280
    fmav_gimbal_manager_information_t* payload = (fmav_gimbal_manager_information_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "time_boot_ms", payload->time_boot_ms);
    lua_pushtablenumber(L, "cap_flags", payload->cap_flags);
    lua_pushtablenumber(L, "gimbal_device_id", payload->gimbal_device_id);
    lua_pushtablenumber(L, "roll_min", payload->roll_min);
    lua_pushtablenumber(L, "roll_max", payload->roll_max);
    lua_pushtablenumber(L, "pitch_min", payload->pitch_min);
    lua_pushtablenumber(L, "pitch_max", payload->pitch_max);
    lua_pushtablenumber(L, "yaw_min", payload->yaw_min);
    lua_pushtablenumber(L, "yaw_max", payload->yaw_max);
    return;
    }
  case FASTMAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS: { // #281
    fmav_gimbal_manager_status_t* payload = (fmav_gimbal_manager_status_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "time_boot_ms", payload->time_boot_ms);
    lua_pushtablenumber(L, "flags", payload->flags);
    lua_pushtablenumber(L, "gimbal_device_id", payload->gimbal_device_id);
    lua_pushtablenumber(L, "primary_control_sysid", payload->primary_control_sysid);
    lua_pushtablenumber(L, "primary_control_compid", payload->primary_control_compid);
    lua_pushtablenumber(L, "secondary_control_sysid", payload->secondary_control_sysid);
    lua_pushtablenumber(L, "secondary_control_compid", payload->secondary_control_compid);
    return;
    }
  case FASTMAVLINK_MSG_ID_GIMBAL_MANAGER_SET_ATTITUDE: { // #282
    fmav_gimbal_manager_set_attitude_t* payload = (fmav_gimbal_manager_set_attitude_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "flags", payload->flags);
    lua_pushtablenumber(L, "gimbal_device_id", payload->gimbal_device_id);
    lua_pushstring(L, "q"); // array q[4]
    lua_newtable(L);
    for (int i = 0; i < 4; i++) { 
      lua_pushtableinumber(L, i+1, payload->q[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushtablenumber(L, "angular_velocity_x", payload->angular_velocity_x);
    lua_pushtablenumber(L, "angular_velocity_y", payload->angular_velocity_y);
    lua_pushtablenumber(L, "angular_velocity_z", payload->angular_velocity_z);
    return;
    }
  case FASTMAVLINK_MSG_ID_GIMBAL_DEVICE_INFORMATION: { // #283
    fmav_gimbal_device_information_t* payload = (fmav_gimbal_device_information_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "time_boot_ms", payload->time_boot_ms);
    lua_pushstring(L, "vendor_name"); // array vendor_name[32]
    lua_newtable(L);
    for (int i = 0; i < 32; i++) { 
      lua_pushtableinumber(L, i+1, payload->vendor_name[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushstring(L, "model_name"); // array model_name[32]
    lua_newtable(L);
    for (int i = 0; i < 32; i++) { 
      lua_pushtableinumber(L, i+1, payload->model_name[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushstring(L, "custom_name"); // array custom_name[32]
    lua_newtable(L);
    for (int i = 0; i < 32; i++) { 
      lua_pushtableinumber(L, i+1, payload->custom_name[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushtablenumber(L, "firmware_version", payload->firmware_version);
    lua_pushtablenumber(L, "hardware_version", payload->hardware_version);
    lua_pushtablenumber(L, "uid", payload->uid);
    lua_pushtablenumber(L, "cap_flags", payload->cap_flags);
    lua_pushtablenumber(L, "custom_cap_flags", payload->custom_cap_flags);
    lua_pushtablenumber(L, "roll_min", payload->roll_min);
    lua_pushtablenumber(L, "roll_max", payload->roll_max);
    lua_pushtablenumber(L, "pitch_min", payload->pitch_min);
    lua_pushtablenumber(L, "pitch_max", payload->pitch_max);
    lua_pushtablenumber(L, "yaw_min", payload->yaw_min);
    lua_pushtablenumber(L, "yaw_max", payload->yaw_max);
    lua_pushtablenumber(L, "gimbal_device_id", payload->gimbal_device_id);
    return;
    }
  case FASTMAVLINK_MSG_ID_GIMBAL_DEVICE_SET_ATTITUDE: { // #284
    fmav_gimbal_device_set_attitude_t* payload = (fmav_gimbal_device_set_attitude_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "flags", payload->flags);
    lua_pushstring(L, "q"); // array q[4]
    lua_newtable(L);
    for (int i = 0; i < 4; i++) { 
      lua_pushtableinumber(L, i+1, payload->q[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushtablenumber(L, "angular_velocity_x", payload->angular_velocity_x);
    lua_pushtablenumber(L, "angular_velocity_y", payload->angular_velocity_y);
    lua_pushtablenumber(L, "angular_velocity_z", payload->angular_velocity_z);
    return;
    }
  case FASTMAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS: { // #285
    fmav_gimbal_device_attitude_status_t* payload = (fmav_gimbal_device_attitude_status_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "time_boot_ms", payload->time_boot_ms);
    lua_pushtablenumber(L, "flags", payload->flags);
    lua_pushstring(L, "q"); // array q[4]
    lua_newtable(L);
    for (int i = 0; i < 4; i++) { 
      lua_pushtableinumber(L, i+1, payload->q[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushtablenumber(L, "angular_velocity_x", payload->angular_velocity_x);
    lua_pushtablenumber(L, "angular_velocity_y", payload->angular_velocity_y);
    lua_pushtablenumber(L, "angular_velocity_z", payload->angular_velocity_z);
    lua_pushtablenumber(L, "failure_flags", payload->failure_flags);
    lua_pushtablenumber(L, "delta_yaw", payload->delta_yaw);
    lua_pushtablenumber(L, "delta_yaw_velocity", payload->delta_yaw_velocity);
    lua_pushtablenumber(L, "gimbal_device_id", payload->gimbal_device_id);
    return;
    }
  case FASTMAVLINK_MSG_ID_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE: { // #286
    fmav_autopilot_state_for_gimbal_device_t* payload = (fmav_autopilot_state_for_gimbal_device_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "time_boot_us", payload->time_boot_us);
    lua_pushstring(L, "q"); // array q[4]
    lua_newtable(L);
    for (int i = 0; i < 4; i++) { 
      lua_pushtableinumber(L, i+1, payload->q[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushtablenumber(L, "q_estimated_delay_us", payload->q_estimated_delay_us);
    lua_pushtablenumber(L, "vx", payload->vx);
    lua_pushtablenumber(L, "vy", payload->vy);
    lua_pushtablenumber(L, "vz", payload->vz);
    lua_pushtablenumber(L, "v_estimated_delay_us", payload->v_estimated_delay_us);
    lua_pushtablenumber(L, "feed_forward_angular_velocity_z", payload->feed_forward_angular_velocity_z);
    lua_pushtablenumber(L, "estimator_status", payload->estimator_status);
    lua_pushtablenumber(L, "landed_state", payload->landed_state);
    lua_pushtablenumber(L, "angular_velocity_z", payload->angular_velocity_z);
    return;
    }
  case FASTMAVLINK_MSG_ID_GIMBAL_MANAGER_SET_PITCHYAW: { // #287
    fmav_gimbal_manager_set_pitchyaw_t* payload = (fmav_gimbal_manager_set_pitchyaw_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "flags", payload->flags);
    lua_pushtablenumber(L, "gimbal_device_id", payload->gimbal_device_id);
    lua_pushtablenumber(L, "pitch", payload->pitch);
    lua_pushtablenumber(L, "yaw", payload->yaw);
    lua_pushtablenumber(L, "pitch_rate", payload->pitch_rate);
    lua_pushtablenumber(L, "yaw_rate", payload->yaw_rate);
    return;
    }
  case FASTMAVLINK_MSG_ID_GIMBAL_MANAGER_SET_MANUAL_CONTROL: { // #288
    fmav_gimbal_manager_set_manual_control_t* payload = (fmav_gimbal_manager_set_manual_control_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "flags", payload->flags);
    lua_pushtablenumber(L, "gimbal_device_id", payload->gimbal_device_id);
    lua_pushtablenumber(L, "pitch", payload->pitch);
    lua_pushtablenumber(L, "yaw", payload->yaw);
    lua_pushtablenumber(L, "pitch_rate", payload->pitch_rate);
    lua_pushtablenumber(L, "yaw_rate", payload->yaw_rate);
    return;
    }
  case FASTMAVLINK_MSG_ID_ESC_INFO: { // #290
    fmav_esc_info_t* payload = (fmav_esc_info_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "index", payload->index);
    lua_pushtablenumber(L, "time_usec", payload->time_usec);
    lua_pushtablenumber(L, "counter", payload->counter);
    lua_pushtablenumber(L, "count", payload->count);
    lua_pushtablenumber(L, "connection_type", payload->connection_type);
    lua_pushtablenumber(L, "info", payload->info);
    lua_pushstring(L, "failure_flags"); // array failure_flags[4]
    lua_newtable(L);
    for (int i = 0; i < 4; i++) { 
      lua_pushtableinumber(L, i+1, payload->failure_flags[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushstring(L, "error_count"); // array error_count[4]
    lua_newtable(L);
    for (int i = 0; i < 4; i++) { 
      lua_pushtableinumber(L, i+1, payload->error_count[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushstring(L, "temperature"); // array temperature[4]
    lua_newtable(L);
    for (int i = 0; i < 4; i++) { 
      lua_pushtableinumber(L, i+1, payload->temperature[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    return;
    }
  case FASTMAVLINK_MSG_ID_ESC_STATUS: { // #291
    fmav_esc_status_t* payload = (fmav_esc_status_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "index", payload->index);
    lua_pushtablenumber(L, "time_usec", payload->time_usec);
    lua_pushstring(L, "rpm"); // array rpm[4]
    lua_newtable(L);
    for (int i = 0; i < 4; i++) { 
      lua_pushtableinumber(L, i+1, payload->rpm[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushstring(L, "voltage"); // array voltage[4]
    lua_newtable(L);
    for (int i = 0; i < 4; i++) { 
      lua_pushtableinumber(L, i+1, payload->voltage[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushstring(L, "current"); // array current[4]
    lua_newtable(L);
    for (int i = 0; i < 4; i++) { 
      lua_pushtableinumber(L, i+1, payload->current[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    return;
    }
  case FASTMAVLINK_MSG_ID_WIFI_CONFIG_AP: { // #299
    fmav_wifi_config_ap_t* payload = (fmav_wifi_config_ap_t*)(mavmsg->payload_ptr);
    lua_pushstring(L, "ssid"); // array ssid[32]
    lua_newtable(L);
    for (int i = 0; i < 32; i++) { 
      lua_pushtableinumber(L, i+1, payload->ssid[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushstring(L, "password"); // array password[64]
    lua_newtable(L);
    for (int i = 0; i < 64; i++) { 
      lua_pushtableinumber(L, i+1, payload->password[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushtablenumber(L, "mode", payload->mode);
    lua_pushtablenumber(L, "response", payload->response);
    return;
    }
  case FASTMAVLINK_MSG_ID_PROTOCOL_VERSION: { // #300
    fmav_protocol_version_t* payload = (fmav_protocol_version_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "version", payload->version);
    lua_pushtablenumber(L, "min_version", payload->min_version);
    lua_pushtablenumber(L, "max_version", payload->max_version);
    lua_pushstring(L, "spec_version_hash"); // array spec_version_hash[8]
    lua_newtable(L);
    for (int i = 0; i < 8; i++) { 
      lua_pushtableinumber(L, i+1, payload->spec_version_hash[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushstring(L, "library_version_hash"); // array library_version_hash[8]
    lua_newtable(L);
    for (int i = 0; i < 8; i++) { 
      lua_pushtableinumber(L, i+1, payload->library_version_hash[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    return;
    }
  case FASTMAVLINK_MSG_ID_AIS_VESSEL: { // #301
    fmav_ais_vessel_t* payload = (fmav_ais_vessel_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "MMSI", payload->MMSI);
    lua_pushtablenumber(L, "lat", payload->lat);
    lua_pushtablenumber(L, "lon", payload->lon);
    lua_pushtablenumber(L, "COG", payload->COG);
    lua_pushtablenumber(L, "heading", payload->heading);
    lua_pushtablenumber(L, "velocity", payload->velocity);
    lua_pushtablenumber(L, "turn_rate", payload->turn_rate);
    lua_pushtablenumber(L, "navigational_status", payload->navigational_status);
    lua_pushtablenumber(L, "type", payload->type);
    lua_pushtablenumber(L, "dimension_bow", payload->dimension_bow);
    lua_pushtablenumber(L, "dimension_stern", payload->dimension_stern);
    lua_pushtablenumber(L, "dimension_port", payload->dimension_port);
    lua_pushtablenumber(L, "dimension_starboard", payload->dimension_starboard);
    lua_pushstring(L, "callsign"); // array callsign[7]
    lua_newtable(L);
    for (int i = 0; i < 7; i++) { 
      lua_pushtableinumber(L, i+1, payload->callsign[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushstring(L, "name"); // array name[20]
    lua_newtable(L);
    for (int i = 0; i < 20; i++) { 
      lua_pushtableinumber(L, i+1, payload->name[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushtablenumber(L, "tslc", payload->tslc);
    lua_pushtablenumber(L, "flags", payload->flags);
    return;
    }
  case FASTMAVLINK_MSG_ID_UAVCAN_NODE_STATUS: { // #310
    fmav_uavcan_node_status_t* payload = (fmav_uavcan_node_status_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "time_usec", payload->time_usec);
    lua_pushtablenumber(L, "uptime_sec", payload->uptime_sec);
    lua_pushtablenumber(L, "health", payload->health);
    lua_pushtablenumber(L, "mode", payload->mode);
    lua_pushtablenumber(L, "sub_mode", payload->sub_mode);
    lua_pushtablenumber(L, "vendor_specific_status_code", payload->vendor_specific_status_code);
    return;
    }
  case FASTMAVLINK_MSG_ID_UAVCAN_NODE_INFO: { // #311
    fmav_uavcan_node_info_t* payload = (fmav_uavcan_node_info_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "time_usec", payload->time_usec);
    lua_pushtablenumber(L, "uptime_sec", payload->uptime_sec);
    lua_pushstring(L, "name"); // array name[80]
    lua_newtable(L);
    for (int i = 0; i < 80; i++) { 
      lua_pushtableinumber(L, i+1, payload->name[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushtablenumber(L, "hw_version_major", payload->hw_version_major);
    lua_pushtablenumber(L, "hw_version_minor", payload->hw_version_minor);
    lua_pushstring(L, "hw_unique_id"); // array hw_unique_id[16]
    lua_newtable(L);
    for (int i = 0; i < 16; i++) { 
      lua_pushtableinumber(L, i+1, payload->hw_unique_id[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushtablenumber(L, "sw_version_major", payload->sw_version_major);
    lua_pushtablenumber(L, "sw_version_minor", payload->sw_version_minor);
    lua_pushtablenumber(L, "sw_vcs_commit", payload->sw_vcs_commit);
    return;
    }
  case FASTMAVLINK_MSG_ID_PARAM_EXT_REQUEST_READ: { // #320
    fmav_param_ext_request_read_t* payload = (fmav_param_ext_request_read_t*)(mavmsg->payload_ptr);
    lua_pushstring(L, "param_id"); // array param_id[16]
    lua_newtable(L);
    for (int i = 0; i < 16; i++) { 
      lua_pushtableinumber(L, i+1, payload->param_id[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushtablenumber(L, "param_index", payload->param_index);
    return;
    }
  case FASTMAVLINK_MSG_ID_PARAM_EXT_REQUEST_LIST: { // #321
    return;
    }
  case FASTMAVLINK_MSG_ID_PARAM_EXT_VALUE: { // #322
    fmav_param_ext_value_t* payload = (fmav_param_ext_value_t*)(mavmsg->payload_ptr);
    lua_pushstring(L, "param_id"); // array param_id[16]
    lua_newtable(L);
    for (int i = 0; i < 16; i++) { 
      lua_pushtableinumber(L, i+1, payload->param_id[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushstring(L, "param_value"); // array param_value[128]
    lua_newtable(L);
    for (int i = 0; i < 128; i++) { 
      lua_pushtableinumber(L, i+1, payload->param_value[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushtablenumber(L, "param_type", payload->param_type);
    lua_pushtablenumber(L, "param_count", payload->param_count);
    lua_pushtablenumber(L, "param_index", payload->param_index);
    return;
    }
  case FASTMAVLINK_MSG_ID_PARAM_EXT_SET: { // #323
    fmav_param_ext_set_t* payload = (fmav_param_ext_set_t*)(mavmsg->payload_ptr);
    lua_pushstring(L, "param_id"); // array param_id[16]
    lua_newtable(L);
    for (int i = 0; i < 16; i++) { 
      lua_pushtableinumber(L, i+1, payload->param_id[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushstring(L, "param_value"); // array param_value[128]
    lua_newtable(L);
    for (int i = 0; i < 128; i++) { 
      lua_pushtableinumber(L, i+1, payload->param_value[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushtablenumber(L, "param_type", payload->param_type);
    return;
    }
  case FASTMAVLINK_MSG_ID_PARAM_EXT_ACK: { // #324
    fmav_param_ext_ack_t* payload = (fmav_param_ext_ack_t*)(mavmsg->payload_ptr);
    lua_pushstring(L, "param_id"); // array param_id[16]
    lua_newtable(L);
    for (int i = 0; i < 16; i++) { 
      lua_pushtableinumber(L, i+1, payload->param_id[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushstring(L, "param_value"); // array param_value[128]
    lua_newtable(L);
    for (int i = 0; i < 128; i++) { 
      lua_pushtableinumber(L, i+1, payload->param_value[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushtablenumber(L, "param_type", payload->param_type);
    lua_pushtablenumber(L, "param_result", payload->param_result);
    return;
    }
  case FASTMAVLINK_MSG_ID_OBSTACLE_DISTANCE: { // #330
    fmav_obstacle_distance_t* payload = (fmav_obstacle_distance_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "time_usec", payload->time_usec);
    lua_pushtablenumber(L, "sensor_type", payload->sensor_type);
    lua_pushstring(L, "distances"); // array distances[72]
    lua_newtable(L);
    for (int i = 0; i < 72; i++) { 
      lua_pushtableinumber(L, i+1, payload->distances[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushtablenumber(L, "increment", payload->increment);
    lua_pushtablenumber(L, "min_distance", payload->min_distance);
    lua_pushtablenumber(L, "max_distance", payload->max_distance);
    lua_pushtablenumber(L, "increment_f", payload->increment_f);
    lua_pushtablenumber(L, "angle_offset", payload->angle_offset);
    lua_pushtablenumber(L, "frame", payload->frame);
    return;
    }
  case FASTMAVLINK_MSG_ID_ODOMETRY: { // #331
    fmav_odometry_t* payload = (fmav_odometry_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "time_usec", payload->time_usec);
    lua_pushtablenumber(L, "frame_id", payload->frame_id);
    lua_pushtablenumber(L, "child_frame_id", payload->child_frame_id);
    lua_pushtablenumber(L, "x", payload->x);
    lua_pushtablenumber(L, "y", payload->y);
    lua_pushtablenumber(L, "z", payload->z);
    lua_pushstring(L, "q"); // array q[4]
    lua_newtable(L);
    for (int i = 0; i < 4; i++) { 
      lua_pushtableinumber(L, i+1, payload->q[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushtablenumber(L, "vx", payload->vx);
    lua_pushtablenumber(L, "vy", payload->vy);
    lua_pushtablenumber(L, "vz", payload->vz);
    lua_pushtablenumber(L, "rollspeed", payload->rollspeed);
    lua_pushtablenumber(L, "pitchspeed", payload->pitchspeed);
    lua_pushtablenumber(L, "yawspeed", payload->yawspeed);
    lua_pushstring(L, "pose_covariance"); // array pose_covariance[21]
    lua_newtable(L);
    for (int i = 0; i < 21; i++) { 
      lua_pushtableinumber(L, i+1, payload->pose_covariance[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushstring(L, "velocity_covariance"); // array velocity_covariance[21]
    lua_newtable(L);
    for (int i = 0; i < 21; i++) { 
      lua_pushtableinumber(L, i+1, payload->velocity_covariance[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushtablenumber(L, "reset_counter", payload->reset_counter);
    lua_pushtablenumber(L, "estimator_type", payload->estimator_type);
    lua_pushtablenumber(L, "quality", payload->quality);
    return;
    }
  case FASTMAVLINK_MSG_ID_TRAJECTORY_REPRESENTATION_WAYPOINTS: { // #332
    fmav_trajectory_representation_waypoints_t* payload = (fmav_trajectory_representation_waypoints_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "time_usec", payload->time_usec);
    lua_pushtablenumber(L, "valid_points", payload->valid_points);
    lua_pushstring(L, "pos_x"); // array pos_x[5]
    lua_newtable(L);
    for (int i = 0; i < 5; i++) { 
      lua_pushtableinumber(L, i+1, payload->pos_x[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushstring(L, "pos_y"); // array pos_y[5]
    lua_newtable(L);
    for (int i = 0; i < 5; i++) { 
      lua_pushtableinumber(L, i+1, payload->pos_y[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushstring(L, "pos_z"); // array pos_z[5]
    lua_newtable(L);
    for (int i = 0; i < 5; i++) { 
      lua_pushtableinumber(L, i+1, payload->pos_z[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushstring(L, "vel_x"); // array vel_x[5]
    lua_newtable(L);
    for (int i = 0; i < 5; i++) { 
      lua_pushtableinumber(L, i+1, payload->vel_x[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushstring(L, "vel_y"); // array vel_y[5]
    lua_newtable(L);
    for (int i = 0; i < 5; i++) { 
      lua_pushtableinumber(L, i+1, payload->vel_y[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushstring(L, "vel_z"); // array vel_z[5]
    lua_newtable(L);
    for (int i = 0; i < 5; i++) { 
      lua_pushtableinumber(L, i+1, payload->vel_z[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushstring(L, "acc_x"); // array acc_x[5]
    lua_newtable(L);
    for (int i = 0; i < 5; i++) { 
      lua_pushtableinumber(L, i+1, payload->acc_x[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushstring(L, "acc_y"); // array acc_y[5]
    lua_newtable(L);
    for (int i = 0; i < 5; i++) { 
      lua_pushtableinumber(L, i+1, payload->acc_y[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushstring(L, "acc_z"); // array acc_z[5]
    lua_newtable(L);
    for (int i = 0; i < 5; i++) { 
      lua_pushtableinumber(L, i+1, payload->acc_z[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushstring(L, "pos_yaw"); // array pos_yaw[5]
    lua_newtable(L);
    for (int i = 0; i < 5; i++) { 
      lua_pushtableinumber(L, i+1, payload->pos_yaw[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushstring(L, "vel_yaw"); // array vel_yaw[5]
    lua_newtable(L);
    for (int i = 0; i < 5; i++) { 
      lua_pushtableinumber(L, i+1, payload->vel_yaw[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushstring(L, "command"); // array command[5]
    lua_newtable(L);
    for (int i = 0; i < 5; i++) { 
      lua_pushtableinumber(L, i+1, payload->command[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    return;
    }
  case FASTMAVLINK_MSG_ID_TRAJECTORY_REPRESENTATION_BEZIER: { // #333
    fmav_trajectory_representation_bezier_t* payload = (fmav_trajectory_representation_bezier_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "time_usec", payload->time_usec);
    lua_pushtablenumber(L, "valid_points", payload->valid_points);
    lua_pushstring(L, "pos_x"); // array pos_x[5]
    lua_newtable(L);
    for (int i = 0; i < 5; i++) { 
      lua_pushtableinumber(L, i+1, payload->pos_x[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushstring(L, "pos_y"); // array pos_y[5]
    lua_newtable(L);
    for (int i = 0; i < 5; i++) { 
      lua_pushtableinumber(L, i+1, payload->pos_y[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushstring(L, "pos_z"); // array pos_z[5]
    lua_newtable(L);
    for (int i = 0; i < 5; i++) { 
      lua_pushtableinumber(L, i+1, payload->pos_z[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushstring(L, "delta"); // array delta[5]
    lua_newtable(L);
    for (int i = 0; i < 5; i++) { 
      lua_pushtableinumber(L, i+1, payload->delta[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushstring(L, "pos_yaw"); // array pos_yaw[5]
    lua_newtable(L);
    for (int i = 0; i < 5; i++) { 
      lua_pushtableinumber(L, i+1, payload->pos_yaw[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    return;
    }
  case FASTMAVLINK_MSG_ID_CELLULAR_STATUS: { // #334
    fmav_cellular_status_t* payload = (fmav_cellular_status_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "status", payload->status);
    lua_pushtablenumber(L, "failure_reason", payload->failure_reason);
    lua_pushtablenumber(L, "type", payload->type);
    lua_pushtablenumber(L, "quality", payload->quality);
    lua_pushtablenumber(L, "mcc", payload->mcc);
    lua_pushtablenumber(L, "mnc", payload->mnc);
    lua_pushtablenumber(L, "lac", payload->lac);
    return;
    }
  case FASTMAVLINK_MSG_ID_ISBD_LINK_STATUS: { // #335
    fmav_isbd_link_status_t* payload = (fmav_isbd_link_status_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "timestamp", payload->timestamp);
    lua_pushtablenumber(L, "last_heartbeat", payload->last_heartbeat);
    lua_pushtablenumber(L, "failed_sessions", payload->failed_sessions);
    lua_pushtablenumber(L, "successful_sessions", payload->successful_sessions);
    lua_pushtablenumber(L, "signal_quality", payload->signal_quality);
    lua_pushtablenumber(L, "ring_pending", payload->ring_pending);
    lua_pushtablenumber(L, "tx_session_pending", payload->tx_session_pending);
    lua_pushtablenumber(L, "rx_session_pending", payload->rx_session_pending);
    return;
    }
  case FASTMAVLINK_MSG_ID_CELLULAR_CONFIG: { // #336
    fmav_cellular_config_t* payload = (fmav_cellular_config_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "enable_lte", payload->enable_lte);
    lua_pushtablenumber(L, "enable_pin", payload->enable_pin);
    lua_pushstring(L, "pin"); // array pin[16]
    lua_newtable(L);
    for (int i = 0; i < 16; i++) { 
      lua_pushtableinumber(L, i+1, payload->pin[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushstring(L, "new_pin"); // array new_pin[16]
    lua_newtable(L);
    for (int i = 0; i < 16; i++) { 
      lua_pushtableinumber(L, i+1, payload->new_pin[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushstring(L, "apn"); // array apn[32]
    lua_newtable(L);
    for (int i = 0; i < 32; i++) { 
      lua_pushtableinumber(L, i+1, payload->apn[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushstring(L, "puk"); // array puk[16]
    lua_newtable(L);
    for (int i = 0; i < 16; i++) { 
      lua_pushtableinumber(L, i+1, payload->puk[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushtablenumber(L, "roaming", payload->roaming);
    lua_pushtablenumber(L, "response", payload->response);
    return;
    }
  case FASTMAVLINK_MSG_ID_RAW_RPM: { // #339
    fmav_raw_rpm_t* payload = (fmav_raw_rpm_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "index", payload->index);
    lua_pushtablenumber(L, "frequency", payload->frequency);
    return;
    }
  case FASTMAVLINK_MSG_ID_UTM_GLOBAL_POSITION: { // #340
    fmav_utm_global_position_t* payload = (fmav_utm_global_position_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "time", payload->time);
    lua_pushstring(L, "uas_id"); // array uas_id[18]
    lua_newtable(L);
    for (int i = 0; i < 18; i++) { 
      lua_pushtableinumber(L, i+1, payload->uas_id[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushtablenumber(L, "lat", payload->lat);
    lua_pushtablenumber(L, "lon", payload->lon);
    lua_pushtablenumber(L, "alt", payload->alt);
    lua_pushtablenumber(L, "relative_alt", payload->relative_alt);
    lua_pushtablenumber(L, "vx", payload->vx);
    lua_pushtablenumber(L, "vy", payload->vy);
    lua_pushtablenumber(L, "vz", payload->vz);
    lua_pushtablenumber(L, "h_acc", payload->h_acc);
    lua_pushtablenumber(L, "v_acc", payload->v_acc);
    lua_pushtablenumber(L, "vel_acc", payload->vel_acc);
    lua_pushtablenumber(L, "next_lat", payload->next_lat);
    lua_pushtablenumber(L, "next_lon", payload->next_lon);
    lua_pushtablenumber(L, "next_alt", payload->next_alt);
    lua_pushtablenumber(L, "update_rate", payload->update_rate);
    lua_pushtablenumber(L, "flight_state", payload->flight_state);
    lua_pushtablenumber(L, "flags", payload->flags);
    return;
    }
  case FASTMAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY: { // #350
    fmav_debug_float_array_t* payload = (fmav_debug_float_array_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "time_usec", payload->time_usec);
    lua_pushstring(L, "name"); // array name[10]
    lua_newtable(L);
    for (int i = 0; i < 10; i++) { 
      lua_pushtableinumber(L, i+1, payload->name[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushtablenumber(L, "array_id", payload->array_id);
    lua_pushstring(L, "data"); // array data[58]
    lua_newtable(L);
    for (int i = 0; i < 58; i++) { 
      lua_pushtableinumber(L, i+1, payload->data[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    return;
    }
  case FASTMAVLINK_MSG_ID_ORBIT_EXECUTION_STATUS: { // #360
    fmav_orbit_execution_status_t* payload = (fmav_orbit_execution_status_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "time_usec", payload->time_usec);
    lua_pushtablenumber(L, "radius", payload->radius);
    lua_pushtablenumber(L, "frame", payload->frame);
    lua_pushtablenumber(L, "x", payload->x);
    lua_pushtablenumber(L, "y", payload->y);
    lua_pushtablenumber(L, "z", payload->z);
    return;
    }
  case FASTMAVLINK_MSG_ID_SMART_BATTERY_INFO: { // #370
    fmav_smart_battery_info_t* payload = (fmav_smart_battery_info_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "id", payload->id);
    lua_pushtablenumber(L, "battery_function", payload->battery_function);
    lua_pushtablenumber(L, "type", payload->type);
    lua_pushtablenumber(L, "capacity_full_specification", payload->capacity_full_specification);
    lua_pushtablenumber(L, "capacity_full", payload->capacity_full);
    lua_pushtablenumber(L, "cycle_count", payload->cycle_count);
    lua_pushstring(L, "serial_number"); // array serial_number[16]
    lua_newtable(L);
    for (int i = 0; i < 16; i++) { 
      lua_pushtableinumber(L, i+1, payload->serial_number[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushstring(L, "device_name"); // array device_name[50]
    lua_newtable(L);
    for (int i = 0; i < 50; i++) { 
      lua_pushtableinumber(L, i+1, payload->device_name[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushtablenumber(L, "weight", payload->weight);
    lua_pushtablenumber(L, "discharge_minimum_voltage", payload->discharge_minimum_voltage);
    lua_pushtablenumber(L, "charging_minimum_voltage", payload->charging_minimum_voltage);
    lua_pushtablenumber(L, "resting_minimum_voltage", payload->resting_minimum_voltage);
    lua_pushtablenumber(L, "charging_maximum_voltage", payload->charging_maximum_voltage);
    lua_pushtablenumber(L, "cells_in_series", payload->cells_in_series);
    lua_pushtablenumber(L, "discharge_maximum_current", payload->discharge_maximum_current);
    lua_pushtablenumber(L, "discharge_maximum_burst_current", payload->discharge_maximum_burst_current);
    lua_pushstring(L, "manufacture_date"); // array manufacture_date[11]
    lua_newtable(L);
    for (int i = 0; i < 11; i++) { 
      lua_pushtableinumber(L, i+1, payload->manufacture_date[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    return;
    }
  case FASTMAVLINK_MSG_ID_GENERATOR_STATUS: { // #373
    fmav_generator_status_t* payload = (fmav_generator_status_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "status", payload->status);
    lua_pushtablenumber(L, "generator_speed", payload->generator_speed);
    lua_pushtablenumber(L, "battery_current", payload->battery_current);
    lua_pushtablenumber(L, "load_current", payload->load_current);
    lua_pushtablenumber(L, "power_generated", payload->power_generated);
    lua_pushtablenumber(L, "bus_voltage", payload->bus_voltage);
    lua_pushtablenumber(L, "rectifier_temperature", payload->rectifier_temperature);
    lua_pushtablenumber(L, "bat_current_setpoint", payload->bat_current_setpoint);
    lua_pushtablenumber(L, "generator_temperature", payload->generator_temperature);
    lua_pushtablenumber(L, "runtime", payload->runtime);
    lua_pushtablenumber(L, "time_until_maintenance", payload->time_until_maintenance);
    return;
    }
  case FASTMAVLINK_MSG_ID_ACTUATOR_OUTPUT_STATUS: { // #375
    fmav_actuator_output_status_t* payload = (fmav_actuator_output_status_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "time_usec", payload->time_usec);
    lua_pushtablenumber(L, "active", payload->active);
    lua_pushstring(L, "actuator"); // array actuator[32]
    lua_newtable(L);
    for (int i = 0; i < 32; i++) { 
      lua_pushtableinumber(L, i+1, payload->actuator[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    return;
    }
  case FASTMAVLINK_MSG_ID_TIME_ESTIMATE_TO_TARGET: { // #380
    fmav_time_estimate_to_target_t* payload = (fmav_time_estimate_to_target_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "safe_return", payload->safe_return);
    lua_pushtablenumber(L, "land", payload->land);
    lua_pushtablenumber(L, "mission_next_item", payload->mission_next_item);
    lua_pushtablenumber(L, "mission_end", payload->mission_end);
    lua_pushtablenumber(L, "commanded_action", payload->commanded_action);
    return;
    }
  case FASTMAVLINK_MSG_ID_TUNNEL: { // #385
    fmav_tunnel_t* payload = (fmav_tunnel_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "payload_type", payload->payload_type);
    lua_pushtablenumber(L, "payload_length", payload->payload_length);
    lua_pushstring(L, "payload"); // array payload[128]
    lua_newtable(L);
    for (int i = 0; i < 128; i++) { 
      lua_pushtableinumber(L, i+1, payload->payload[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    return;
    }
  case FASTMAVLINK_MSG_ID_CAN_FRAME: { // #386
    fmav_can_frame_t* payload = (fmav_can_frame_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "bus", payload->bus);
    lua_pushtablenumber(L, "len", payload->len);
    lua_pushtablenumber(L, "id", payload->id);
    lua_pushstring(L, "data"); // array data[8]
    lua_newtable(L);
    for (int i = 0; i < 8; i++) { 
      lua_pushtableinumber(L, i+1, payload->data[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    return;
    }
  case FASTMAVLINK_MSG_ID_CANFD_FRAME: { // #387
    fmav_canfd_frame_t* payload = (fmav_canfd_frame_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "bus", payload->bus);
    lua_pushtablenumber(L, "len", payload->len);
    lua_pushtablenumber(L, "id", payload->id);
    lua_pushstring(L, "data"); // array data[64]
    lua_newtable(L);
    for (int i = 0; i < 64; i++) { 
      lua_pushtableinumber(L, i+1, payload->data[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    return;
    }
  case FASTMAVLINK_MSG_ID_CAN_FILTER_MODIFY: { // #388
    fmav_can_filter_modify_t* payload = (fmav_can_filter_modify_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "bus", payload->bus);
    lua_pushtablenumber(L, "operation", payload->operation);
    lua_pushtablenumber(L, "num_ids", payload->num_ids);
    lua_pushstring(L, "ids"); // array ids[16]
    lua_newtable(L);
    for (int i = 0; i < 16; i++) { 
      lua_pushtableinumber(L, i+1, payload->ids[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    return;
    }
  case FASTMAVLINK_MSG_ID_ONBOARD_COMPUTER_STATUS: { // #390
    fmav_onboard_computer_status_t* payload = (fmav_onboard_computer_status_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "time_usec", payload->time_usec);
    lua_pushtablenumber(L, "uptime", payload->uptime);
    lua_pushtablenumber(L, "type", payload->type);
    lua_pushstring(L, "cpu_cores"); // array cpu_cores[8]
    lua_newtable(L);
    for (int i = 0; i < 8; i++) { 
      lua_pushtableinumber(L, i+1, payload->cpu_cores[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushstring(L, "cpu_combined"); // array cpu_combined[10]
    lua_newtable(L);
    for (int i = 0; i < 10; i++) { 
      lua_pushtableinumber(L, i+1, payload->cpu_combined[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushstring(L, "gpu_cores"); // array gpu_cores[4]
    lua_newtable(L);
    for (int i = 0; i < 4; i++) { 
      lua_pushtableinumber(L, i+1, payload->gpu_cores[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushstring(L, "gpu_combined"); // array gpu_combined[10]
    lua_newtable(L);
    for (int i = 0; i < 10; i++) { 
      lua_pushtableinumber(L, i+1, payload->gpu_combined[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushtablenumber(L, "temperature_board", payload->temperature_board);
    lua_pushstring(L, "temperature_core"); // array temperature_core[8]
    lua_newtable(L);
    for (int i = 0; i < 8; i++) { 
      lua_pushtableinumber(L, i+1, payload->temperature_core[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushstring(L, "fan_speed"); // array fan_speed[4]
    lua_newtable(L);
    for (int i = 0; i < 4; i++) { 
      lua_pushtableinumber(L, i+1, payload->fan_speed[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushtablenumber(L, "ram_usage", payload->ram_usage);
    lua_pushtablenumber(L, "ram_total", payload->ram_total);
    lua_pushstring(L, "storage_type"); // array storage_type[4]
    lua_newtable(L);
    for (int i = 0; i < 4; i++) { 
      lua_pushtableinumber(L, i+1, payload->storage_type[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushstring(L, "storage_usage"); // array storage_usage[4]
    lua_newtable(L);
    for (int i = 0; i < 4; i++) { 
      lua_pushtableinumber(L, i+1, payload->storage_usage[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushstring(L, "storage_total"); // array storage_total[4]
    lua_newtable(L);
    for (int i = 0; i < 4; i++) { 
      lua_pushtableinumber(L, i+1, payload->storage_total[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushstring(L, "link_type"); // array link_type[6]
    lua_newtable(L);
    for (int i = 0; i < 6; i++) { 
      lua_pushtableinumber(L, i+1, payload->link_type[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushstring(L, "link_tx_rate"); // array link_tx_rate[6]
    lua_newtable(L);
    for (int i = 0; i < 6; i++) { 
      lua_pushtableinumber(L, i+1, payload->link_tx_rate[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushstring(L, "link_rx_rate"); // array link_rx_rate[6]
    lua_newtable(L);
    for (int i = 0; i < 6; i++) { 
      lua_pushtableinumber(L, i+1, payload->link_rx_rate[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushstring(L, "link_tx_max"); // array link_tx_max[6]
    lua_newtable(L);
    for (int i = 0; i < 6; i++) { 
      lua_pushtableinumber(L, i+1, payload->link_tx_max[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushstring(L, "link_rx_max"); // array link_rx_max[6]
    lua_newtable(L);
    for (int i = 0; i < 6; i++) { 
      lua_pushtableinumber(L, i+1, payload->link_rx_max[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    return;
    }
  case FASTMAVLINK_MSG_ID_COMPONENT_INFORMATION: { // #395
    fmav_component_information_t* payload = (fmav_component_information_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "time_boot_ms", payload->time_boot_ms);
    lua_pushtablenumber(L, "general_metadata_file_crc", payload->general_metadata_file_crc);
    lua_pushstring(L, "general_metadata_uri"); // array general_metadata_uri[100]
    lua_newtable(L);
    for (int i = 0; i < 100; i++) { 
      lua_pushtableinumber(L, i+1, payload->general_metadata_uri[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushtablenumber(L, "peripherals_metadata_file_crc", payload->peripherals_metadata_file_crc);
    lua_pushstring(L, "peripherals_metadata_uri"); // array peripherals_metadata_uri[100]
    lua_newtable(L);
    for (int i = 0; i < 100; i++) { 
      lua_pushtableinumber(L, i+1, payload->peripherals_metadata_uri[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    return;
    }
  case FASTMAVLINK_MSG_ID_COMPONENT_METADATA: { // #397
    fmav_component_metadata_t* payload = (fmav_component_metadata_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "time_boot_ms", payload->time_boot_ms);
    lua_pushtablenumber(L, "file_crc", payload->file_crc);
    lua_pushstring(L, "uri"); // array uri[100]
    lua_newtable(L);
    for (int i = 0; i < 100; i++) { 
      lua_pushtableinumber(L, i+1, payload->uri[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    return;
    }
  case FASTMAVLINK_MSG_ID_PLAY_TUNE_V2: { // #400
    fmav_play_tune_v2_t* payload = (fmav_play_tune_v2_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "format", payload->format);
    lua_pushstring(L, "tune"); // array tune[248]
    lua_newtable(L);
    for (int i = 0; i < 248; i++) { 
      lua_pushtableinumber(L, i+1, payload->tune[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    return;
    }
  case FASTMAVLINK_MSG_ID_SUPPORTED_TUNES: { // #401
    fmav_supported_tunes_t* payload = (fmav_supported_tunes_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "format", payload->format);
    return;
    }
  case FASTMAVLINK_MSG_ID_EVENT: { // #410
    fmav_event_t* payload = (fmav_event_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "destination_component", payload->destination_component);
    lua_pushtablenumber(L, "destination_system", payload->destination_system);
    lua_pushtablenumber(L, "id", payload->id);
    lua_pushtablenumber(L, "event_time_boot_ms", payload->event_time_boot_ms);
    lua_pushtablenumber(L, "sequence", payload->sequence);
    lua_pushtablenumber(L, "log_levels", payload->log_levels);
    lua_pushstring(L, "arguments"); // array arguments[40]
    lua_newtable(L);
    for (int i = 0; i < 40; i++) { 
      lua_pushtableinumber(L, i+1, payload->arguments[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    return;
    }
  case FASTMAVLINK_MSG_ID_CURRENT_EVENT_SEQUENCE: { // #411
    fmav_current_event_sequence_t* payload = (fmav_current_event_sequence_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "sequence", payload->sequence);
    lua_pushtablenumber(L, "flags", payload->flags);
    return;
    }
  case FASTMAVLINK_MSG_ID_REQUEST_EVENT: { // #412
    fmav_request_event_t* payload = (fmav_request_event_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "first_sequence", payload->first_sequence);
    lua_pushtablenumber(L, "last_sequence", payload->last_sequence);
    return;
    }
  case FASTMAVLINK_MSG_ID_RESPONSE_EVENT_ERROR: { // #413
    fmav_response_event_error_t* payload = (fmav_response_event_error_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "sequence", payload->sequence);
    lua_pushtablenumber(L, "sequence_oldest_available", payload->sequence_oldest_available);
    lua_pushtablenumber(L, "reason", payload->reason);
    return;
    }
  case FASTMAVLINK_MSG_ID_RADIO_RC_CHANNELS_DEV: { // #420
    fmav_radio_rc_channels_dev_t* payload = (fmav_radio_rc_channels_dev_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "count", payload->count);
    lua_pushtablenumber(L, "flags", payload->flags);
    lua_pushstring(L, "channels"); // array channels[32]
    lua_newtable(L);
    for (int i = 0; i < 32; i++) { 
      lua_pushtableinumber(L, i+1, payload->channels[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    return;
    }
  case FASTMAVLINK_MSG_ID_RADIO_LINK_STATS_DEV: { // #421
    fmav_radio_link_stats_dev_t* payload = (fmav_radio_link_stats_dev_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "flags", payload->flags);
    lua_pushtablenumber(L, "rx_LQ_rc", payload->rx_LQ_rc);
    lua_pushtablenumber(L, "rx_LQ_ser", payload->rx_LQ_ser);
    lua_pushtablenumber(L, "rx_rssi1", payload->rx_rssi1);
    lua_pushtablenumber(L, "rx_snr1", payload->rx_snr1);
    lua_pushtablenumber(L, "rx_rssi2", payload->rx_rssi2);
    lua_pushtablenumber(L, "rx_snr2", payload->rx_snr2);
    lua_pushtablenumber(L, "rx_receive_antenna", payload->rx_receive_antenna);
    lua_pushtablenumber(L, "rx_transmit_antenna", payload->rx_transmit_antenna);
    lua_pushtablenumber(L, "rx_power", payload->rx_power);
    lua_pushtablenumber(L, "tx_LQ_ser", payload->tx_LQ_ser);
    lua_pushtablenumber(L, "tx_rssi1", payload->tx_rssi1);
    lua_pushtablenumber(L, "tx_snr1", payload->tx_snr1);
    lua_pushtablenumber(L, "tx_rssi2", payload->tx_rssi2);
    lua_pushtablenumber(L, "tx_snr2", payload->tx_snr2);
    lua_pushtablenumber(L, "tx_receive_antenna", payload->tx_receive_antenna);
    lua_pushtablenumber(L, "tx_transmit_antenna", payload->tx_transmit_antenna);
    lua_pushtablenumber(L, "tx_power", payload->tx_power);
    return;
    }
  case FASTMAVLINK_MSG_ID_RADIO_LINK_INFORMATION_DEV: { // #422
    fmav_radio_link_information_dev_t* payload = (fmav_radio_link_information_dev_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "type", payload->type);
    lua_pushtablenumber(L, "mode", payload->mode);
    lua_pushtablenumber(L, "tx_rate", payload->tx_rate);
    lua_pushtablenumber(L, "rx_rate", payload->rx_rate);
    lua_pushtablenumber(L, "tx_receive_sensitivity", payload->tx_receive_sensitivity);
    lua_pushtablenumber(L, "rx_receive_sensitivity", payload->rx_receive_sensitivity);
    return;
    }
  case FASTMAVLINK_MSG_ID_WHEEL_DISTANCE: { // #9000
    fmav_wheel_distance_t* payload = (fmav_wheel_distance_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "time_usec", payload->time_usec);
    lua_pushtablenumber(L, "count", payload->count);
    lua_pushstring(L, "distance"); // array distance[16]
    lua_newtable(L);
    for (int i = 0; i < 16; i++) { 
      lua_pushtableinumber(L, i+1, payload->distance[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    return;
    }
  case FASTMAVLINK_MSG_ID_WINCH_STATUS: { // #9005
    fmav_winch_status_t* payload = (fmav_winch_status_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "time_usec", payload->time_usec);
    lua_pushtablenumber(L, "line_length", payload->line_length);
    lua_pushtablenumber(L, "speed", payload->speed);
    lua_pushtablenumber(L, "tension", payload->tension);
    lua_pushtablenumber(L, "voltage", payload->voltage);
    lua_pushtablenumber(L, "current", payload->current);
    lua_pushtablenumber(L, "temperature", payload->temperature);
    lua_pushtablenumber(L, "status", payload->status);
    return;
    }
  case FASTMAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG: { // #10001
    fmav_uavionix_adsb_out_cfg_t* payload = (fmav_uavionix_adsb_out_cfg_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "ICAO", payload->ICAO);
    lua_pushstring(L, "callsign"); // array callsign[9]
    lua_newtable(L);
    for (int i = 0; i < 9; i++) { 
      lua_pushtableinumber(L, i+1, payload->callsign[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushtablenumber(L, "emitterType", payload->emitterType);
    lua_pushtablenumber(L, "aircraftSize", payload->aircraftSize);
    lua_pushtablenumber(L, "gpsOffsetLat", payload->gpsOffsetLat);
    lua_pushtablenumber(L, "gpsOffsetLon", payload->gpsOffsetLon);
    lua_pushtablenumber(L, "stallSpeed", payload->stallSpeed);
    lua_pushtablenumber(L, "rfSelect", payload->rfSelect);
    return;
    }
  case FASTMAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_DYNAMIC: { // #10002
    fmav_uavionix_adsb_out_dynamic_t* payload = (fmav_uavionix_adsb_out_dynamic_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "utcTime", payload->utcTime);
    lua_pushtablenumber(L, "gpsLat", payload->gpsLat);
    lua_pushtablenumber(L, "gpsLon", payload->gpsLon);
    lua_pushtablenumber(L, "gpsAlt", payload->gpsAlt);
    lua_pushtablenumber(L, "gpsFix", payload->gpsFix);
    lua_pushtablenumber(L, "numSats", payload->numSats);
    lua_pushtablenumber(L, "baroAltMSL", payload->baroAltMSL);
    lua_pushtablenumber(L, "accuracyHor", payload->accuracyHor);
    lua_pushtablenumber(L, "accuracyVert", payload->accuracyVert);
    lua_pushtablenumber(L, "accuracyVel", payload->accuracyVel);
    lua_pushtablenumber(L, "velVert", payload->velVert);
    lua_pushtablenumber(L, "velNS", payload->velNS);
    lua_pushtablenumber(L, "VelEW", payload->VelEW);
    lua_pushtablenumber(L, "emergencyStatus", payload->emergencyStatus);
    lua_pushtablenumber(L, "state", payload->state);
    lua_pushtablenumber(L, "squawk", payload->squawk);
    return;
    }
  case FASTMAVLINK_MSG_ID_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT: { // #10003
    fmav_uavionix_adsb_transceiver_health_report_t* payload = (fmav_uavionix_adsb_transceiver_health_report_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "rfHealth", payload->rfHealth);
    return;
    }
  case FASTMAVLINK_MSG_ID_DEVICE_OP_READ: { // #11000
    fmav_device_op_read_t* payload = (fmav_device_op_read_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "request_id", payload->request_id);
    lua_pushtablenumber(L, "bustype", payload->bustype);
    lua_pushtablenumber(L, "bus", payload->bus);
    lua_pushtablenumber(L, "address", payload->address);
    lua_pushstring(L, "busname"); // array busname[40]
    lua_newtable(L);
    for (int i = 0; i < 40; i++) { 
      lua_pushtableinumber(L, i+1, payload->busname[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushtablenumber(L, "regstart", payload->regstart);
    lua_pushtablenumber(L, "count", payload->count);
    lua_pushtablenumber(L, "bank", payload->bank);
    return;
    }
  case FASTMAVLINK_MSG_ID_DEVICE_OP_READ_REPLY: { // #11001
    fmav_device_op_read_reply_t* payload = (fmav_device_op_read_reply_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "request_id", payload->request_id);
    lua_pushtablenumber(L, "result", payload->result);
    lua_pushtablenumber(L, "regstart", payload->regstart);
    lua_pushtablenumber(L, "count", payload->count);
    lua_pushstring(L, "data"); // array data[128]
    lua_newtable(L);
    for (int i = 0; i < 128; i++) { 
      lua_pushtableinumber(L, i+1, payload->data[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushtablenumber(L, "bank", payload->bank);
    return;
    }
  case FASTMAVLINK_MSG_ID_DEVICE_OP_WRITE: { // #11002
    fmav_device_op_write_t* payload = (fmav_device_op_write_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "request_id", payload->request_id);
    lua_pushtablenumber(L, "bustype", payload->bustype);
    lua_pushtablenumber(L, "bus", payload->bus);
    lua_pushtablenumber(L, "address", payload->address);
    lua_pushstring(L, "busname"); // array busname[40]
    lua_newtable(L);
    for (int i = 0; i < 40; i++) { 
      lua_pushtableinumber(L, i+1, payload->busname[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushtablenumber(L, "regstart", payload->regstart);
    lua_pushtablenumber(L, "count", payload->count);
    lua_pushstring(L, "data"); // array data[128]
    lua_newtable(L);
    for (int i = 0; i < 128; i++) { 
      lua_pushtableinumber(L, i+1, payload->data[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushtablenumber(L, "bank", payload->bank);
    return;
    }
  case FASTMAVLINK_MSG_ID_DEVICE_OP_WRITE_REPLY: { // #11003
    fmav_device_op_write_reply_t* payload = (fmav_device_op_write_reply_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "request_id", payload->request_id);
    lua_pushtablenumber(L, "result", payload->result);
    return;
    }
  case FASTMAVLINK_MSG_ID_ADAP_TUNING: { // #11010
    fmav_adap_tuning_t* payload = (fmav_adap_tuning_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "axis", payload->axis);
    lua_pushtablenumber(L, "desired", payload->desired);
    lua_pushtablenumber(L, "achieved", payload->achieved);
    lua_pushtablenumber(L, "error", payload->error);
    lua_pushtablenumber(L, "theta", payload->theta);
    lua_pushtablenumber(L, "omega", payload->omega);
    lua_pushtablenumber(L, "sigma", payload->sigma);
    lua_pushtablenumber(L, "theta_dot", payload->theta_dot);
    lua_pushtablenumber(L, "omega_dot", payload->omega_dot);
    lua_pushtablenumber(L, "sigma_dot", payload->sigma_dot);
    lua_pushtablenumber(L, "f", payload->f);
    lua_pushtablenumber(L, "f_dot", payload->f_dot);
    lua_pushtablenumber(L, "u", payload->u);
    return;
    }
  case FASTMAVLINK_MSG_ID_VISION_POSITION_DELTA: { // #11011
    fmav_vision_position_delta_t* payload = (fmav_vision_position_delta_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "time_usec", payload->time_usec);
    lua_pushtablenumber(L, "time_delta_usec", payload->time_delta_usec);
    lua_pushstring(L, "angle_delta"); // array angle_delta[3]
    lua_newtable(L);
    for (int i = 0; i < 3; i++) { 
      lua_pushtableinumber(L, i+1, payload->angle_delta[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushstring(L, "position_delta"); // array position_delta[3]
    lua_newtable(L);
    for (int i = 0; i < 3; i++) { 
      lua_pushtableinumber(L, i+1, payload->position_delta[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushtablenumber(L, "confidence", payload->confidence);
    return;
    }
  case FASTMAVLINK_MSG_ID_AOA_SSA: { // #11020
    fmav_aoa_ssa_t* payload = (fmav_aoa_ssa_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "time_usec", payload->time_usec);
    lua_pushtablenumber(L, "AOA", payload->AOA);
    lua_pushtablenumber(L, "SSA", payload->SSA);
    return;
    }
  case FASTMAVLINK_MSG_ID_ESC_TELEMETRY_1_TO_4: { // #11030
    fmav_esc_telemetry_1_to_4_t* payload = (fmav_esc_telemetry_1_to_4_t*)(mavmsg->payload_ptr);
    lua_pushstring(L, "temperature"); // array temperature[4]
    lua_newtable(L);
    for (int i = 0; i < 4; i++) { 
      lua_pushtableinumber(L, i+1, payload->temperature[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushstring(L, "voltage"); // array voltage[4]
    lua_newtable(L);
    for (int i = 0; i < 4; i++) { 
      lua_pushtableinumber(L, i+1, payload->voltage[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushstring(L, "current"); // array current[4]
    lua_newtable(L);
    for (int i = 0; i < 4; i++) { 
      lua_pushtableinumber(L, i+1, payload->current[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushstring(L, "totalcurrent"); // array totalcurrent[4]
    lua_newtable(L);
    for (int i = 0; i < 4; i++) { 
      lua_pushtableinumber(L, i+1, payload->totalcurrent[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushstring(L, "rpm"); // array rpm[4]
    lua_newtable(L);
    for (int i = 0; i < 4; i++) { 
      lua_pushtableinumber(L, i+1, payload->rpm[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushstring(L, "count"); // array count[4]
    lua_newtable(L);
    for (int i = 0; i < 4; i++) { 
      lua_pushtableinumber(L, i+1, payload->count[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    return;
    }
  case FASTMAVLINK_MSG_ID_ESC_TELEMETRY_5_TO_8: { // #11031
    fmav_esc_telemetry_5_to_8_t* payload = (fmav_esc_telemetry_5_to_8_t*)(mavmsg->payload_ptr);
    lua_pushstring(L, "temperature"); // array temperature[4]
    lua_newtable(L);
    for (int i = 0; i < 4; i++) { 
      lua_pushtableinumber(L, i+1, payload->temperature[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushstring(L, "voltage"); // array voltage[4]
    lua_newtable(L);
    for (int i = 0; i < 4; i++) { 
      lua_pushtableinumber(L, i+1, payload->voltage[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushstring(L, "current"); // array current[4]
    lua_newtable(L);
    for (int i = 0; i < 4; i++) { 
      lua_pushtableinumber(L, i+1, payload->current[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushstring(L, "totalcurrent"); // array totalcurrent[4]
    lua_newtable(L);
    for (int i = 0; i < 4; i++) { 
      lua_pushtableinumber(L, i+1, payload->totalcurrent[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushstring(L, "rpm"); // array rpm[4]
    lua_newtable(L);
    for (int i = 0; i < 4; i++) { 
      lua_pushtableinumber(L, i+1, payload->rpm[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushstring(L, "count"); // array count[4]
    lua_newtable(L);
    for (int i = 0; i < 4; i++) { 
      lua_pushtableinumber(L, i+1, payload->count[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    return;
    }
  case FASTMAVLINK_MSG_ID_ESC_TELEMETRY_9_TO_12: { // #11032
    fmav_esc_telemetry_9_to_12_t* payload = (fmav_esc_telemetry_9_to_12_t*)(mavmsg->payload_ptr);
    lua_pushstring(L, "temperature"); // array temperature[4]
    lua_newtable(L);
    for (int i = 0; i < 4; i++) { 
      lua_pushtableinumber(L, i+1, payload->temperature[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushstring(L, "voltage"); // array voltage[4]
    lua_newtable(L);
    for (int i = 0; i < 4; i++) { 
      lua_pushtableinumber(L, i+1, payload->voltage[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushstring(L, "current"); // array current[4]
    lua_newtable(L);
    for (int i = 0; i < 4; i++) { 
      lua_pushtableinumber(L, i+1, payload->current[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushstring(L, "totalcurrent"); // array totalcurrent[4]
    lua_newtable(L);
    for (int i = 0; i < 4; i++) { 
      lua_pushtableinumber(L, i+1, payload->totalcurrent[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushstring(L, "rpm"); // array rpm[4]
    lua_newtable(L);
    for (int i = 0; i < 4; i++) { 
      lua_pushtableinumber(L, i+1, payload->rpm[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushstring(L, "count"); // array count[4]
    lua_newtable(L);
    for (int i = 0; i < 4; i++) { 
      lua_pushtableinumber(L, i+1, payload->count[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    return;
    }
  case FASTMAVLINK_MSG_ID_OSD_PARAM_CONFIG: { // #11033
    fmav_osd_param_config_t* payload = (fmav_osd_param_config_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "request_id", payload->request_id);
    lua_pushtablenumber(L, "osd_screen", payload->osd_screen);
    lua_pushtablenumber(L, "osd_index", payload->osd_index);
    lua_pushstring(L, "param_id"); // array param_id[16]
    lua_newtable(L);
    for (int i = 0; i < 16; i++) { 
      lua_pushtableinumber(L, i+1, payload->param_id[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushtablenumber(L, "config_type", payload->config_type);
    lua_pushtablenumber(L, "min_value", payload->min_value);
    lua_pushtablenumber(L, "max_value", payload->max_value);
    lua_pushtablenumber(L, "increment", payload->increment);
    return;
    }
  case FASTMAVLINK_MSG_ID_OSD_PARAM_CONFIG_REPLY: { // #11034
    fmav_osd_param_config_reply_t* payload = (fmav_osd_param_config_reply_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "request_id", payload->request_id);
    lua_pushtablenumber(L, "result", payload->result);
    return;
    }
  case FASTMAVLINK_MSG_ID_OSD_PARAM_SHOW_CONFIG: { // #11035
    fmav_osd_param_show_config_t* payload = (fmav_osd_param_show_config_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "request_id", payload->request_id);
    lua_pushtablenumber(L, "osd_screen", payload->osd_screen);
    lua_pushtablenumber(L, "osd_index", payload->osd_index);
    return;
    }
  case FASTMAVLINK_MSG_ID_OSD_PARAM_SHOW_CONFIG_REPLY: { // #11036
    fmav_osd_param_show_config_reply_t* payload = (fmav_osd_param_show_config_reply_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "request_id", payload->request_id);
    lua_pushtablenumber(L, "result", payload->result);
    lua_pushstring(L, "param_id"); // array param_id[16]
    lua_newtable(L);
    for (int i = 0; i < 16; i++) { 
      lua_pushtableinumber(L, i+1, payload->param_id[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushtablenumber(L, "config_type", payload->config_type);
    lua_pushtablenumber(L, "min_value", payload->min_value);
    lua_pushtablenumber(L, "max_value", payload->max_value);
    lua_pushtablenumber(L, "increment", payload->increment);
    return;
    }
  case FASTMAVLINK_MSG_ID_OBSTACLE_DISTANCE_3D: { // #11037
    fmav_obstacle_distance_3d_t* payload = (fmav_obstacle_distance_3d_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "time_boot_ms", payload->time_boot_ms);
    lua_pushtablenumber(L, "sensor_type", payload->sensor_type);
    lua_pushtablenumber(L, "frame", payload->frame);
    lua_pushtablenumber(L, "obstacle_id", payload->obstacle_id);
    lua_pushtablenumber(L, "x", payload->x);
    lua_pushtablenumber(L, "y", payload->y);
    lua_pushtablenumber(L, "z", payload->z);
    lua_pushtablenumber(L, "min_distance", payload->min_distance);
    lua_pushtablenumber(L, "max_distance", payload->max_distance);
    return;
    }
  case FASTMAVLINK_MSG_ID_WATER_DEPTH: { // #11038
    fmav_water_depth_t* payload = (fmav_water_depth_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "time_boot_ms", payload->time_boot_ms);
    lua_pushtablenumber(L, "id", payload->id);
    lua_pushtablenumber(L, "healthy", payload->healthy);
    lua_pushtablenumber(L, "lat", payload->lat);
    lua_pushtablenumber(L, "lng", payload->lng);
    lua_pushtablenumber(L, "alt", payload->alt);
    lua_pushtablenumber(L, "roll", payload->roll);
    lua_pushtablenumber(L, "pitch", payload->pitch);
    lua_pushtablenumber(L, "yaw", payload->yaw);
    lua_pushtablenumber(L, "distance", payload->distance);
    lua_pushtablenumber(L, "temperature", payload->temperature);
    return;
    }
  case FASTMAVLINK_MSG_ID_MCU_STATUS: { // #11039
    fmav_mcu_status_t* payload = (fmav_mcu_status_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "id", payload->id);
    lua_pushtablenumber(L, "MCU_temperature", payload->MCU_temperature);
    lua_pushtablenumber(L, "MCU_voltage", payload->MCU_voltage);
    lua_pushtablenumber(L, "MCU_voltage_min", payload->MCU_voltage_min);
    lua_pushtablenumber(L, "MCU_voltage_max", payload->MCU_voltage_max);
    return;
    }
  case FASTMAVLINK_MSG_ID_OPEN_DRONE_ID_BASIC_ID: { // #12900
    fmav_open_drone_id_basic_id_t* payload = (fmav_open_drone_id_basic_id_t*)(mavmsg->payload_ptr);
    lua_pushstring(L, "id_or_mac"); // array id_or_mac[20]
    lua_newtable(L);
    for (int i = 0; i < 20; i++) { 
      lua_pushtableinumber(L, i+1, payload->id_or_mac[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushtablenumber(L, "id_type", payload->id_type);
    lua_pushtablenumber(L, "ua_type", payload->ua_type);
    lua_pushstring(L, "uas_id"); // array uas_id[20]
    lua_newtable(L);
    for (int i = 0; i < 20; i++) { 
      lua_pushtableinumber(L, i+1, payload->uas_id[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    return;
    }
  case FASTMAVLINK_MSG_ID_OPEN_DRONE_ID_LOCATION: { // #12901
    fmav_open_drone_id_location_t* payload = (fmav_open_drone_id_location_t*)(mavmsg->payload_ptr);
    lua_pushstring(L, "id_or_mac"); // array id_or_mac[20]
    lua_newtable(L);
    for (int i = 0; i < 20; i++) { 
      lua_pushtableinumber(L, i+1, payload->id_or_mac[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushtablenumber(L, "status", payload->status);
    lua_pushtablenumber(L, "direction", payload->direction);
    lua_pushtablenumber(L, "speed_horizontal", payload->speed_horizontal);
    lua_pushtablenumber(L, "speed_vertical", payload->speed_vertical);
    lua_pushtablenumber(L, "latitude", payload->latitude);
    lua_pushtablenumber(L, "longitude", payload->longitude);
    lua_pushtablenumber(L, "altitude_barometric", payload->altitude_barometric);
    lua_pushtablenumber(L, "altitude_geodetic", payload->altitude_geodetic);
    lua_pushtablenumber(L, "height_reference", payload->height_reference);
    lua_pushtablenumber(L, "height", payload->height);
    lua_pushtablenumber(L, "horizontal_accuracy", payload->horizontal_accuracy);
    lua_pushtablenumber(L, "vertical_accuracy", payload->vertical_accuracy);
    lua_pushtablenumber(L, "barometer_accuracy", payload->barometer_accuracy);
    lua_pushtablenumber(L, "speed_accuracy", payload->speed_accuracy);
    lua_pushtablenumber(L, "timestamp", payload->timestamp);
    lua_pushtablenumber(L, "timestamp_accuracy", payload->timestamp_accuracy);
    return;
    }
  case FASTMAVLINK_MSG_ID_OPEN_DRONE_ID_AUTHENTICATION: { // #12902
    fmav_open_drone_id_authentication_t* payload = (fmav_open_drone_id_authentication_t*)(mavmsg->payload_ptr);
    lua_pushstring(L, "id_or_mac"); // array id_or_mac[20]
    lua_newtable(L);
    for (int i = 0; i < 20; i++) { 
      lua_pushtableinumber(L, i+1, payload->id_or_mac[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushtablenumber(L, "authentication_type", payload->authentication_type);
    lua_pushtablenumber(L, "data_page", payload->data_page);
    lua_pushtablenumber(L, "last_page_index", payload->last_page_index);
    lua_pushtablenumber(L, "length", payload->length);
    lua_pushtablenumber(L, "timestamp", payload->timestamp);
    lua_pushstring(L, "authentication_data"); // array authentication_data[23]
    lua_newtable(L);
    for (int i = 0; i < 23; i++) { 
      lua_pushtableinumber(L, i+1, payload->authentication_data[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    return;
    }
  case FASTMAVLINK_MSG_ID_OPEN_DRONE_ID_SELF_ID: { // #12903
    fmav_open_drone_id_self_id_t* payload = (fmav_open_drone_id_self_id_t*)(mavmsg->payload_ptr);
    lua_pushstring(L, "id_or_mac"); // array id_or_mac[20]
    lua_newtable(L);
    for (int i = 0; i < 20; i++) { 
      lua_pushtableinumber(L, i+1, payload->id_or_mac[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushtablenumber(L, "description_type", payload->description_type);
    lua_pushstring(L, "description"); // array description[23]
    lua_newtable(L);
    for (int i = 0; i < 23; i++) { 
      lua_pushtableinumber(L, i+1, payload->description[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    return;
    }
  case FASTMAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM: { // #12904
    fmav_open_drone_id_system_t* payload = (fmav_open_drone_id_system_t*)(mavmsg->payload_ptr);
    lua_pushstring(L, "id_or_mac"); // array id_or_mac[20]
    lua_newtable(L);
    for (int i = 0; i < 20; i++) { 
      lua_pushtableinumber(L, i+1, payload->id_or_mac[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushtablenumber(L, "operator_location_type", payload->operator_location_type);
    lua_pushtablenumber(L, "classification_type", payload->classification_type);
    lua_pushtablenumber(L, "operator_latitude", payload->operator_latitude);
    lua_pushtablenumber(L, "operator_longitude", payload->operator_longitude);
    lua_pushtablenumber(L, "area_count", payload->area_count);
    lua_pushtablenumber(L, "area_radius", payload->area_radius);
    lua_pushtablenumber(L, "area_ceiling", payload->area_ceiling);
    lua_pushtablenumber(L, "area_floor", payload->area_floor);
    lua_pushtablenumber(L, "category_eu", payload->category_eu);
    lua_pushtablenumber(L, "class_eu", payload->class_eu);
    lua_pushtablenumber(L, "operator_altitude_geo", payload->operator_altitude_geo);
    lua_pushtablenumber(L, "timestamp", payload->timestamp);
    return;
    }
  case FASTMAVLINK_MSG_ID_OPEN_DRONE_ID_OPERATOR_ID: { // #12905
    fmav_open_drone_id_operator_id_t* payload = (fmav_open_drone_id_operator_id_t*)(mavmsg->payload_ptr);
    lua_pushstring(L, "id_or_mac"); // array id_or_mac[20]
    lua_newtable(L);
    for (int i = 0; i < 20; i++) { 
      lua_pushtableinumber(L, i+1, payload->id_or_mac[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushtablenumber(L, "operator_id_type", payload->operator_id_type);
    lua_pushstring(L, "operator_id"); // array operator_id[20]
    lua_newtable(L);
    for (int i = 0; i < 20; i++) { 
      lua_pushtableinumber(L, i+1, payload->operator_id[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    return;
    }
  case FASTMAVLINK_MSG_ID_OPEN_DRONE_ID_MESSAGE_PACK: { // #12915
    fmav_open_drone_id_message_pack_t* payload = (fmav_open_drone_id_message_pack_t*)(mavmsg->payload_ptr);
    lua_pushstring(L, "id_or_mac"); // array id_or_mac[20]
    lua_newtable(L);
    for (int i = 0; i < 20; i++) { 
      lua_pushtableinumber(L, i+1, payload->id_or_mac[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushtablenumber(L, "single_message_size", payload->single_message_size);
    lua_pushtablenumber(L, "msg_pack_size", payload->msg_pack_size);
    lua_pushstring(L, "messages"); // array messages[225]
    lua_newtable(L);
    for (int i = 0; i < 225; i++) { 
      lua_pushtableinumber(L, i+1, payload->messages[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    return;
    }
  case FASTMAVLINK_MSG_ID_OPEN_DRONE_ID_ARM_STATUS: { // #12918
    fmav_open_drone_id_arm_status_t* payload = (fmav_open_drone_id_arm_status_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "status", payload->status);
    lua_pushstring(L, "error"); // array error[50]
    lua_newtable(L);
    for (int i = 0; i < 50; i++) { 
      lua_pushtableinumber(L, i+1, payload->error[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    return;
    }
  case FASTMAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM_UPDATE: { // #12919
    fmav_open_drone_id_system_update_t* payload = (fmav_open_drone_id_system_update_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "operator_latitude", payload->operator_latitude);
    lua_pushtablenumber(L, "operator_longitude", payload->operator_longitude);
    lua_pushtablenumber(L, "operator_altitude_geo", payload->operator_altitude_geo);
    lua_pushtablenumber(L, "timestamp", payload->timestamp);
    return;
    }
  case FASTMAVLINK_MSG_ID_HYGROMETER_SENSOR: { // #12920
    fmav_hygrometer_sensor_t* payload = (fmav_hygrometer_sensor_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "id", payload->id);
    lua_pushtablenumber(L, "temperature", payload->temperature);
    lua_pushtablenumber(L, "humidity", payload->humidity);
    return;
    }
  case FASTMAVLINK_MSG_ID_ICAROUS_HEARTBEAT: { // #42000
    fmav_icarous_heartbeat_t* payload = (fmav_icarous_heartbeat_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "status", payload->status);
    return;
    }
  case FASTMAVLINK_MSG_ID_ICAROUS_KINEMATIC_BANDS: { // #42001
    fmav_icarous_kinematic_bands_t* payload = (fmav_icarous_kinematic_bands_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "numBands", payload->numBands);
    lua_pushtablenumber(L, "type1", payload->type1);
    lua_pushtablenumber(L, "min1", payload->min1);
    lua_pushtablenumber(L, "max1", payload->max1);
    lua_pushtablenumber(L, "type2", payload->type2);
    lua_pushtablenumber(L, "min2", payload->min2);
    lua_pushtablenumber(L, "max2", payload->max2);
    lua_pushtablenumber(L, "type3", payload->type3);
    lua_pushtablenumber(L, "min3", payload->min3);
    lua_pushtablenumber(L, "max3", payload->max3);
    lua_pushtablenumber(L, "type4", payload->type4);
    lua_pushtablenumber(L, "min4", payload->min4);
    lua_pushtablenumber(L, "max4", payload->max4);
    lua_pushtablenumber(L, "type5", payload->type5);
    lua_pushtablenumber(L, "min5", payload->min5);
    lua_pushtablenumber(L, "max5", payload->max5);
    return;
    }
  case FASTMAVLINK_MSG_ID_CUBEPILOT_RAW_RC: { // #50001
    fmav_cubepilot_raw_rc_t* payload = (fmav_cubepilot_raw_rc_t*)(mavmsg->payload_ptr);
    lua_pushstring(L, "rc_raw"); // array rc_raw[32]
    lua_newtable(L);
    for (int i = 0; i < 32; i++) { 
      lua_pushtableinumber(L, i+1, payload->rc_raw[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    return;
    }
  case FASTMAVLINK_MSG_ID_HERELINK_VIDEO_STREAM_INFORMATION: { // #50002
    fmav_herelink_video_stream_information_t* payload = (fmav_herelink_video_stream_information_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "camera_id", payload->camera_id);
    lua_pushtablenumber(L, "status", payload->status);
    lua_pushtablenumber(L, "framerate", payload->framerate);
    lua_pushtablenumber(L, "resolution_h", payload->resolution_h);
    lua_pushtablenumber(L, "resolution_v", payload->resolution_v);
    lua_pushtablenumber(L, "bitrate", payload->bitrate);
    lua_pushtablenumber(L, "rotation", payload->rotation);
    lua_pushstring(L, "uri"); // array uri[230]
    lua_newtable(L);
    for (int i = 0; i < 230; i++) { 
      lua_pushtableinumber(L, i+1, payload->uri[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    return;
    }
  case FASTMAVLINK_MSG_ID_HERELINK_TELEM: { // #50003
    fmav_herelink_telem_t* payload = (fmav_herelink_telem_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "rssi", payload->rssi);
    lua_pushtablenumber(L, "snr", payload->snr);
    lua_pushtablenumber(L, "rf_freq", payload->rf_freq);
    lua_pushtablenumber(L, "link_bw", payload->link_bw);
    lua_pushtablenumber(L, "link_rate", payload->link_rate);
    lua_pushtablenumber(L, "cpu_temp", payload->cpu_temp);
    lua_pushtablenumber(L, "board_temp", payload->board_temp);
    return;
    }
  case FASTMAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_START: { // #50004
    fmav_cubepilot_firmware_update_start_t* payload = (fmav_cubepilot_firmware_update_start_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "size", payload->size);
    lua_pushtablenumber(L, "crc", payload->crc);
    return;
    }
  case FASTMAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_RESP: { // #50005
    fmav_cubepilot_firmware_update_resp_t* payload = (fmav_cubepilot_firmware_update_resp_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "offset", payload->offset);
    return;
    }
  case FASTMAVLINK_MSG_ID_AIRLINK_AUTH: { // #52000
    fmav_airlink_auth_t* payload = (fmav_airlink_auth_t*)(mavmsg->payload_ptr);
    lua_pushstring(L, "login"); // array login[50]
    lua_newtable(L);
    for (int i = 0; i < 50; i++) { 
      lua_pushtableinumber(L, i+1, payload->login[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushstring(L, "password"); // array password[50]
    lua_newtable(L);
    for (int i = 0; i < 50; i++) { 
      lua_pushtableinumber(L, i+1, payload->password[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    return;
    }
  case FASTMAVLINK_MSG_ID_AIRLINK_AUTH_RESPONSE: { // #52001
    fmav_airlink_auth_response_t* payload = (fmav_airlink_auth_response_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "resp_type", payload->resp_type);
    return;
    }
  case FASTMAVLINK_MSG_ID_AIRLINK_EYE_GS_HOLE_PUSH_REQUEST: { // #52002
    fmav_airlink_eye_gs_hole_push_request_t* payload = (fmav_airlink_eye_gs_hole_push_request_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "resp_type", payload->resp_type);
    return;
    }
  case FASTMAVLINK_MSG_ID_AIRLINK_EYE_GS_HOLE_PUSH_RESPONSE: { // #52003
    fmav_airlink_eye_gs_hole_push_response_t* payload = (fmav_airlink_eye_gs_hole_push_response_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "resp_type", payload->resp_type);
    lua_pushtablenumber(L, "ip_version", payload->ip_version);
    lua_pushstring(L, "ip_address_4"); // array ip_address_4[4]
    lua_newtable(L);
    for (int i = 0; i < 4; i++) { 
      lua_pushtableinumber(L, i+1, payload->ip_address_4[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushstring(L, "ip_address_6"); // array ip_address_6[16]
    lua_newtable(L);
    for (int i = 0; i < 16; i++) { 
      lua_pushtableinumber(L, i+1, payload->ip_address_6[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushtablenumber(L, "ip_port", payload->ip_port);
    return;
    }
  case FASTMAVLINK_MSG_ID_AIRLINK_EYE_HP: { // #52004
    fmav_airlink_eye_hp_t* payload = (fmav_airlink_eye_hp_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "resp_type", payload->resp_type);
    return;
    }
  case FASTMAVLINK_MSG_ID_AIRLINK_EYE_TURN_INIT: { // #52005
    fmav_airlink_eye_turn_init_t* payload = (fmav_airlink_eye_turn_init_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "resp_type", payload->resp_type);
    return;
    }
  case FASTMAVLINK_MSG_ID_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_EXT: { // #60000
    fmav_autopilot_state_for_gimbal_device_ext_t* payload = (fmav_autopilot_state_for_gimbal_device_ext_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "time_boot_us", payload->time_boot_us);
    lua_pushtablenumber(L, "wind_x", payload->wind_x);
    lua_pushtablenumber(L, "wind_y", payload->wind_y);
    lua_pushtablenumber(L, "wind_correction_angle", payload->wind_correction_angle);
    return;
    }
  case FASTMAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_INFORMATION: { // #60010
    fmav_storm32_gimbal_manager_information_t* payload = (fmav_storm32_gimbal_manager_information_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "gimbal_id", payload->gimbal_id);
    lua_pushtablenumber(L, "device_cap_flags", payload->device_cap_flags);
    lua_pushtablenumber(L, "manager_cap_flags", payload->manager_cap_flags);
    lua_pushtablenumber(L, "roll_min", payload->roll_min);
    lua_pushtablenumber(L, "roll_max", payload->roll_max);
    lua_pushtablenumber(L, "pitch_min", payload->pitch_min);
    lua_pushtablenumber(L, "pitch_max", payload->pitch_max);
    lua_pushtablenumber(L, "yaw_min", payload->yaw_min);
    lua_pushtablenumber(L, "yaw_max", payload->yaw_max);
    return;
    }
  case FASTMAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_STATUS: { // #60011
    fmav_storm32_gimbal_manager_status_t* payload = (fmav_storm32_gimbal_manager_status_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "gimbal_id", payload->gimbal_id);
    lua_pushtablenumber(L, "supervisor", payload->supervisor);
    lua_pushtablenumber(L, "device_flags", payload->device_flags);
    lua_pushtablenumber(L, "manager_flags", payload->manager_flags);
    lua_pushtablenumber(L, "profile", payload->profile);
    return;
    }
  case FASTMAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CONTROL: { // #60012
    fmav_storm32_gimbal_manager_control_t* payload = (fmav_storm32_gimbal_manager_control_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "gimbal_id", payload->gimbal_id);
    lua_pushtablenumber(L, "client", payload->client);
    lua_pushtablenumber(L, "device_flags", payload->device_flags);
    lua_pushtablenumber(L, "manager_flags", payload->manager_flags);
    lua_pushstring(L, "q"); // array q[4]
    lua_newtable(L);
    for (int i = 0; i < 4; i++) { 
      lua_pushtableinumber(L, i+1, payload->q[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    lua_pushtablenumber(L, "angular_velocity_x", payload->angular_velocity_x);
    lua_pushtablenumber(L, "angular_velocity_y", payload->angular_velocity_y);
    lua_pushtablenumber(L, "angular_velocity_z", payload->angular_velocity_z);
    return;
    }
  case FASTMAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CONTROL_PITCHYAW: { // #60013
    fmav_storm32_gimbal_manager_control_pitchyaw_t* payload = (fmav_storm32_gimbal_manager_control_pitchyaw_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "gimbal_id", payload->gimbal_id);
    lua_pushtablenumber(L, "client", payload->client);
    lua_pushtablenumber(L, "device_flags", payload->device_flags);
    lua_pushtablenumber(L, "manager_flags", payload->manager_flags);
    lua_pushtablenumber(L, "pitch", payload->pitch);
    lua_pushtablenumber(L, "yaw", payload->yaw);
    lua_pushtablenumber(L, "pitch_rate", payload->pitch_rate);
    lua_pushtablenumber(L, "yaw_rate", payload->yaw_rate);
    return;
    }
  case FASTMAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CORRECT_ROLL: { // #60014
    fmav_storm32_gimbal_manager_correct_roll_t* payload = (fmav_storm32_gimbal_manager_correct_roll_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "gimbal_id", payload->gimbal_id);
    lua_pushtablenumber(L, "client", payload->client);
    lua_pushtablenumber(L, "roll", payload->roll);
    return;
    }
  case FASTMAVLINK_MSG_ID_QSHOT_STATUS: { // #60020
    fmav_qshot_status_t* payload = (fmav_qshot_status_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "mode", payload->mode);
    lua_pushtablenumber(L, "shot_state", payload->shot_state);
    return;
    }
  case FASTMAVLINK_MSG_ID_FRSKY_PASSTHROUGH_ARRAY: { // #60040
    fmav_frsky_passthrough_array_t* payload = (fmav_frsky_passthrough_array_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "time_boot_ms", payload->time_boot_ms);
    lua_pushtablenumber(L, "count", payload->count);
    lua_pushstring(L, "packet_buf"); // array packet_buf[240]
    lua_newtable(L);
    for (int i = 0; i < 240; i++) { 
      lua_pushtableinumber(L, i+1, payload->packet_buf[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    return;
    }
  case FASTMAVLINK_MSG_ID_PARAM_VALUE_ARRAY: { // #60041
    fmav_param_value_array_t* payload = (fmav_param_value_array_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "param_count", payload->param_count);
    lua_pushtablenumber(L, "param_index_first", payload->param_index_first);
    lua_pushtablenumber(L, "param_array_len", payload->param_array_len);
    lua_pushtablenumber(L, "flags", payload->flags);
    lua_pushstring(L, "packet_buf"); // array packet_buf[248]
    lua_newtable(L);
    for (int i = 0; i < 248; i++) { 
      lua_pushtableinumber(L, i+1, payload->packet_buf[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    return;
    }
  case FASTMAVLINK_MSG_ID_RADIO_RC_CHANNELS: { // #60045
    fmav_radio_rc_channels_t* payload = (fmav_radio_rc_channels_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "count", payload->count);
    lua_pushtablenumber(L, "flags", payload->flags);
    lua_pushstring(L, "channels"); // array channels[24]
    lua_newtable(L);
    for (int i = 0; i < 24; i++) { 
      lua_pushtableinumber(L, i+1, payload->channels[i]); // lua is 1 indexed
    }
    lua_rawset(L, -3);
    return;
    }
  case FASTMAVLINK_MSG_ID_RADIO_LINK_STATS: { // #60046
    fmav_radio_link_stats_t* payload = (fmav_radio_link_stats_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "flags", payload->flags);
    lua_pushtablenumber(L, "rx_LQ", payload->rx_LQ);
    lua_pushtablenumber(L, "rx_rssi1", payload->rx_rssi1);
    lua_pushtablenumber(L, "rx_snr1", payload->rx_snr1);
    lua_pushtablenumber(L, "rx_rssi2", payload->rx_rssi2);
    lua_pushtablenumber(L, "rx_snr2", payload->rx_snr2);
    lua_pushtablenumber(L, "rx_receive_antenna", payload->rx_receive_antenna);
    lua_pushtablenumber(L, "rx_transmit_antenna", payload->rx_transmit_antenna);
    lua_pushtablenumber(L, "tx_LQ", payload->tx_LQ);
    lua_pushtablenumber(L, "tx_rssi1", payload->tx_rssi1);
    lua_pushtablenumber(L, "tx_snr1", payload->tx_snr1);
    lua_pushtablenumber(L, "tx_rssi2", payload->tx_rssi2);
    lua_pushtablenumber(L, "tx_snr2", payload->tx_snr2);
    lua_pushtablenumber(L, "tx_receive_antenna", payload->tx_receive_antenna);
    lua_pushtablenumber(L, "tx_transmit_antenna", payload->tx_transmit_antenna);
    return;
    }
  case FASTMAVLINK_MSG_ID_RADIO_LINK_FLOW_CONTROL: { // #60047
    fmav_radio_link_flow_control_t* payload = (fmav_radio_link_flow_control_t*)(mavmsg->payload_ptr);
    lua_pushtablenumber(L, "tx_rate", payload->tx_rate);
    lua_pushtablenumber(L, "rx_rate", payload->rx_rate);
    lua_pushtablenumber(L, "tx_used_bandwidth", payload->tx_used_bandwidth);
    lua_pushtablenumber(L, "rx_used_bandwidth", payload->rx_used_bandwidth);
    lua_pushtablenumber(L, "txbuf", payload->txbuf);
    return;
    }
  }
}


//------------------------------------------------------------
// check
//------------------------------------------------------------

#define lua_checktableinteger(L, r, k, d)  (lua_pushstring(L,(k)), lua_gettable(L,-2 ), r = (lua_isnumber(L,-1))?lua_tointeger(L,-1):(d), lua_pop(L,1))
#define lua_checktablenumber(L, r, k, d)   (lua_pushstring(L,(k)), lua_gettable(L,-2 ), r = (lua_isnumber(L,-1))?lua_tonumber(L,-1):(d), lua_pop(L,1))
#define lua_checktableinumber(L, r, i, d)  (lua_pushnumber(L,(i)), lua_gettable(L,-2 ), r = (lua_isnumber(L,-1))?lua_tonumber(L,-1):(d), lua_pop(L,1))


static uint8_t luaMavlinkCheckMsgOut(lua_State *L, fmav_message_t* msg_out)
{
  uint32_t msgid;
  lua_checktableinteger(L, msgid, "msgid", UINT32_MAX);
  if (msgid == UINT32_MAX) return 0;
  
  msg_out->sysid = mavlinkTelem.mySysId();
  msg_out->compid = mavlinkTelem.myCompId();
  msg_out->msgid = msgid;
  msg_out->target_sysid = 0;
  msg_out->target_compid = 0;
  
  switch (msgid) {
  case FASTMAVLINK_MSG_ID_HEARTBEAT: { // #0
    fmav_heartbeat_t* payload = (fmav_heartbeat_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->type, "type", 0);
    lua_checktablenumber(L, payload->autopilot, "autopilot", 0);
    lua_checktablenumber(L, payload->base_mode, "base_mode", 0);
    lua_checktablenumber(L, payload->custom_mode, "custom_mode", 0);
    lua_checktablenumber(L, payload->system_status, "system_status", 0);
    payload->mavlink_version = FASTMAVLINK_MAVLINK_VERSION;
    msg_out->crc_extra = FASTMAVLINK_MSG_HEARTBEAT_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_HEARTBEAT_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_SYS_STATUS: { // #1
    fmav_sys_status_t* payload = (fmav_sys_status_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->onboard_control_sensors_present, "onboard_control_sensors_present", 0);
    lua_checktablenumber(L, payload->onboard_control_sensors_enabled, "onboard_control_sensors_enabled", 0);
    lua_checktablenumber(L, payload->onboard_control_sensors_health, "onboard_control_sensors_health", 0);
    lua_checktablenumber(L, payload->load, "load", 0);
    lua_checktablenumber(L, payload->voltage_battery, "voltage_battery", UINT16_MAX);
    lua_checktablenumber(L, payload->current_battery, "current_battery", -1);
    lua_checktablenumber(L, payload->battery_remaining, "battery_remaining", -1);
    lua_checktablenumber(L, payload->drop_rate_comm, "drop_rate_comm", 0);
    lua_checktablenumber(L, payload->errors_comm, "errors_comm", 0);
    lua_checktablenumber(L, payload->errors_count1, "errors_count1", 0);
    lua_checktablenumber(L, payload->errors_count2, "errors_count2", 0);
    lua_checktablenumber(L, payload->errors_count3, "errors_count3", 0);
    lua_checktablenumber(L, payload->errors_count4, "errors_count4", 0);
    lua_checktablenumber(L, payload->onboard_control_sensors_present_extended, "onboard_con_sen_pre_extended", 0);
    lua_checktablenumber(L, payload->onboard_control_sensors_enabled_extended, "onboard_con_sen_ena_extended", 0);
    lua_checktablenumber(L, payload->onboard_control_sensors_health_extended, "onboard_con_sen_hea_extended", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_SYS_STATUS_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_SYS_STATUS_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_SYSTEM_TIME: { // #2
    fmav_system_time_t* payload = (fmav_system_time_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->time_unix_usec, "time_unix_usec", 0);
    lua_checktablenumber(L, payload->time_boot_ms, "time_boot_ms", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_SYSTEM_TIME_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_SYSTEM_TIME_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_PING: { // #4
    fmav_ping_t* payload = (fmav_ping_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->time_usec, "time_usec", 0);
    lua_checktablenumber(L, payload->seq, "seq", 0);
    lua_checktablenumber(L, payload->target_system, "target_sysid", 0);
    lua_checktablenumber(L, payload->target_component, "target_compid", 0);
    msg_out->target_sysid = payload->target_system;
    msg_out->target_compid = payload->target_component;
    msg_out->crc_extra = FASTMAVLINK_MSG_PING_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_PING_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL: { // #5
    fmav_change_operator_control_t* payload = (fmav_change_operator_control_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->target_system, "target_sysid", 0);
    lua_checktablenumber(L, payload->control_request, "control_request", 0);
    lua_checktablenumber(L, payload->version, "version", 0);
    lua_pushstring(L, "passkey"); // array passkey[25]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 25; i++) { 
        lua_checktableinumber(L, payload->passkey[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    msg_out->target_sysid = payload->target_system;
    msg_out->crc_extra = FASTMAVLINK_MSG_CHANGE_OPERATOR_CONTROL_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_CHANGE_OPERATOR_CONTROL_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL_ACK: { // #6
    fmav_change_operator_control_ack_t* payload = (fmav_change_operator_control_ack_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->gcs_system_id, "gcs_system_id", 0);
    lua_checktablenumber(L, payload->control_request, "control_request", 0);
    lua_checktablenumber(L, payload->ack, "ack", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_CHANGE_OPERATOR_CONTROL_ACK_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_CHANGE_OPERATOR_CONTROL_ACK_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_AUTH_KEY: { // #7
    fmav_auth_key_t* payload = (fmav_auth_key_t*)(msg_out->payload);
    lua_pushstring(L, "key"); // array key[32]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 32; i++) { 
        lua_checktableinumber(L, payload->key[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    msg_out->crc_extra = FASTMAVLINK_MSG_AUTH_KEY_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_AUTH_KEY_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_LINK_NODE_STATUS: { // #8
    fmav_link_node_status_t* payload = (fmav_link_node_status_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->timestamp, "timestamp", 0);
    lua_checktablenumber(L, payload->tx_buf, "tx_buf", 0);
    lua_checktablenumber(L, payload->rx_buf, "rx_buf", 0);
    lua_checktablenumber(L, payload->tx_rate, "tx_rate", 0);
    lua_checktablenumber(L, payload->rx_rate, "rx_rate", 0);
    lua_checktablenumber(L, payload->rx_parse_err, "rx_parse_err", 0);
    lua_checktablenumber(L, payload->tx_overflows, "tx_overflows", 0);
    lua_checktablenumber(L, payload->rx_overflows, "rx_overflows", 0);
    lua_checktablenumber(L, payload->messages_sent, "messages_sent", 0);
    lua_checktablenumber(L, payload->messages_received, "messages_received", 0);
    lua_checktablenumber(L, payload->messages_lost, "messages_lost", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_LINK_NODE_STATUS_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_LINK_NODE_STATUS_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_SET_MODE: { // #11
    fmav_set_mode_t* payload = (fmav_set_mode_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->target_system, "target_sysid", 0);
    lua_checktablenumber(L, payload->base_mode, "base_mode", 0);
    lua_checktablenumber(L, payload->custom_mode, "custom_mode", 0);
    msg_out->target_sysid = payload->target_system;
    msg_out->crc_extra = FASTMAVLINK_MSG_SET_MODE_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_SET_MODE_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_PARAM_REQUEST_READ: { // #20
    fmav_param_request_read_t* payload = (fmav_param_request_read_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->target_system, "target_sysid", 0);
    lua_checktablenumber(L, payload->target_component, "target_compid", 0);
    lua_pushstring(L, "param_id"); // array param_id[16]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 16; i++) { 
        lua_checktableinumber(L, payload->param_id[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_checktablenumber(L, payload->param_index, "param_index", -1);
    msg_out->target_sysid = payload->target_system;
    msg_out->target_compid = payload->target_component;
    msg_out->crc_extra = FASTMAVLINK_MSG_PARAM_REQUEST_READ_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_PARAM_REQUEST_READ_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_PARAM_REQUEST_LIST: { // #21
    fmav_param_request_list_t* payload = (fmav_param_request_list_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->target_system, "target_sysid", 0);
    lua_checktablenumber(L, payload->target_component, "target_compid", 0);
    msg_out->target_sysid = payload->target_system;
    msg_out->target_compid = payload->target_component;
    msg_out->crc_extra = FASTMAVLINK_MSG_PARAM_REQUEST_LIST_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_PARAM_REQUEST_LIST_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_PARAM_VALUE: { // #22
    fmav_param_value_t* payload = (fmav_param_value_t*)(msg_out->payload);
    lua_pushstring(L, "param_id"); // array param_id[16]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 16; i++) { 
        lua_checktableinumber(L, payload->param_id[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_checktablenumber(L, payload->param_value, "param_value", 0);
    lua_checktablenumber(L, payload->param_type, "param_type", 0);
    lua_checktablenumber(L, payload->param_count, "param_count", 0);
    lua_checktablenumber(L, payload->param_index, "param_index", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_PARAM_VALUE_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_PARAM_VALUE_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_PARAM_SET: { // #23
    fmav_param_set_t* payload = (fmav_param_set_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->target_system, "target_sysid", 0);
    lua_checktablenumber(L, payload->target_component, "target_compid", 0);
    lua_pushstring(L, "param_id"); // array param_id[16]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 16; i++) { 
        lua_checktableinumber(L, payload->param_id[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_checktablenumber(L, payload->param_value, "param_value", 0);
    lua_checktablenumber(L, payload->param_type, "param_type", 0);
    msg_out->target_sysid = payload->target_system;
    msg_out->target_compid = payload->target_component;
    msg_out->crc_extra = FASTMAVLINK_MSG_PARAM_SET_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_PARAM_SET_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_GPS_RAW_INT: { // #24
    fmav_gps_raw_int_t* payload = (fmav_gps_raw_int_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->time_usec, "time_usec", 0);
    lua_checktablenumber(L, payload->fix_type, "fix_type", 0);
    lua_checktablenumber(L, payload->lat, "lat", 0);
    lua_checktablenumber(L, payload->lon, "lon", 0);
    lua_checktablenumber(L, payload->alt, "alt", 0);
    lua_checktablenumber(L, payload->eph, "eph", UINT16_MAX);
    lua_checktablenumber(L, payload->epv, "epv", UINT16_MAX);
    lua_checktablenumber(L, payload->vel, "vel", UINT16_MAX);
    lua_checktablenumber(L, payload->cog, "cog", UINT16_MAX);
    lua_checktablenumber(L, payload->satellites_visible, "satellites_visible", UINT8_MAX);
    lua_checktablenumber(L, payload->alt_ellipsoid, "alt_ellipsoid", 0);
    lua_checktablenumber(L, payload->h_acc, "h_acc", 0);
    lua_checktablenumber(L, payload->v_acc, "v_acc", 0);
    lua_checktablenumber(L, payload->vel_acc, "vel_acc", 0);
    lua_checktablenumber(L, payload->hdg_acc, "hdg_acc", 0);
    lua_checktablenumber(L, payload->yaw, "yaw", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_GPS_RAW_INT_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_GPS_RAW_INT_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_GPS_STATUS: { // #25
    fmav_gps_status_t* payload = (fmav_gps_status_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->satellites_visible, "satellites_visible", 0);
    lua_pushstring(L, "satellite_prn"); // array satellite_prn[20]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 20; i++) { 
        lua_checktableinumber(L, payload->satellite_prn[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_pushstring(L, "satellite_used"); // array satellite_used[20]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 20; i++) { 
        lua_checktableinumber(L, payload->satellite_used[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_pushstring(L, "satellite_elevation"); // array satellite_elevation[20]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 20; i++) { 
        lua_checktableinumber(L, payload->satellite_elevation[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_pushstring(L, "satellite_azimuth"); // array satellite_azimuth[20]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 20; i++) { 
        lua_checktableinumber(L, payload->satellite_azimuth[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_pushstring(L, "satellite_snr"); // array satellite_snr[20]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 20; i++) { 
        lua_checktableinumber(L, payload->satellite_snr[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    msg_out->crc_extra = FASTMAVLINK_MSG_GPS_STATUS_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_GPS_STATUS_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_SCALED_IMU: { // #26
    fmav_scaled_imu_t* payload = (fmav_scaled_imu_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->time_boot_ms, "time_boot_ms", 0);
    lua_checktablenumber(L, payload->xacc, "xacc", 0);
    lua_checktablenumber(L, payload->yacc, "yacc", 0);
    lua_checktablenumber(L, payload->zacc, "zacc", 0);
    lua_checktablenumber(L, payload->xgyro, "xgyro", 0);
    lua_checktablenumber(L, payload->ygyro, "ygyro", 0);
    lua_checktablenumber(L, payload->zgyro, "zgyro", 0);
    lua_checktablenumber(L, payload->xmag, "xmag", 0);
    lua_checktablenumber(L, payload->ymag, "ymag", 0);
    lua_checktablenumber(L, payload->zmag, "zmag", 0);
    lua_checktablenumber(L, payload->temperature, "temperature", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_SCALED_IMU_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_SCALED_IMU_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_RAW_IMU: { // #27
    fmav_raw_imu_t* payload = (fmav_raw_imu_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->time_usec, "time_usec", 0);
    lua_checktablenumber(L, payload->xacc, "xacc", 0);
    lua_checktablenumber(L, payload->yacc, "yacc", 0);
    lua_checktablenumber(L, payload->zacc, "zacc", 0);
    lua_checktablenumber(L, payload->xgyro, "xgyro", 0);
    lua_checktablenumber(L, payload->ygyro, "ygyro", 0);
    lua_checktablenumber(L, payload->zgyro, "zgyro", 0);
    lua_checktablenumber(L, payload->xmag, "xmag", 0);
    lua_checktablenumber(L, payload->ymag, "ymag", 0);
    lua_checktablenumber(L, payload->zmag, "zmag", 0);
    lua_checktablenumber(L, payload->id, "id", 0);
    lua_checktablenumber(L, payload->temperature, "temperature", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_RAW_IMU_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_RAW_IMU_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_RAW_PRESSURE: { // #28
    fmav_raw_pressure_t* payload = (fmav_raw_pressure_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->time_usec, "time_usec", 0);
    lua_checktablenumber(L, payload->press_abs, "press_abs", 0);
    lua_checktablenumber(L, payload->press_diff1, "press_diff1", 0);
    lua_checktablenumber(L, payload->press_diff2, "press_diff2", 0);
    lua_checktablenumber(L, payload->temperature, "temperature", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_RAW_PRESSURE_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_RAW_PRESSURE_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_SCALED_PRESSURE: { // #29
    fmav_scaled_pressure_t* payload = (fmav_scaled_pressure_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->time_boot_ms, "time_boot_ms", 0);
    lua_checktablenumber(L, payload->press_abs, "press_abs", 0);
    lua_checktablenumber(L, payload->press_diff, "press_diff", 0);
    lua_checktablenumber(L, payload->temperature, "temperature", 0);
    lua_checktablenumber(L, payload->temperature_press_diff, "temperature_press_diff", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_SCALED_PRESSURE_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_SCALED_PRESSURE_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_ATTITUDE: { // #30
    fmav_attitude_t* payload = (fmav_attitude_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->time_boot_ms, "time_boot_ms", 0);
    lua_checktablenumber(L, payload->roll, "roll", 0);
    lua_checktablenumber(L, payload->pitch, "pitch", 0);
    lua_checktablenumber(L, payload->yaw, "yaw", 0);
    lua_checktablenumber(L, payload->rollspeed, "rollspeed", 0);
    lua_checktablenumber(L, payload->pitchspeed, "pitchspeed", 0);
    lua_checktablenumber(L, payload->yawspeed, "yawspeed", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_ATTITUDE_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_ATTITUDE_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_ATTITUDE_QUATERNION: { // #31
    fmav_attitude_quaternion_t* payload = (fmav_attitude_quaternion_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->time_boot_ms, "time_boot_ms", 0);
    lua_checktablenumber(L, payload->q1, "q1", 0);
    lua_checktablenumber(L, payload->q2, "q2", 0);
    lua_checktablenumber(L, payload->q3, "q3", 0);
    lua_checktablenumber(L, payload->q4, "q4", 0);
    lua_checktablenumber(L, payload->rollspeed, "rollspeed", 0);
    lua_checktablenumber(L, payload->pitchspeed, "pitchspeed", 0);
    lua_checktablenumber(L, payload->yawspeed, "yawspeed", 0);
    lua_pushstring(L, "repr_offset_q"); // array repr_offset_q[4]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 4; i++) { 
        lua_checktableinumber(L, payload->repr_offset_q[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    msg_out->crc_extra = FASTMAVLINK_MSG_ATTITUDE_QUATERNION_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_ATTITUDE_QUATERNION_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_LOCAL_POSITION_NED: { // #32
    fmav_local_position_ned_t* payload = (fmav_local_position_ned_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->time_boot_ms, "time_boot_ms", 0);
    lua_checktablenumber(L, payload->x, "x", 0);
    lua_checktablenumber(L, payload->y, "y", 0);
    lua_checktablenumber(L, payload->z, "z", 0);
    lua_checktablenumber(L, payload->vx, "vx", 0);
    lua_checktablenumber(L, payload->vy, "vy", 0);
    lua_checktablenumber(L, payload->vz, "vz", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_LOCAL_POSITION_NED_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_LOCAL_POSITION_NED_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_GLOBAL_POSITION_INT: { // #33
    fmav_global_position_int_t* payload = (fmav_global_position_int_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->time_boot_ms, "time_boot_ms", 0);
    lua_checktablenumber(L, payload->lat, "lat", 0);
    lua_checktablenumber(L, payload->lon, "lon", 0);
    lua_checktablenumber(L, payload->alt, "alt", 0);
    lua_checktablenumber(L, payload->relative_alt, "relative_alt", 0);
    lua_checktablenumber(L, payload->vx, "vx", 0);
    lua_checktablenumber(L, payload->vy, "vy", 0);
    lua_checktablenumber(L, payload->vz, "vz", 0);
    lua_checktablenumber(L, payload->hdg, "hdg", UINT16_MAX);
    msg_out->crc_extra = FASTMAVLINK_MSG_GLOBAL_POSITION_INT_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_GLOBAL_POSITION_INT_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_RC_CHANNELS_SCALED: { // #34
    fmav_rc_channels_scaled_t* payload = (fmav_rc_channels_scaled_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->time_boot_ms, "time_boot_ms", 0);
    lua_checktablenumber(L, payload->port, "port", 0);
    lua_checktablenumber(L, payload->chan1_scaled, "chan1_scaled", INT16_MAX);
    lua_checktablenumber(L, payload->chan2_scaled, "chan2_scaled", INT16_MAX);
    lua_checktablenumber(L, payload->chan3_scaled, "chan3_scaled", INT16_MAX);
    lua_checktablenumber(L, payload->chan4_scaled, "chan4_scaled", INT16_MAX);
    lua_checktablenumber(L, payload->chan5_scaled, "chan5_scaled", INT16_MAX);
    lua_checktablenumber(L, payload->chan6_scaled, "chan6_scaled", INT16_MAX);
    lua_checktablenumber(L, payload->chan7_scaled, "chan7_scaled", INT16_MAX);
    lua_checktablenumber(L, payload->chan8_scaled, "chan8_scaled", INT16_MAX);
    lua_checktablenumber(L, payload->rssi, "rssi", UINT8_MAX);
    msg_out->crc_extra = FASTMAVLINK_MSG_RC_CHANNELS_SCALED_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_RC_CHANNELS_SCALED_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_RC_CHANNELS_RAW: { // #35
    fmav_rc_channels_raw_t* payload = (fmav_rc_channels_raw_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->time_boot_ms, "time_boot_ms", 0);
    lua_checktablenumber(L, payload->port, "port", 0);
    lua_checktablenumber(L, payload->chan1_raw, "chan1_raw", UINT16_MAX);
    lua_checktablenumber(L, payload->chan2_raw, "chan2_raw", UINT16_MAX);
    lua_checktablenumber(L, payload->chan3_raw, "chan3_raw", UINT16_MAX);
    lua_checktablenumber(L, payload->chan4_raw, "chan4_raw", UINT16_MAX);
    lua_checktablenumber(L, payload->chan5_raw, "chan5_raw", UINT16_MAX);
    lua_checktablenumber(L, payload->chan6_raw, "chan6_raw", UINT16_MAX);
    lua_checktablenumber(L, payload->chan7_raw, "chan7_raw", UINT16_MAX);
    lua_checktablenumber(L, payload->chan8_raw, "chan8_raw", UINT16_MAX);
    lua_checktablenumber(L, payload->rssi, "rssi", UINT8_MAX);
    msg_out->crc_extra = FASTMAVLINK_MSG_RC_CHANNELS_RAW_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_RC_CHANNELS_RAW_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_SERVO_OUTPUT_RAW: { // #36
    fmav_servo_output_raw_t* payload = (fmav_servo_output_raw_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->time_usec, "time_usec", 0);
    lua_checktablenumber(L, payload->port, "port", 0);
    lua_checktablenumber(L, payload->servo1_raw, "servo1_raw", 0);
    lua_checktablenumber(L, payload->servo2_raw, "servo2_raw", 0);
    lua_checktablenumber(L, payload->servo3_raw, "servo3_raw", 0);
    lua_checktablenumber(L, payload->servo4_raw, "servo4_raw", 0);
    lua_checktablenumber(L, payload->servo5_raw, "servo5_raw", 0);
    lua_checktablenumber(L, payload->servo6_raw, "servo6_raw", 0);
    lua_checktablenumber(L, payload->servo7_raw, "servo7_raw", 0);
    lua_checktablenumber(L, payload->servo8_raw, "servo8_raw", 0);
    lua_checktablenumber(L, payload->servo9_raw, "servo9_raw", 0);
    lua_checktablenumber(L, payload->servo10_raw, "servo10_raw", 0);
    lua_checktablenumber(L, payload->servo11_raw, "servo11_raw", 0);
    lua_checktablenumber(L, payload->servo12_raw, "servo12_raw", 0);
    lua_checktablenumber(L, payload->servo13_raw, "servo13_raw", 0);
    lua_checktablenumber(L, payload->servo14_raw, "servo14_raw", 0);
    lua_checktablenumber(L, payload->servo15_raw, "servo15_raw", 0);
    lua_checktablenumber(L, payload->servo16_raw, "servo16_raw", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_SERVO_OUTPUT_RAW_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_SERVO_OUTPUT_RAW_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_MISSION_REQUEST_PARTIAL_LIST: { // #37
    fmav_mission_request_partial_list_t* payload = (fmav_mission_request_partial_list_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->target_system, "target_sysid", 0);
    lua_checktablenumber(L, payload->target_component, "target_compid", 0);
    lua_checktablenumber(L, payload->start_index, "start_index", 0);
    lua_checktablenumber(L, payload->end_index, "end_index", 0);
    lua_checktablenumber(L, payload->mission_type, "mission_type", 0);
    msg_out->target_sysid = payload->target_system;
    msg_out->target_compid = payload->target_component;
    msg_out->crc_extra = FASTMAVLINK_MSG_MISSION_REQUEST_PARTIAL_LIST_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_MISSION_REQUEST_PARTIAL_LIST_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_MISSION_WRITE_PARTIAL_LIST: { // #38
    fmav_mission_write_partial_list_t* payload = (fmav_mission_write_partial_list_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->target_system, "target_sysid", 0);
    lua_checktablenumber(L, payload->target_component, "target_compid", 0);
    lua_checktablenumber(L, payload->start_index, "start_index", 0);
    lua_checktablenumber(L, payload->end_index, "end_index", 0);
    lua_checktablenumber(L, payload->mission_type, "mission_type", 0);
    msg_out->target_sysid = payload->target_system;
    msg_out->target_compid = payload->target_component;
    msg_out->crc_extra = FASTMAVLINK_MSG_MISSION_WRITE_PARTIAL_LIST_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_MISSION_WRITE_PARTIAL_LIST_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_MISSION_ITEM: { // #39
    fmav_mission_item_t* payload = (fmav_mission_item_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->target_system, "target_sysid", 0);
    lua_checktablenumber(L, payload->target_component, "target_compid", 0);
    lua_checktablenumber(L, payload->seq, "seq", 0);
    lua_checktablenumber(L, payload->frame, "frame", 0);
    lua_checktablenumber(L, payload->command, "command", 0);
    lua_checktablenumber(L, payload->current, "current", 0);
    lua_checktablenumber(L, payload->autocontinue, "autocontinue", 0);
    lua_checktablenumber(L, payload->param1, "param1", 0);
    lua_checktablenumber(L, payload->param2, "param2", 0);
    lua_checktablenumber(L, payload->param3, "param3", 0);
    lua_checktablenumber(L, payload->param4, "param4", 0);
    lua_checktablenumber(L, payload->x, "x", 0);
    lua_checktablenumber(L, payload->y, "y", 0);
    lua_checktablenumber(L, payload->z, "z", 0);
    lua_checktablenumber(L, payload->mission_type, "mission_type", 0);
    msg_out->target_sysid = payload->target_system;
    msg_out->target_compid = payload->target_component;
    msg_out->crc_extra = FASTMAVLINK_MSG_MISSION_ITEM_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_MISSION_ITEM_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_MISSION_REQUEST: { // #40
    fmav_mission_request_t* payload = (fmav_mission_request_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->target_system, "target_sysid", 0);
    lua_checktablenumber(L, payload->target_component, "target_compid", 0);
    lua_checktablenumber(L, payload->seq, "seq", 0);
    lua_checktablenumber(L, payload->mission_type, "mission_type", 0);
    msg_out->target_sysid = payload->target_system;
    msg_out->target_compid = payload->target_component;
    msg_out->crc_extra = FASTMAVLINK_MSG_MISSION_REQUEST_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_MISSION_REQUEST_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_MISSION_SET_CURRENT: { // #41
    fmav_mission_set_current_t* payload = (fmav_mission_set_current_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->target_system, "target_sysid", 0);
    lua_checktablenumber(L, payload->target_component, "target_compid", 0);
    lua_checktablenumber(L, payload->seq, "seq", 0);
    msg_out->target_sysid = payload->target_system;
    msg_out->target_compid = payload->target_component;
    msg_out->crc_extra = FASTMAVLINK_MSG_MISSION_SET_CURRENT_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_MISSION_SET_CURRENT_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_MISSION_CURRENT: { // #42
    fmav_mission_current_t* payload = (fmav_mission_current_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->seq, "seq", 0);
    lua_checktablenumber(L, payload->total, "total", UINT16_MAX);
    lua_checktablenumber(L, payload->mission_state, "mission_state", 0);
    lua_checktablenumber(L, payload->mission_mode, "mission_mode", 0);
    lua_checktablenumber(L, payload->mission_id, "mission_id", 0);
    lua_checktablenumber(L, payload->fence_id, "fence_id", 0);
    lua_checktablenumber(L, payload->rally_points_id, "rally_points_id", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_MISSION_CURRENT_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_MISSION_CURRENT_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_MISSION_REQUEST_LIST: { // #43
    fmav_mission_request_list_t* payload = (fmav_mission_request_list_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->target_system, "target_sysid", 0);
    lua_checktablenumber(L, payload->target_component, "target_compid", 0);
    lua_checktablenumber(L, payload->mission_type, "mission_type", 0);
    msg_out->target_sysid = payload->target_system;
    msg_out->target_compid = payload->target_component;
    msg_out->crc_extra = FASTMAVLINK_MSG_MISSION_REQUEST_LIST_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_MISSION_REQUEST_LIST_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_MISSION_COUNT: { // #44
    fmav_mission_count_t* payload = (fmav_mission_count_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->target_system, "target_sysid", 0);
    lua_checktablenumber(L, payload->target_component, "target_compid", 0);
    lua_checktablenumber(L, payload->count, "count", 0);
    lua_checktablenumber(L, payload->mission_type, "mission_type", 0);
    lua_checktablenumber(L, payload->opaque_id, "opaque_id", 0);
    msg_out->target_sysid = payload->target_system;
    msg_out->target_compid = payload->target_component;
    msg_out->crc_extra = FASTMAVLINK_MSG_MISSION_COUNT_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_MISSION_COUNT_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_MISSION_CLEAR_ALL: { // #45
    fmav_mission_clear_all_t* payload = (fmav_mission_clear_all_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->target_system, "target_sysid", 0);
    lua_checktablenumber(L, payload->target_component, "target_compid", 0);
    lua_checktablenumber(L, payload->mission_type, "mission_type", 0);
    msg_out->target_sysid = payload->target_system;
    msg_out->target_compid = payload->target_component;
    msg_out->crc_extra = FASTMAVLINK_MSG_MISSION_CLEAR_ALL_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_MISSION_CLEAR_ALL_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_MISSION_ITEM_REACHED: { // #46
    fmav_mission_item_reached_t* payload = (fmav_mission_item_reached_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->seq, "seq", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_MISSION_ITEM_REACHED_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_MISSION_ITEM_REACHED_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_MISSION_ACK: { // #47
    fmav_mission_ack_t* payload = (fmav_mission_ack_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->target_system, "target_sysid", 0);
    lua_checktablenumber(L, payload->target_component, "target_compid", 0);
    lua_checktablenumber(L, payload->type, "type", 0);
    lua_checktablenumber(L, payload->mission_type, "mission_type", 0);
    lua_checktablenumber(L, payload->opaque_id, "opaque_id", 0);
    msg_out->target_sysid = payload->target_system;
    msg_out->target_compid = payload->target_component;
    msg_out->crc_extra = FASTMAVLINK_MSG_MISSION_ACK_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_MISSION_ACK_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_SET_GPS_GLOBAL_ORIGIN: { // #48
    fmav_set_gps_global_origin_t* payload = (fmav_set_gps_global_origin_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->target_system, "target_sysid", 0);
    lua_checktablenumber(L, payload->latitude, "latitude", 0);
    lua_checktablenumber(L, payload->longitude, "longitude", 0);
    lua_checktablenumber(L, payload->altitude, "altitude", 0);
    lua_checktablenumber(L, payload->time_usec, "time_usec", 0);
    msg_out->target_sysid = payload->target_system;
    msg_out->crc_extra = FASTMAVLINK_MSG_SET_GPS_GLOBAL_ORIGIN_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_SET_GPS_GLOBAL_ORIGIN_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN: { // #49
    fmav_gps_global_origin_t* payload = (fmav_gps_global_origin_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->latitude, "latitude", 0);
    lua_checktablenumber(L, payload->longitude, "longitude", 0);
    lua_checktablenumber(L, payload->altitude, "altitude", 0);
    lua_checktablenumber(L, payload->time_usec, "time_usec", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_GPS_GLOBAL_ORIGIN_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_GPS_GLOBAL_ORIGIN_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_PARAM_MAP_RC: { // #50
    fmav_param_map_rc_t* payload = (fmav_param_map_rc_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->target_system, "target_sysid", 0);
    lua_checktablenumber(L, payload->target_component, "target_compid", 0);
    lua_pushstring(L, "param_id"); // array param_id[16]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 16; i++) { 
        lua_checktableinumber(L, payload->param_id[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_checktablenumber(L, payload->param_index, "param_index", 0);
    lua_checktablenumber(L, payload->parameter_rc_channel_index, "parameter_rc_channel_index", 0);
    lua_checktablenumber(L, payload->param_value0, "param_value0", 0);
    lua_checktablenumber(L, payload->scale, "scale", 0);
    lua_checktablenumber(L, payload->param_value_min, "param_value_min", 0);
    lua_checktablenumber(L, payload->param_value_max, "param_value_max", 0);
    msg_out->target_sysid = payload->target_system;
    msg_out->target_compid = payload->target_component;
    msg_out->crc_extra = FASTMAVLINK_MSG_PARAM_MAP_RC_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_PARAM_MAP_RC_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_MISSION_REQUEST_INT: { // #51
    fmav_mission_request_int_t* payload = (fmav_mission_request_int_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->target_system, "target_sysid", 0);
    lua_checktablenumber(L, payload->target_component, "target_compid", 0);
    lua_checktablenumber(L, payload->seq, "seq", 0);
    lua_checktablenumber(L, payload->mission_type, "mission_type", 0);
    msg_out->target_sysid = payload->target_system;
    msg_out->target_compid = payload->target_component;
    msg_out->crc_extra = FASTMAVLINK_MSG_MISSION_REQUEST_INT_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_MISSION_REQUEST_INT_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_SAFETY_SET_ALLOWED_AREA: { // #54
    fmav_safety_set_allowed_area_t* payload = (fmav_safety_set_allowed_area_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->target_system, "target_sysid", 0);
    lua_checktablenumber(L, payload->target_component, "target_compid", 0);
    lua_checktablenumber(L, payload->frame, "frame", 0);
    lua_checktablenumber(L, payload->p1x, "p1x", 0);
    lua_checktablenumber(L, payload->p1y, "p1y", 0);
    lua_checktablenumber(L, payload->p1z, "p1z", 0);
    lua_checktablenumber(L, payload->p2x, "p2x", 0);
    lua_checktablenumber(L, payload->p2y, "p2y", 0);
    lua_checktablenumber(L, payload->p2z, "p2z", 0);
    msg_out->target_sysid = payload->target_system;
    msg_out->target_compid = payload->target_component;
    msg_out->crc_extra = FASTMAVLINK_MSG_SAFETY_SET_ALLOWED_AREA_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_SAFETY_SET_ALLOWED_AREA_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_SAFETY_ALLOWED_AREA: { // #55
    fmav_safety_allowed_area_t* payload = (fmav_safety_allowed_area_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->frame, "frame", 0);
    lua_checktablenumber(L, payload->p1x, "p1x", 0);
    lua_checktablenumber(L, payload->p1y, "p1y", 0);
    lua_checktablenumber(L, payload->p1z, "p1z", 0);
    lua_checktablenumber(L, payload->p2x, "p2x", 0);
    lua_checktablenumber(L, payload->p2y, "p2y", 0);
    lua_checktablenumber(L, payload->p2z, "p2z", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_SAFETY_ALLOWED_AREA_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_SAFETY_ALLOWED_AREA_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_ATTITUDE_QUATERNION_COV: { // #61
    fmav_attitude_quaternion_cov_t* payload = (fmav_attitude_quaternion_cov_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->time_usec, "time_usec", 0);
    lua_pushstring(L, "q"); // array q[4]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 4; i++) { 
        lua_checktableinumber(L, payload->q[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_checktablenumber(L, payload->rollspeed, "rollspeed", 0);
    lua_checktablenumber(L, payload->pitchspeed, "pitchspeed", 0);
    lua_checktablenumber(L, payload->yawspeed, "yawspeed", 0);
    lua_pushstring(L, "covariance"); // array covariance[9]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      lua_checktableinumber(L, payload->covariance[0], 1, NAN); // lua is 1 indexed
      for (int i = 1; i < 9; i++) { 
        lua_checktableinumber(L, payload->covariance[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    msg_out->crc_extra = FASTMAVLINK_MSG_ATTITUDE_QUATERNION_COV_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_ATTITUDE_QUATERNION_COV_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT: { // #62
    fmav_nav_controller_output_t* payload = (fmav_nav_controller_output_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->nav_roll, "nav_roll", 0);
    lua_checktablenumber(L, payload->nav_pitch, "nav_pitch", 0);
    lua_checktablenumber(L, payload->nav_bearing, "nav_bearing", 0);
    lua_checktablenumber(L, payload->target_bearing, "target_bearing", 0);
    lua_checktablenumber(L, payload->wp_dist, "wp_dist", 0);
    lua_checktablenumber(L, payload->alt_error, "alt_error", 0);
    lua_checktablenumber(L, payload->aspd_error, "aspd_error", 0);
    lua_checktablenumber(L, payload->xtrack_error, "xtrack_error", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_NAV_CONTROLLER_OUTPUT_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_NAV_CONTROLLER_OUTPUT_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_GLOBAL_POSITION_INT_COV: { // #63
    fmav_global_position_int_cov_t* payload = (fmav_global_position_int_cov_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->time_usec, "time_usec", 0);
    lua_checktablenumber(L, payload->estimator_type, "estimator_type", 0);
    lua_checktablenumber(L, payload->lat, "lat", 0);
    lua_checktablenumber(L, payload->lon, "lon", 0);
    lua_checktablenumber(L, payload->alt, "alt", 0);
    lua_checktablenumber(L, payload->relative_alt, "relative_alt", 0);
    lua_checktablenumber(L, payload->vx, "vx", 0);
    lua_checktablenumber(L, payload->vy, "vy", 0);
    lua_checktablenumber(L, payload->vz, "vz", 0);
    lua_pushstring(L, "covariance"); // array covariance[36]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      lua_checktableinumber(L, payload->covariance[0], 1, NAN); // lua is 1 indexed
      for (int i = 1; i < 36; i++) { 
        lua_checktableinumber(L, payload->covariance[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    msg_out->crc_extra = FASTMAVLINK_MSG_GLOBAL_POSITION_INT_COV_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_GLOBAL_POSITION_INT_COV_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_LOCAL_POSITION_NED_COV: { // #64
    fmav_local_position_ned_cov_t* payload = (fmav_local_position_ned_cov_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->time_usec, "time_usec", 0);
    lua_checktablenumber(L, payload->estimator_type, "estimator_type", 0);
    lua_checktablenumber(L, payload->x, "x", 0);
    lua_checktablenumber(L, payload->y, "y", 0);
    lua_checktablenumber(L, payload->z, "z", 0);
    lua_checktablenumber(L, payload->vx, "vx", 0);
    lua_checktablenumber(L, payload->vy, "vy", 0);
    lua_checktablenumber(L, payload->vz, "vz", 0);
    lua_checktablenumber(L, payload->ax, "ax", 0);
    lua_checktablenumber(L, payload->ay, "ay", 0);
    lua_checktablenumber(L, payload->az, "az", 0);
    lua_pushstring(L, "covariance"); // array covariance[45]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      lua_checktableinumber(L, payload->covariance[0], 1, NAN); // lua is 1 indexed
      for (int i = 1; i < 45; i++) { 
        lua_checktableinumber(L, payload->covariance[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    msg_out->crc_extra = FASTMAVLINK_MSG_LOCAL_POSITION_NED_COV_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_LOCAL_POSITION_NED_COV_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_RC_CHANNELS: { // #65
    fmav_rc_channels_t* payload = (fmav_rc_channels_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->time_boot_ms, "time_boot_ms", 0);
    lua_checktablenumber(L, payload->chancount, "chancount", 0);
    lua_checktablenumber(L, payload->chan1_raw, "chan1_raw", UINT16_MAX);
    lua_checktablenumber(L, payload->chan2_raw, "chan2_raw", UINT16_MAX);
    lua_checktablenumber(L, payload->chan3_raw, "chan3_raw", UINT16_MAX);
    lua_checktablenumber(L, payload->chan4_raw, "chan4_raw", UINT16_MAX);
    lua_checktablenumber(L, payload->chan5_raw, "chan5_raw", UINT16_MAX);
    lua_checktablenumber(L, payload->chan6_raw, "chan6_raw", UINT16_MAX);
    lua_checktablenumber(L, payload->chan7_raw, "chan7_raw", UINT16_MAX);
    lua_checktablenumber(L, payload->chan8_raw, "chan8_raw", UINT16_MAX);
    lua_checktablenumber(L, payload->chan9_raw, "chan9_raw", UINT16_MAX);
    lua_checktablenumber(L, payload->chan10_raw, "chan10_raw", UINT16_MAX);
    lua_checktablenumber(L, payload->chan11_raw, "chan11_raw", UINT16_MAX);
    lua_checktablenumber(L, payload->chan12_raw, "chan12_raw", UINT16_MAX);
    lua_checktablenumber(L, payload->chan13_raw, "chan13_raw", UINT16_MAX);
    lua_checktablenumber(L, payload->chan14_raw, "chan14_raw", UINT16_MAX);
    lua_checktablenumber(L, payload->chan15_raw, "chan15_raw", UINT16_MAX);
    lua_checktablenumber(L, payload->chan16_raw, "chan16_raw", UINT16_MAX);
    lua_checktablenumber(L, payload->chan17_raw, "chan17_raw", UINT16_MAX);
    lua_checktablenumber(L, payload->chan18_raw, "chan18_raw", UINT16_MAX);
    lua_checktablenumber(L, payload->rssi, "rssi", UINT8_MAX);
    msg_out->crc_extra = FASTMAVLINK_MSG_RC_CHANNELS_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_RC_CHANNELS_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_REQUEST_DATA_STREAM: { // #66
    fmav_request_data_stream_t* payload = (fmav_request_data_stream_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->target_system, "target_sysid", 0);
    lua_checktablenumber(L, payload->target_component, "target_compid", 0);
    lua_checktablenumber(L, payload->req_stream_id, "req_stream_id", 0);
    lua_checktablenumber(L, payload->req_message_rate, "req_message_rate", 0);
    lua_checktablenumber(L, payload->start_stop, "start_stop", 0);
    msg_out->target_sysid = payload->target_system;
    msg_out->target_compid = payload->target_component;
    msg_out->crc_extra = FASTMAVLINK_MSG_REQUEST_DATA_STREAM_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_REQUEST_DATA_STREAM_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_DATA_STREAM: { // #67
    fmav_data_stream_t* payload = (fmav_data_stream_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->stream_id, "stream_id", 0);
    lua_checktablenumber(L, payload->message_rate, "message_rate", 0);
    lua_checktablenumber(L, payload->on_off, "on_off", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_DATA_STREAM_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_DATA_STREAM_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_MANUAL_CONTROL: { // #69
    fmav_manual_control_t* payload = (fmav_manual_control_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->target, "target_sysid", 0);
    lua_checktablenumber(L, payload->x, "x", INT16_MAX);
    lua_checktablenumber(L, payload->y, "y", INT16_MAX);
    lua_checktablenumber(L, payload->z, "z", INT16_MAX);
    lua_checktablenumber(L, payload->r, "r", INT16_MAX);
    lua_checktablenumber(L, payload->buttons, "buttons", 0);
    lua_checktablenumber(L, payload->buttons2, "buttons2", 0);
    lua_checktablenumber(L, payload->enabled_extensions, "enabled_extensions", 0);
    lua_checktablenumber(L, payload->s, "s", 0);
    lua_checktablenumber(L, payload->t, "t", 0);
    lua_checktablenumber(L, payload->aux1, "aux1", 0);
    lua_checktablenumber(L, payload->aux2, "aux2", 0);
    lua_checktablenumber(L, payload->aux3, "aux3", 0);
    lua_checktablenumber(L, payload->aux4, "aux4", 0);
    lua_checktablenumber(L, payload->aux5, "aux5", 0);
    lua_checktablenumber(L, payload->aux6, "aux6", 0);
    msg_out->target_sysid = payload->target;
    msg_out->crc_extra = FASTMAVLINK_MSG_MANUAL_CONTROL_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_MANUAL_CONTROL_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE: { // #70
    fmav_rc_channels_override_t* payload = (fmav_rc_channels_override_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->target_system, "target_sysid", 0);
    lua_checktablenumber(L, payload->target_component, "target_compid", 0);
    lua_checktablenumber(L, payload->chan1_raw, "chan1_raw", UINT16_MAX);
    lua_checktablenumber(L, payload->chan2_raw, "chan2_raw", UINT16_MAX);
    lua_checktablenumber(L, payload->chan3_raw, "chan3_raw", UINT16_MAX);
    lua_checktablenumber(L, payload->chan4_raw, "chan4_raw", UINT16_MAX);
    lua_checktablenumber(L, payload->chan5_raw, "chan5_raw", UINT16_MAX);
    lua_checktablenumber(L, payload->chan6_raw, "chan6_raw", UINT16_MAX);
    lua_checktablenumber(L, payload->chan7_raw, "chan7_raw", UINT16_MAX);
    lua_checktablenumber(L, payload->chan8_raw, "chan8_raw", UINT16_MAX);
    lua_checktablenumber(L, payload->chan9_raw, "chan9_raw", 0);
    lua_checktablenumber(L, payload->chan10_raw, "chan10_raw", 0);
    lua_checktablenumber(L, payload->chan11_raw, "chan11_raw", 0);
    lua_checktablenumber(L, payload->chan12_raw, "chan12_raw", 0);
    lua_checktablenumber(L, payload->chan13_raw, "chan13_raw", 0);
    lua_checktablenumber(L, payload->chan14_raw, "chan14_raw", 0);
    lua_checktablenumber(L, payload->chan15_raw, "chan15_raw", 0);
    lua_checktablenumber(L, payload->chan16_raw, "chan16_raw", 0);
    lua_checktablenumber(L, payload->chan17_raw, "chan17_raw", 0);
    lua_checktablenumber(L, payload->chan18_raw, "chan18_raw", 0);
    msg_out->target_sysid = payload->target_system;
    msg_out->target_compid = payload->target_component;
    msg_out->crc_extra = FASTMAVLINK_MSG_RC_CHANNELS_OVERRIDE_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_RC_CHANNELS_OVERRIDE_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_MISSION_ITEM_INT: { // #73
    fmav_mission_item_int_t* payload = (fmav_mission_item_int_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->target_system, "target_sysid", 0);
    lua_checktablenumber(L, payload->target_component, "target_compid", 0);
    lua_checktablenumber(L, payload->seq, "seq", 0);
    lua_checktablenumber(L, payload->frame, "frame", 0);
    lua_checktablenumber(L, payload->command, "command", 0);
    lua_checktablenumber(L, payload->current, "current", 0);
    lua_checktablenumber(L, payload->autocontinue, "autocontinue", 0);
    lua_checktablenumber(L, payload->param1, "param1", 0);
    lua_checktablenumber(L, payload->param2, "param2", 0);
    lua_checktablenumber(L, payload->param3, "param3", 0);
    lua_checktablenumber(L, payload->param4, "param4", 0);
    lua_checktablenumber(L, payload->x, "x", 0);
    lua_checktablenumber(L, payload->y, "y", 0);
    lua_checktablenumber(L, payload->z, "z", 0);
    lua_checktablenumber(L, payload->mission_type, "mission_type", 0);
    msg_out->target_sysid = payload->target_system;
    msg_out->target_compid = payload->target_component;
    msg_out->crc_extra = FASTMAVLINK_MSG_MISSION_ITEM_INT_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_MISSION_ITEM_INT_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_VFR_HUD: { // #74
    fmav_vfr_hud_t* payload = (fmav_vfr_hud_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->airspeed, "airspeed", 0);
    lua_checktablenumber(L, payload->groundspeed, "groundspeed", 0);
    lua_checktablenumber(L, payload->heading, "heading", 0);
    lua_checktablenumber(L, payload->throttle, "throttle", 0);
    lua_checktablenumber(L, payload->alt, "alt", 0);
    lua_checktablenumber(L, payload->climb, "climb", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_VFR_HUD_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_VFR_HUD_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_COMMAND_INT: { // #75
    fmav_command_int_t* payload = (fmav_command_int_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->target_system, "target_sysid", 0);
    lua_checktablenumber(L, payload->target_component, "target_compid", 0);
    lua_checktablenumber(L, payload->frame, "frame", 0);
    lua_checktablenumber(L, payload->command, "command", 0);
    lua_checktablenumber(L, payload->current, "current", 0);
    lua_checktablenumber(L, payload->autocontinue, "autocontinue", 0);
    lua_checktablenumber(L, payload->param1, "param1", NAN);
    lua_checktablenumber(L, payload->param2, "param2", NAN);
    lua_checktablenumber(L, payload->param3, "param3", NAN);
    lua_checktablenumber(L, payload->param4, "param4", NAN);
    lua_checktablenumber(L, payload->x, "x", INT32_MAX);
    lua_checktablenumber(L, payload->y, "y", INT32_MAX);
    lua_checktablenumber(L, payload->z, "z", NAN);
    msg_out->target_sysid = payload->target_system;
    msg_out->target_compid = payload->target_component;
    msg_out->crc_extra = FASTMAVLINK_MSG_COMMAND_INT_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_COMMAND_INT_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_COMMAND_LONG: { // #76
    fmav_command_long_t* payload = (fmav_command_long_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->target_system, "target_sysid", 0);
    lua_checktablenumber(L, payload->target_component, "target_compid", 0);
    lua_checktablenumber(L, payload->command, "command", 0);
    lua_checktablenumber(L, payload->confirmation, "confirmation", 0);
    lua_checktablenumber(L, payload->param1, "param1", NAN);
    lua_checktablenumber(L, payload->param2, "param2", NAN);
    lua_checktablenumber(L, payload->param3, "param3", NAN);
    lua_checktablenumber(L, payload->param4, "param4", NAN);
    lua_checktablenumber(L, payload->param5, "param5", NAN);
    lua_checktablenumber(L, payload->param6, "param6", NAN);
    lua_checktablenumber(L, payload->param7, "param7", NAN);
    msg_out->target_sysid = payload->target_system;
    msg_out->target_compid = payload->target_component;
    msg_out->crc_extra = FASTMAVLINK_MSG_COMMAND_LONG_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_COMMAND_LONG_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_COMMAND_ACK: { // #77
    fmav_command_ack_t* payload = (fmav_command_ack_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->command, "command", 0);
    lua_checktablenumber(L, payload->result, "result", 0);
    lua_checktablenumber(L, payload->progress, "progress", UINT8_MAX);
    lua_checktablenumber(L, payload->result_param2, "result_param2", 0);
    lua_checktablenumber(L, payload->target_system, "target_sysid", 0);
    lua_checktablenumber(L, payload->target_component, "target_compid", 0);
    msg_out->target_sysid = payload->target_system;
    msg_out->target_compid = payload->target_component;
    msg_out->crc_extra = FASTMAVLINK_MSG_COMMAND_ACK_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_COMMAND_ACK_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_COMMAND_CANCEL: { // #80
    fmav_command_cancel_t* payload = (fmav_command_cancel_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->target_system, "target_sysid", 0);
    lua_checktablenumber(L, payload->target_component, "target_compid", 0);
    lua_checktablenumber(L, payload->command, "command", 0);
    msg_out->target_sysid = payload->target_system;
    msg_out->target_compid = payload->target_component;
    msg_out->crc_extra = FASTMAVLINK_MSG_COMMAND_CANCEL_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_COMMAND_CANCEL_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_MANUAL_SETPOINT: { // #81
    fmav_manual_setpoint_t* payload = (fmav_manual_setpoint_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->time_boot_ms, "time_boot_ms", 0);
    lua_checktablenumber(L, payload->roll, "roll", 0);
    lua_checktablenumber(L, payload->pitch, "pitch", 0);
    lua_checktablenumber(L, payload->yaw, "yaw", 0);
    lua_checktablenumber(L, payload->thrust, "thrust", 0);
    lua_checktablenumber(L, payload->mode_switch, "mode_switch", 0);
    lua_checktablenumber(L, payload->manual_override_switch, "manual_override_switch", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_MANUAL_SETPOINT_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_MANUAL_SETPOINT_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_SET_ATTITUDE_TARGET: { // #82
    fmav_set_attitude_target_t* payload = (fmav_set_attitude_target_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->time_boot_ms, "time_boot_ms", 0);
    lua_checktablenumber(L, payload->target_system, "target_sysid", 0);
    lua_checktablenumber(L, payload->target_component, "target_compid", 0);
    lua_checktablenumber(L, payload->type_mask, "type_mask", 0);
    lua_pushstring(L, "q"); // array q[4]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 4; i++) { 
        lua_checktableinumber(L, payload->q[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_checktablenumber(L, payload->body_roll_rate, "body_roll_rate", 0);
    lua_checktablenumber(L, payload->body_pitch_rate, "body_pitch_rate", 0);
    lua_checktablenumber(L, payload->body_yaw_rate, "body_yaw_rate", 0);
    lua_checktablenumber(L, payload->thrust, "thrust", 0);
    lua_pushstring(L, "thrust_body"); // array thrust_body[3]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 3; i++) { 
        lua_checktableinumber(L, payload->thrust_body[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    msg_out->target_sysid = payload->target_system;
    msg_out->target_compid = payload->target_component;
    msg_out->crc_extra = FASTMAVLINK_MSG_SET_ATTITUDE_TARGET_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_SET_ATTITUDE_TARGET_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_ATTITUDE_TARGET: { // #83
    fmav_attitude_target_t* payload = (fmav_attitude_target_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->time_boot_ms, "time_boot_ms", 0);
    lua_checktablenumber(L, payload->type_mask, "type_mask", 0);
    lua_pushstring(L, "q"); // array q[4]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 4; i++) { 
        lua_checktableinumber(L, payload->q[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_checktablenumber(L, payload->body_roll_rate, "body_roll_rate", 0);
    lua_checktablenumber(L, payload->body_pitch_rate, "body_pitch_rate", 0);
    lua_checktablenumber(L, payload->body_yaw_rate, "body_yaw_rate", 0);
    lua_checktablenumber(L, payload->thrust, "thrust", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_ATTITUDE_TARGET_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_ATTITUDE_TARGET_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED: { // #84
    fmav_set_position_target_local_ned_t* payload = (fmav_set_position_target_local_ned_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->time_boot_ms, "time_boot_ms", 0);
    lua_checktablenumber(L, payload->target_system, "target_sysid", 0);
    lua_checktablenumber(L, payload->target_component, "target_compid", 0);
    lua_checktablenumber(L, payload->coordinate_frame, "coordinate_frame", 0);
    lua_checktablenumber(L, payload->type_mask, "type_mask", 0);
    lua_checktablenumber(L, payload->x, "x", 0);
    lua_checktablenumber(L, payload->y, "y", 0);
    lua_checktablenumber(L, payload->z, "z", 0);
    lua_checktablenumber(L, payload->vx, "vx", 0);
    lua_checktablenumber(L, payload->vy, "vy", 0);
    lua_checktablenumber(L, payload->vz, "vz", 0);
    lua_checktablenumber(L, payload->afx, "afx", 0);
    lua_checktablenumber(L, payload->afy, "afy", 0);
    lua_checktablenumber(L, payload->afz, "afz", 0);
    lua_checktablenumber(L, payload->yaw, "yaw", 0);
    lua_checktablenumber(L, payload->yaw_rate, "yaw_rate", 0);
    msg_out->target_sysid = payload->target_system;
    msg_out->target_compid = payload->target_component;
    msg_out->crc_extra = FASTMAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED: { // #85
    fmav_position_target_local_ned_t* payload = (fmav_position_target_local_ned_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->time_boot_ms, "time_boot_ms", 0);
    lua_checktablenumber(L, payload->coordinate_frame, "coordinate_frame", 0);
    lua_checktablenumber(L, payload->type_mask, "type_mask", 0);
    lua_checktablenumber(L, payload->x, "x", 0);
    lua_checktablenumber(L, payload->y, "y", 0);
    lua_checktablenumber(L, payload->z, "z", 0);
    lua_checktablenumber(L, payload->vx, "vx", 0);
    lua_checktablenumber(L, payload->vy, "vy", 0);
    lua_checktablenumber(L, payload->vz, "vz", 0);
    lua_checktablenumber(L, payload->afx, "afx", 0);
    lua_checktablenumber(L, payload->afy, "afy", 0);
    lua_checktablenumber(L, payload->afz, "afz", 0);
    lua_checktablenumber(L, payload->yaw, "yaw", 0);
    lua_checktablenumber(L, payload->yaw_rate, "yaw_rate", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_POSITION_TARGET_LOCAL_NED_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_POSITION_TARGET_LOCAL_NED_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT: { // #86
    fmav_set_position_target_global_int_t* payload = (fmav_set_position_target_global_int_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->time_boot_ms, "time_boot_ms", 0);
    lua_checktablenumber(L, payload->target_system, "target_sysid", 0);
    lua_checktablenumber(L, payload->target_component, "target_compid", 0);
    lua_checktablenumber(L, payload->coordinate_frame, "coordinate_frame", 0);
    lua_checktablenumber(L, payload->type_mask, "type_mask", 0);
    lua_checktablenumber(L, payload->lat_int, "lat_int", 0);
    lua_checktablenumber(L, payload->lon_int, "lon_int", 0);
    lua_checktablenumber(L, payload->alt, "alt", 0);
    lua_checktablenumber(L, payload->vx, "vx", 0);
    lua_checktablenumber(L, payload->vy, "vy", 0);
    lua_checktablenumber(L, payload->vz, "vz", 0);
    lua_checktablenumber(L, payload->afx, "afx", 0);
    lua_checktablenumber(L, payload->afy, "afy", 0);
    lua_checktablenumber(L, payload->afz, "afz", 0);
    lua_checktablenumber(L, payload->yaw, "yaw", 0);
    lua_checktablenumber(L, payload->yaw_rate, "yaw_rate", 0);
    msg_out->target_sysid = payload->target_system;
    msg_out->target_compid = payload->target_component;
    msg_out->crc_extra = FASTMAVLINK_MSG_SET_POSITION_TARGET_GLOBAL_INT_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_SET_POSITION_TARGET_GLOBAL_INT_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT: { // #87
    fmav_position_target_global_int_t* payload = (fmav_position_target_global_int_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->time_boot_ms, "time_boot_ms", 0);
    lua_checktablenumber(L, payload->coordinate_frame, "coordinate_frame", 0);
    lua_checktablenumber(L, payload->type_mask, "type_mask", 0);
    lua_checktablenumber(L, payload->lat_int, "lat_int", 0);
    lua_checktablenumber(L, payload->lon_int, "lon_int", 0);
    lua_checktablenumber(L, payload->alt, "alt", 0);
    lua_checktablenumber(L, payload->vx, "vx", 0);
    lua_checktablenumber(L, payload->vy, "vy", 0);
    lua_checktablenumber(L, payload->vz, "vz", 0);
    lua_checktablenumber(L, payload->afx, "afx", 0);
    lua_checktablenumber(L, payload->afy, "afy", 0);
    lua_checktablenumber(L, payload->afz, "afz", 0);
    lua_checktablenumber(L, payload->yaw, "yaw", 0);
    lua_checktablenumber(L, payload->yaw_rate, "yaw_rate", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_POSITION_TARGET_GLOBAL_INT_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_POSITION_TARGET_GLOBAL_INT_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET: { // #89
    fmav_local_position_ned_system_global_offset_t* payload = (fmav_local_position_ned_system_global_offset_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->time_boot_ms, "time_boot_ms", 0);
    lua_checktablenumber(L, payload->x, "x", 0);
    lua_checktablenumber(L, payload->y, "y", 0);
    lua_checktablenumber(L, payload->z, "z", 0);
    lua_checktablenumber(L, payload->roll, "roll", 0);
    lua_checktablenumber(L, payload->pitch, "pitch", 0);
    lua_checktablenumber(L, payload->yaw, "yaw", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_HIL_STATE: { // #90
    fmav_hil_state_t* payload = (fmav_hil_state_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->time_usec, "time_usec", 0);
    lua_checktablenumber(L, payload->roll, "roll", 0);
    lua_checktablenumber(L, payload->pitch, "pitch", 0);
    lua_checktablenumber(L, payload->yaw, "yaw", 0);
    lua_checktablenumber(L, payload->rollspeed, "rollspeed", 0);
    lua_checktablenumber(L, payload->pitchspeed, "pitchspeed", 0);
    lua_checktablenumber(L, payload->yawspeed, "yawspeed", 0);
    lua_checktablenumber(L, payload->lat, "lat", 0);
    lua_checktablenumber(L, payload->lon, "lon", 0);
    lua_checktablenumber(L, payload->alt, "alt", 0);
    lua_checktablenumber(L, payload->vx, "vx", 0);
    lua_checktablenumber(L, payload->vy, "vy", 0);
    lua_checktablenumber(L, payload->vz, "vz", 0);
    lua_checktablenumber(L, payload->xacc, "xacc", 0);
    lua_checktablenumber(L, payload->yacc, "yacc", 0);
    lua_checktablenumber(L, payload->zacc, "zacc", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_HIL_STATE_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_HIL_STATE_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_HIL_CONTROLS: { // #91
    fmav_hil_controls_t* payload = (fmav_hil_controls_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->time_usec, "time_usec", 0);
    lua_checktablenumber(L, payload->roll_ailerons, "roll_ailerons", 0);
    lua_checktablenumber(L, payload->pitch_elevator, "pitch_elevator", 0);
    lua_checktablenumber(L, payload->yaw_rudder, "yaw_rudder", 0);
    lua_checktablenumber(L, payload->throttle, "throttle", 0);
    lua_checktablenumber(L, payload->aux1, "aux1", 0);
    lua_checktablenumber(L, payload->aux2, "aux2", 0);
    lua_checktablenumber(L, payload->aux3, "aux3", 0);
    lua_checktablenumber(L, payload->aux4, "aux4", 0);
    lua_checktablenumber(L, payload->mode, "mode", 0);
    lua_checktablenumber(L, payload->nav_mode, "nav_mode", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_HIL_CONTROLS_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_HIL_CONTROLS_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_HIL_RC_INPUTS_RAW: { // #92
    fmav_hil_rc_inputs_raw_t* payload = (fmav_hil_rc_inputs_raw_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->time_usec, "time_usec", 0);
    lua_checktablenumber(L, payload->chan1_raw, "chan1_raw", 0);
    lua_checktablenumber(L, payload->chan2_raw, "chan2_raw", 0);
    lua_checktablenumber(L, payload->chan3_raw, "chan3_raw", 0);
    lua_checktablenumber(L, payload->chan4_raw, "chan4_raw", 0);
    lua_checktablenumber(L, payload->chan5_raw, "chan5_raw", 0);
    lua_checktablenumber(L, payload->chan6_raw, "chan6_raw", 0);
    lua_checktablenumber(L, payload->chan7_raw, "chan7_raw", 0);
    lua_checktablenumber(L, payload->chan8_raw, "chan8_raw", 0);
    lua_checktablenumber(L, payload->chan9_raw, "chan9_raw", 0);
    lua_checktablenumber(L, payload->chan10_raw, "chan10_raw", 0);
    lua_checktablenumber(L, payload->chan11_raw, "chan11_raw", 0);
    lua_checktablenumber(L, payload->chan12_raw, "chan12_raw", 0);
    lua_checktablenumber(L, payload->rssi, "rssi", UINT8_MAX);
    msg_out->crc_extra = FASTMAVLINK_MSG_HIL_RC_INPUTS_RAW_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_HIL_RC_INPUTS_RAW_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS: { // #93
    fmav_hil_actuator_controls_t* payload = (fmav_hil_actuator_controls_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->time_usec, "time_usec", 0);
    lua_pushstring(L, "controls"); // array controls[16]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 16; i++) { 
        lua_checktableinumber(L, payload->controls[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_checktablenumber(L, payload->mode, "mode", 0);
    lua_checktablenumber(L, payload->flags, "flags", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_HIL_ACTUATOR_CONTROLS_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_HIL_ACTUATOR_CONTROLS_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_OPTICAL_FLOW: { // #100
    fmav_optical_flow_t* payload = (fmav_optical_flow_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->time_usec, "time_usec", 0);
    lua_checktablenumber(L, payload->sensor_id, "sensor_id", 0);
    lua_checktablenumber(L, payload->flow_x, "flow_x", 0);
    lua_checktablenumber(L, payload->flow_y, "flow_y", 0);
    lua_checktablenumber(L, payload->flow_comp_m_x, "flow_comp_m_x", 0);
    lua_checktablenumber(L, payload->flow_comp_m_y, "flow_comp_m_y", 0);
    lua_checktablenumber(L, payload->quality, "quality", 0);
    lua_checktablenumber(L, payload->ground_distance, "ground_distance", 0);
    lua_checktablenumber(L, payload->flow_rate_x, "flow_rate_x", 0);
    lua_checktablenumber(L, payload->flow_rate_y, "flow_rate_y", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_OPTICAL_FLOW_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_OPTICAL_FLOW_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_GLOBAL_VISION_POSITION_ESTIMATE: { // #101
    fmav_global_vision_position_estimate_t* payload = (fmav_global_vision_position_estimate_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->usec, "usec", 0);
    lua_checktablenumber(L, payload->x, "x", 0);
    lua_checktablenumber(L, payload->y, "y", 0);
    lua_checktablenumber(L, payload->z, "z", 0);
    lua_checktablenumber(L, payload->roll, "roll", 0);
    lua_checktablenumber(L, payload->pitch, "pitch", 0);
    lua_checktablenumber(L, payload->yaw, "yaw", 0);
    lua_pushstring(L, "covariance"); // array covariance[21]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      lua_checktableinumber(L, payload->covariance[0], 1, NAN); // lua is 1 indexed
      for (int i = 1; i < 21; i++) { 
        lua_checktableinumber(L, payload->covariance[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_checktablenumber(L, payload->reset_counter, "reset_counter", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_GLOBAL_VISION_POSITION_ESTIMATE_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_GLOBAL_VISION_POSITION_ESTIMATE_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_VISION_POSITION_ESTIMATE: { // #102
    fmav_vision_position_estimate_t* payload = (fmav_vision_position_estimate_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->usec, "usec", 0);
    lua_checktablenumber(L, payload->x, "x", 0);
    lua_checktablenumber(L, payload->y, "y", 0);
    lua_checktablenumber(L, payload->z, "z", 0);
    lua_checktablenumber(L, payload->roll, "roll", 0);
    lua_checktablenumber(L, payload->pitch, "pitch", 0);
    lua_checktablenumber(L, payload->yaw, "yaw", 0);
    lua_pushstring(L, "covariance"); // array covariance[21]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      lua_checktableinumber(L, payload->covariance[0], 1, NAN); // lua is 1 indexed
      for (int i = 1; i < 21; i++) { 
        lua_checktableinumber(L, payload->covariance[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_checktablenumber(L, payload->reset_counter, "reset_counter", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_VISION_POSITION_ESTIMATE_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_VISION_POSITION_ESTIMATE_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_VISION_SPEED_ESTIMATE: { // #103
    fmav_vision_speed_estimate_t* payload = (fmav_vision_speed_estimate_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->usec, "usec", 0);
    lua_checktablenumber(L, payload->x, "x", 0);
    lua_checktablenumber(L, payload->y, "y", 0);
    lua_checktablenumber(L, payload->z, "z", 0);
    lua_pushstring(L, "covariance"); // array covariance[9]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      lua_checktableinumber(L, payload->covariance[0], 1, NAN); // lua is 1 indexed
      for (int i = 1; i < 9; i++) { 
        lua_checktableinumber(L, payload->covariance[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_checktablenumber(L, payload->reset_counter, "reset_counter", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_VISION_SPEED_ESTIMATE_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_VISION_SPEED_ESTIMATE_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_VICON_POSITION_ESTIMATE: { // #104
    fmav_vicon_position_estimate_t* payload = (fmav_vicon_position_estimate_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->usec, "usec", 0);
    lua_checktablenumber(L, payload->x, "x", 0);
    lua_checktablenumber(L, payload->y, "y", 0);
    lua_checktablenumber(L, payload->z, "z", 0);
    lua_checktablenumber(L, payload->roll, "roll", 0);
    lua_checktablenumber(L, payload->pitch, "pitch", 0);
    lua_checktablenumber(L, payload->yaw, "yaw", 0);
    lua_pushstring(L, "covariance"); // array covariance[21]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      lua_checktableinumber(L, payload->covariance[0], 1, NAN); // lua is 1 indexed
      for (int i = 1; i < 21; i++) { 
        lua_checktableinumber(L, payload->covariance[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    msg_out->crc_extra = FASTMAVLINK_MSG_VICON_POSITION_ESTIMATE_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_VICON_POSITION_ESTIMATE_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_HIGHRES_IMU: { // #105
    fmav_highres_imu_t* payload = (fmav_highres_imu_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->time_usec, "time_usec", 0);
    lua_checktablenumber(L, payload->xacc, "xacc", 0);
    lua_checktablenumber(L, payload->yacc, "yacc", 0);
    lua_checktablenumber(L, payload->zacc, "zacc", 0);
    lua_checktablenumber(L, payload->xgyro, "xgyro", 0);
    lua_checktablenumber(L, payload->ygyro, "ygyro", 0);
    lua_checktablenumber(L, payload->zgyro, "zgyro", 0);
    lua_checktablenumber(L, payload->xmag, "xmag", 0);
    lua_checktablenumber(L, payload->ymag, "ymag", 0);
    lua_checktablenumber(L, payload->zmag, "zmag", 0);
    lua_checktablenumber(L, payload->abs_pressure, "abs_pressure", 0);
    lua_checktablenumber(L, payload->diff_pressure, "diff_pressure", 0);
    lua_checktablenumber(L, payload->pressure_alt, "pressure_alt", 0);
    lua_checktablenumber(L, payload->temperature, "temperature", 0);
    lua_checktablenumber(L, payload->fields_updated, "fields_updated", 0);
    lua_checktablenumber(L, payload->id, "id", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_HIGHRES_IMU_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_HIGHRES_IMU_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_OPTICAL_FLOW_RAD: { // #106
    fmav_optical_flow_rad_t* payload = (fmav_optical_flow_rad_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->time_usec, "time_usec", 0);
    lua_checktablenumber(L, payload->sensor_id, "sensor_id", 0);
    lua_checktablenumber(L, payload->integration_time_us, "integration_time_us", 0);
    lua_checktablenumber(L, payload->integrated_x, "integrated_x", 0);
    lua_checktablenumber(L, payload->integrated_y, "integrated_y", 0);
    lua_checktablenumber(L, payload->integrated_xgyro, "integrated_xgyro", 0);
    lua_checktablenumber(L, payload->integrated_ygyro, "integrated_ygyro", 0);
    lua_checktablenumber(L, payload->integrated_zgyro, "integrated_zgyro", 0);
    lua_checktablenumber(L, payload->temperature, "temperature", 0);
    lua_checktablenumber(L, payload->quality, "quality", 0);
    lua_checktablenumber(L, payload->time_delta_distance_us, "time_delta_distance_us", 0);
    lua_checktablenumber(L, payload->distance, "distance", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_OPTICAL_FLOW_RAD_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_OPTICAL_FLOW_RAD_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_HIL_SENSOR: { // #107
    fmav_hil_sensor_t* payload = (fmav_hil_sensor_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->time_usec, "time_usec", 0);
    lua_checktablenumber(L, payload->xacc, "xacc", 0);
    lua_checktablenumber(L, payload->yacc, "yacc", 0);
    lua_checktablenumber(L, payload->zacc, "zacc", 0);
    lua_checktablenumber(L, payload->xgyro, "xgyro", 0);
    lua_checktablenumber(L, payload->ygyro, "ygyro", 0);
    lua_checktablenumber(L, payload->zgyro, "zgyro", 0);
    lua_checktablenumber(L, payload->xmag, "xmag", 0);
    lua_checktablenumber(L, payload->ymag, "ymag", 0);
    lua_checktablenumber(L, payload->zmag, "zmag", 0);
    lua_checktablenumber(L, payload->abs_pressure, "abs_pressure", 0);
    lua_checktablenumber(L, payload->diff_pressure, "diff_pressure", 0);
    lua_checktablenumber(L, payload->pressure_alt, "pressure_alt", 0);
    lua_checktablenumber(L, payload->temperature, "temperature", 0);
    lua_checktablenumber(L, payload->fields_updated, "fields_updated", 0);
    lua_checktablenumber(L, payload->id, "id", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_HIL_SENSOR_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_HIL_SENSOR_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_SIM_STATE: { // #108
    fmav_sim_state_t* payload = (fmav_sim_state_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->q1, "q1", 0);
    lua_checktablenumber(L, payload->q2, "q2", 0);
    lua_checktablenumber(L, payload->q3, "q3", 0);
    lua_checktablenumber(L, payload->q4, "q4", 0);
    lua_checktablenumber(L, payload->roll, "roll", 0);
    lua_checktablenumber(L, payload->pitch, "pitch", 0);
    lua_checktablenumber(L, payload->yaw, "yaw", 0);
    lua_checktablenumber(L, payload->xacc, "xacc", 0);
    lua_checktablenumber(L, payload->yacc, "yacc", 0);
    lua_checktablenumber(L, payload->zacc, "zacc", 0);
    lua_checktablenumber(L, payload->xgyro, "xgyro", 0);
    lua_checktablenumber(L, payload->ygyro, "ygyro", 0);
    lua_checktablenumber(L, payload->zgyro, "zgyro", 0);
    lua_checktablenumber(L, payload->lat, "lat", 0);
    lua_checktablenumber(L, payload->lon, "lon", 0);
    lua_checktablenumber(L, payload->alt, "alt", 0);
    lua_checktablenumber(L, payload->std_dev_horz, "std_dev_horz", 0);
    lua_checktablenumber(L, payload->std_dev_vert, "std_dev_vert", 0);
    lua_checktablenumber(L, payload->vn, "vn", 0);
    lua_checktablenumber(L, payload->ve, "ve", 0);
    lua_checktablenumber(L, payload->vd, "vd", 0);
    lua_checktablenumber(L, payload->lat_int, "lat_int", 0);
    lua_checktablenumber(L, payload->lon_int, "lon_int", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_SIM_STATE_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_SIM_STATE_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_RADIO_STATUS: { // #109
    fmav_radio_status_t* payload = (fmav_radio_status_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->rssi, "rssi", UINT8_MAX);
    lua_checktablenumber(L, payload->remrssi, "remrssi", UINT8_MAX);
    lua_checktablenumber(L, payload->txbuf, "txbuf", 0);
    lua_checktablenumber(L, payload->noise, "noise", UINT8_MAX);
    lua_checktablenumber(L, payload->remnoise, "remnoise", UINT8_MAX);
    lua_checktablenumber(L, payload->rxerrors, "rxerrors", 0);
    lua_checktablenumber(L, payload->fixed, "fixed", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_RADIO_STATUS_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_RADIO_STATUS_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL: { // #110
    fmav_file_transfer_protocol_t* payload = (fmav_file_transfer_protocol_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->target_network, "target_network", 0);
    lua_checktablenumber(L, payload->target_system, "target_sysid", 0);
    lua_checktablenumber(L, payload->target_component, "target_compid", 0);
    lua_pushstring(L, "payload"); // array payload[251]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 251; i++) { 
        lua_checktableinumber(L, payload->payload[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    msg_out->target_sysid = payload->target_system;
    msg_out->target_compid = payload->target_component;
    msg_out->crc_extra = FASTMAVLINK_MSG_FILE_TRANSFER_PROTOCOL_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_FILE_TRANSFER_PROTOCOL_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_TIMESYNC: { // #111
    fmav_timesync_t* payload = (fmav_timesync_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->tc1, "tc1", 0);
    lua_checktablenumber(L, payload->ts1, "ts1", 0);
    lua_checktablenumber(L, payload->target_system, "target_sysid", 0);
    lua_checktablenumber(L, payload->target_component, "target_compid", 0);
    msg_out->target_sysid = payload->target_system;
    msg_out->target_compid = payload->target_component;
    msg_out->crc_extra = FASTMAVLINK_MSG_TIMESYNC_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_TIMESYNC_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_CAMERA_TRIGGER: { // #112
    fmav_camera_trigger_t* payload = (fmav_camera_trigger_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->time_usec, "time_usec", 0);
    lua_checktablenumber(L, payload->seq, "seq", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_CAMERA_TRIGGER_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_CAMERA_TRIGGER_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_HIL_GPS: { // #113
    fmav_hil_gps_t* payload = (fmav_hil_gps_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->time_usec, "time_usec", 0);
    lua_checktablenumber(L, payload->fix_type, "fix_type", 0);
    lua_checktablenumber(L, payload->lat, "lat", 0);
    lua_checktablenumber(L, payload->lon, "lon", 0);
    lua_checktablenumber(L, payload->alt, "alt", 0);
    lua_checktablenumber(L, payload->eph, "eph", UINT16_MAX);
    lua_checktablenumber(L, payload->epv, "epv", UINT16_MAX);
    lua_checktablenumber(L, payload->vel, "vel", UINT16_MAX);
    lua_checktablenumber(L, payload->vn, "vn", 0);
    lua_checktablenumber(L, payload->ve, "ve", 0);
    lua_checktablenumber(L, payload->vd, "vd", 0);
    lua_checktablenumber(L, payload->cog, "cog", UINT16_MAX);
    lua_checktablenumber(L, payload->satellites_visible, "satellites_visible", UINT8_MAX);
    lua_checktablenumber(L, payload->id, "id", 0);
    lua_checktablenumber(L, payload->yaw, "yaw", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_HIL_GPS_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_HIL_GPS_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_HIL_OPTICAL_FLOW: { // #114
    fmav_hil_optical_flow_t* payload = (fmav_hil_optical_flow_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->time_usec, "time_usec", 0);
    lua_checktablenumber(L, payload->sensor_id, "sensor_id", 0);
    lua_checktablenumber(L, payload->integration_time_us, "integration_time_us", 0);
    lua_checktablenumber(L, payload->integrated_x, "integrated_x", 0);
    lua_checktablenumber(L, payload->integrated_y, "integrated_y", 0);
    lua_checktablenumber(L, payload->integrated_xgyro, "integrated_xgyro", 0);
    lua_checktablenumber(L, payload->integrated_ygyro, "integrated_ygyro", 0);
    lua_checktablenumber(L, payload->integrated_zgyro, "integrated_zgyro", 0);
    lua_checktablenumber(L, payload->temperature, "temperature", 0);
    lua_checktablenumber(L, payload->quality, "quality", 0);
    lua_checktablenumber(L, payload->time_delta_distance_us, "time_delta_distance_us", 0);
    lua_checktablenumber(L, payload->distance, "distance", -1.0);
    msg_out->crc_extra = FASTMAVLINK_MSG_HIL_OPTICAL_FLOW_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_HIL_OPTICAL_FLOW_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_HIL_STATE_QUATERNION: { // #115
    fmav_hil_state_quaternion_t* payload = (fmav_hil_state_quaternion_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->time_usec, "time_usec", 0);
    lua_pushstring(L, "attitude_quaternion"); // array attitude_quaternion[4]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 4; i++) { 
        lua_checktableinumber(L, payload->attitude_quaternion[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_checktablenumber(L, payload->rollspeed, "rollspeed", 0);
    lua_checktablenumber(L, payload->pitchspeed, "pitchspeed", 0);
    lua_checktablenumber(L, payload->yawspeed, "yawspeed", 0);
    lua_checktablenumber(L, payload->lat, "lat", 0);
    lua_checktablenumber(L, payload->lon, "lon", 0);
    lua_checktablenumber(L, payload->alt, "alt", 0);
    lua_checktablenumber(L, payload->vx, "vx", 0);
    lua_checktablenumber(L, payload->vy, "vy", 0);
    lua_checktablenumber(L, payload->vz, "vz", 0);
    lua_checktablenumber(L, payload->ind_airspeed, "ind_airspeed", 0);
    lua_checktablenumber(L, payload->true_airspeed, "true_airspeed", 0);
    lua_checktablenumber(L, payload->xacc, "xacc", 0);
    lua_checktablenumber(L, payload->yacc, "yacc", 0);
    lua_checktablenumber(L, payload->zacc, "zacc", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_HIL_STATE_QUATERNION_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_HIL_STATE_QUATERNION_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_SCALED_IMU2: { // #116
    fmav_scaled_imu2_t* payload = (fmav_scaled_imu2_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->time_boot_ms, "time_boot_ms", 0);
    lua_checktablenumber(L, payload->xacc, "xacc", 0);
    lua_checktablenumber(L, payload->yacc, "yacc", 0);
    lua_checktablenumber(L, payload->zacc, "zacc", 0);
    lua_checktablenumber(L, payload->xgyro, "xgyro", 0);
    lua_checktablenumber(L, payload->ygyro, "ygyro", 0);
    lua_checktablenumber(L, payload->zgyro, "zgyro", 0);
    lua_checktablenumber(L, payload->xmag, "xmag", 0);
    lua_checktablenumber(L, payload->ymag, "ymag", 0);
    lua_checktablenumber(L, payload->zmag, "zmag", 0);
    lua_checktablenumber(L, payload->temperature, "temperature", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_SCALED_IMU2_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_SCALED_IMU2_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_LOG_REQUEST_LIST: { // #117
    fmav_log_request_list_t* payload = (fmav_log_request_list_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->target_system, "target_sysid", 0);
    lua_checktablenumber(L, payload->target_component, "target_compid", 0);
    lua_checktablenumber(L, payload->start, "start", 0);
    lua_checktablenumber(L, payload->end, "end", 0);
    msg_out->target_sysid = payload->target_system;
    msg_out->target_compid = payload->target_component;
    msg_out->crc_extra = FASTMAVLINK_MSG_LOG_REQUEST_LIST_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_LOG_REQUEST_LIST_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_LOG_ENTRY: { // #118
    fmav_log_entry_t* payload = (fmav_log_entry_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->id, "id", 0);
    lua_checktablenumber(L, payload->num_logs, "num_logs", 0);
    lua_checktablenumber(L, payload->last_log_num, "last_log_num", 0);
    lua_checktablenumber(L, payload->time_utc, "time_utc", 0);
    lua_checktablenumber(L, payload->size, "size", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_LOG_ENTRY_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_LOG_ENTRY_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_LOG_REQUEST_DATA: { // #119
    fmav_log_request_data_t* payload = (fmav_log_request_data_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->target_system, "target_sysid", 0);
    lua_checktablenumber(L, payload->target_component, "target_compid", 0);
    lua_checktablenumber(L, payload->id, "id", 0);
    lua_checktablenumber(L, payload->ofs, "ofs", 0);
    lua_checktablenumber(L, payload->count, "count", 0);
    msg_out->target_sysid = payload->target_system;
    msg_out->target_compid = payload->target_component;
    msg_out->crc_extra = FASTMAVLINK_MSG_LOG_REQUEST_DATA_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_LOG_REQUEST_DATA_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_LOG_DATA: { // #120
    fmav_log_data_t* payload = (fmav_log_data_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->id, "id", 0);
    lua_checktablenumber(L, payload->ofs, "ofs", 0);
    lua_checktablenumber(L, payload->count, "count", 0);
    lua_pushstring(L, "data"); // array data[90]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 90; i++) { 
        lua_checktableinumber(L, payload->data[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    msg_out->crc_extra = FASTMAVLINK_MSG_LOG_DATA_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_LOG_DATA_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_LOG_ERASE: { // #121
    fmav_log_erase_t* payload = (fmav_log_erase_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->target_system, "target_sysid", 0);
    lua_checktablenumber(L, payload->target_component, "target_compid", 0);
    msg_out->target_sysid = payload->target_system;
    msg_out->target_compid = payload->target_component;
    msg_out->crc_extra = FASTMAVLINK_MSG_LOG_ERASE_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_LOG_ERASE_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_LOG_REQUEST_END: { // #122
    fmav_log_request_end_t* payload = (fmav_log_request_end_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->target_system, "target_sysid", 0);
    lua_checktablenumber(L, payload->target_component, "target_compid", 0);
    msg_out->target_sysid = payload->target_system;
    msg_out->target_compid = payload->target_component;
    msg_out->crc_extra = FASTMAVLINK_MSG_LOG_REQUEST_END_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_LOG_REQUEST_END_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_GPS_INJECT_DATA: { // #123
    fmav_gps_inject_data_t* payload = (fmav_gps_inject_data_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->target_system, "target_sysid", 0);
    lua_checktablenumber(L, payload->target_component, "target_compid", 0);
    lua_checktablenumber(L, payload->len, "len", 0);
    lua_pushstring(L, "data"); // array data[110]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 110; i++) { 
        lua_checktableinumber(L, payload->data[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    msg_out->target_sysid = payload->target_system;
    msg_out->target_compid = payload->target_component;
    msg_out->crc_extra = FASTMAVLINK_MSG_GPS_INJECT_DATA_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_GPS_INJECT_DATA_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_GPS2_RAW: { // #124
    fmav_gps2_raw_t* payload = (fmav_gps2_raw_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->time_usec, "time_usec", 0);
    lua_checktablenumber(L, payload->fix_type, "fix_type", 0);
    lua_checktablenumber(L, payload->lat, "lat", 0);
    lua_checktablenumber(L, payload->lon, "lon", 0);
    lua_checktablenumber(L, payload->alt, "alt", 0);
    lua_checktablenumber(L, payload->eph, "eph", UINT16_MAX);
    lua_checktablenumber(L, payload->epv, "epv", UINT16_MAX);
    lua_checktablenumber(L, payload->vel, "vel", UINT16_MAX);
    lua_checktablenumber(L, payload->cog, "cog", UINT16_MAX);
    lua_checktablenumber(L, payload->satellites_visible, "satellites_visible", UINT8_MAX);
    lua_checktablenumber(L, payload->dgps_numch, "dgps_numch", 0);
    lua_checktablenumber(L, payload->dgps_age, "dgps_age", 0);
    lua_checktablenumber(L, payload->yaw, "yaw", 0);
    lua_checktablenumber(L, payload->alt_ellipsoid, "alt_ellipsoid", 0);
    lua_checktablenumber(L, payload->h_acc, "h_acc", 0);
    lua_checktablenumber(L, payload->v_acc, "v_acc", 0);
    lua_checktablenumber(L, payload->vel_acc, "vel_acc", 0);
    lua_checktablenumber(L, payload->hdg_acc, "hdg_acc", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_GPS2_RAW_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_GPS2_RAW_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_POWER_STATUS: { // #125
    fmav_power_status_t* payload = (fmav_power_status_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->Vcc, "Vcc", 0);
    lua_checktablenumber(L, payload->Vservo, "Vservo", 0);
    lua_checktablenumber(L, payload->flags, "flags", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_POWER_STATUS_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_POWER_STATUS_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_SERIAL_CONTROL: { // #126
    fmav_serial_control_t* payload = (fmav_serial_control_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->device, "device", 0);
    lua_checktablenumber(L, payload->flags, "flags", 0);
    lua_checktablenumber(L, payload->timeout, "timeout", 0);
    lua_checktablenumber(L, payload->baudrate, "baudrate", 0);
    lua_checktablenumber(L, payload->count, "count", 0);
    lua_pushstring(L, "data"); // array data[70]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 70; i++) { 
        lua_checktableinumber(L, payload->data[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_checktablenumber(L, payload->target_system, "target_sysid", 0);
    lua_checktablenumber(L, payload->target_component, "target_compid", 0);
    msg_out->target_sysid = payload->target_system;
    msg_out->target_compid = payload->target_component;
    msg_out->crc_extra = FASTMAVLINK_MSG_SERIAL_CONTROL_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_SERIAL_CONTROL_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_GPS_RTK: { // #127
    fmav_gps_rtk_t* payload = (fmav_gps_rtk_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->time_last_baseline_ms, "time_last_baseline_ms", 0);
    lua_checktablenumber(L, payload->rtk_receiver_id, "rtk_receiver_id", 0);
    lua_checktablenumber(L, payload->wn, "wn", 0);
    lua_checktablenumber(L, payload->tow, "tow", 0);
    lua_checktablenumber(L, payload->rtk_health, "rtk_health", 0);
    lua_checktablenumber(L, payload->rtk_rate, "rtk_rate", 0);
    lua_checktablenumber(L, payload->nsats, "nsats", 0);
    lua_checktablenumber(L, payload->baseline_coords_type, "baseline_coords_type", 0);
    lua_checktablenumber(L, payload->baseline_a_mm, "baseline_a_mm", 0);
    lua_checktablenumber(L, payload->baseline_b_mm, "baseline_b_mm", 0);
    lua_checktablenumber(L, payload->baseline_c_mm, "baseline_c_mm", 0);
    lua_checktablenumber(L, payload->accuracy, "accuracy", 0);
    lua_checktablenumber(L, payload->iar_num_hypotheses, "iar_num_hypotheses", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_GPS_RTK_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_GPS_RTK_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_GPS2_RTK: { // #128
    fmav_gps2_rtk_t* payload = (fmav_gps2_rtk_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->time_last_baseline_ms, "time_last_baseline_ms", 0);
    lua_checktablenumber(L, payload->rtk_receiver_id, "rtk_receiver_id", 0);
    lua_checktablenumber(L, payload->wn, "wn", 0);
    lua_checktablenumber(L, payload->tow, "tow", 0);
    lua_checktablenumber(L, payload->rtk_health, "rtk_health", 0);
    lua_checktablenumber(L, payload->rtk_rate, "rtk_rate", 0);
    lua_checktablenumber(L, payload->nsats, "nsats", 0);
    lua_checktablenumber(L, payload->baseline_coords_type, "baseline_coords_type", 0);
    lua_checktablenumber(L, payload->baseline_a_mm, "baseline_a_mm", 0);
    lua_checktablenumber(L, payload->baseline_b_mm, "baseline_b_mm", 0);
    lua_checktablenumber(L, payload->baseline_c_mm, "baseline_c_mm", 0);
    lua_checktablenumber(L, payload->accuracy, "accuracy", 0);
    lua_checktablenumber(L, payload->iar_num_hypotheses, "iar_num_hypotheses", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_GPS2_RTK_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_GPS2_RTK_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_SCALED_IMU3: { // #129
    fmav_scaled_imu3_t* payload = (fmav_scaled_imu3_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->time_boot_ms, "time_boot_ms", 0);
    lua_checktablenumber(L, payload->xacc, "xacc", 0);
    lua_checktablenumber(L, payload->yacc, "yacc", 0);
    lua_checktablenumber(L, payload->zacc, "zacc", 0);
    lua_checktablenumber(L, payload->xgyro, "xgyro", 0);
    lua_checktablenumber(L, payload->ygyro, "ygyro", 0);
    lua_checktablenumber(L, payload->zgyro, "zgyro", 0);
    lua_checktablenumber(L, payload->xmag, "xmag", 0);
    lua_checktablenumber(L, payload->ymag, "ymag", 0);
    lua_checktablenumber(L, payload->zmag, "zmag", 0);
    lua_checktablenumber(L, payload->temperature, "temperature", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_SCALED_IMU3_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_SCALED_IMU3_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_DATA_TRANSMISSION_HANDSHAKE: { // #130
    fmav_data_transmission_handshake_t* payload = (fmav_data_transmission_handshake_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->type, "type", 0);
    lua_checktablenumber(L, payload->size, "size", 0);
    lua_checktablenumber(L, payload->width, "width", 0);
    lua_checktablenumber(L, payload->height, "height", 0);
    lua_checktablenumber(L, payload->packets, "packets", 0);
    lua_checktablenumber(L, payload->payload, "payload", 0);
    lua_checktablenumber(L, payload->jpg_quality, "jpg_quality", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_DATA_TRANSMISSION_HANDSHAKE_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_DATA_TRANSMISSION_HANDSHAKE_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_ENCAPSULATED_DATA: { // #131
    fmav_encapsulated_data_t* payload = (fmav_encapsulated_data_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->seqnr, "seqnr", 0);
    lua_pushstring(L, "data"); // array data[253]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 253; i++) { 
        lua_checktableinumber(L, payload->data[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    msg_out->crc_extra = FASTMAVLINK_MSG_ENCAPSULATED_DATA_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_ENCAPSULATED_DATA_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_DISTANCE_SENSOR: { // #132
    fmav_distance_sensor_t* payload = (fmav_distance_sensor_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->time_boot_ms, "time_boot_ms", 0);
    lua_checktablenumber(L, payload->min_distance, "min_distance", 0);
    lua_checktablenumber(L, payload->max_distance, "max_distance", 0);
    lua_checktablenumber(L, payload->current_distance, "current_distance", 0);
    lua_checktablenumber(L, payload->type, "type", 0);
    lua_checktablenumber(L, payload->id, "id", 0);
    lua_checktablenumber(L, payload->orientation, "orientation", 0);
    lua_checktablenumber(L, payload->covariance, "covariance", UINT8_MAX);
    lua_checktablenumber(L, payload->horizontal_fov, "horizontal_fov", 0);
    lua_checktablenumber(L, payload->vertical_fov, "vertical_fov", 0);
    lua_pushstring(L, "quaternion"); // array quaternion[4]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 4; i++) { 
        lua_checktableinumber(L, payload->quaternion[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_checktablenumber(L, payload->signal_quality, "signal_quality", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_DISTANCE_SENSOR_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_DISTANCE_SENSOR_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_TERRAIN_REQUEST: { // #133
    fmav_terrain_request_t* payload = (fmav_terrain_request_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->lat, "lat", 0);
    lua_checktablenumber(L, payload->lon, "lon", 0);
    lua_checktablenumber(L, payload->grid_spacing, "grid_spacing", 0);
    lua_checktablenumber(L, payload->mask, "mask", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_TERRAIN_REQUEST_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_TERRAIN_REQUEST_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_TERRAIN_DATA: { // #134
    fmav_terrain_data_t* payload = (fmav_terrain_data_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->lat, "lat", 0);
    lua_checktablenumber(L, payload->lon, "lon", 0);
    lua_checktablenumber(L, payload->grid_spacing, "grid_spacing", 0);
    lua_checktablenumber(L, payload->gridbit, "gridbit", 0);
    lua_pushstring(L, "data"); // array data[16]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 16; i++) { 
        lua_checktableinumber(L, payload->data[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    msg_out->crc_extra = FASTMAVLINK_MSG_TERRAIN_DATA_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_TERRAIN_DATA_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_TERRAIN_CHECK: { // #135
    fmav_terrain_check_t* payload = (fmav_terrain_check_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->lat, "lat", 0);
    lua_checktablenumber(L, payload->lon, "lon", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_TERRAIN_CHECK_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_TERRAIN_CHECK_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_TERRAIN_REPORT: { // #136
    fmav_terrain_report_t* payload = (fmav_terrain_report_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->lat, "lat", 0);
    lua_checktablenumber(L, payload->lon, "lon", 0);
    lua_checktablenumber(L, payload->spacing, "spacing", 0);
    lua_checktablenumber(L, payload->terrain_height, "terrain_height", 0);
    lua_checktablenumber(L, payload->current_height, "current_height", 0);
    lua_checktablenumber(L, payload->pending, "pending", 0);
    lua_checktablenumber(L, payload->loaded, "loaded", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_TERRAIN_REPORT_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_TERRAIN_REPORT_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_SCALED_PRESSURE2: { // #137
    fmav_scaled_pressure2_t* payload = (fmav_scaled_pressure2_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->time_boot_ms, "time_boot_ms", 0);
    lua_checktablenumber(L, payload->press_abs, "press_abs", 0);
    lua_checktablenumber(L, payload->press_diff, "press_diff", 0);
    lua_checktablenumber(L, payload->temperature, "temperature", 0);
    lua_checktablenumber(L, payload->temperature_press_diff, "temperature_press_diff", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_SCALED_PRESSURE2_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_SCALED_PRESSURE2_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_ATT_POS_MOCAP: { // #138
    fmav_att_pos_mocap_t* payload = (fmav_att_pos_mocap_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->time_usec, "time_usec", 0);
    lua_pushstring(L, "q"); // array q[4]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 4; i++) { 
        lua_checktableinumber(L, payload->q[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_checktablenumber(L, payload->x, "x", 0);
    lua_checktablenumber(L, payload->y, "y", 0);
    lua_checktablenumber(L, payload->z, "z", 0);
    lua_pushstring(L, "covariance"); // array covariance[21]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      lua_checktableinumber(L, payload->covariance[0], 1, NAN); // lua is 1 indexed
      for (int i = 1; i < 21; i++) { 
        lua_checktableinumber(L, payload->covariance[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    msg_out->crc_extra = FASTMAVLINK_MSG_ATT_POS_MOCAP_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_ATT_POS_MOCAP_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_SET_ACTUATOR_CONTROL_TARGET: { // #139
    fmav_set_actuator_control_target_t* payload = (fmav_set_actuator_control_target_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->time_usec, "time_usec", 0);
    lua_checktablenumber(L, payload->group_mlx, "group_mlx", 0);
    lua_checktablenumber(L, payload->target_system, "target_sysid", 0);
    lua_checktablenumber(L, payload->target_component, "target_compid", 0);
    lua_pushstring(L, "controls"); // array controls[8]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 8; i++) { 
        lua_checktableinumber(L, payload->controls[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    msg_out->target_sysid = payload->target_system;
    msg_out->target_compid = payload->target_component;
    msg_out->crc_extra = FASTMAVLINK_MSG_SET_ACTUATOR_CONTROL_TARGET_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_SET_ACTUATOR_CONTROL_TARGET_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_ACTUATOR_CONTROL_TARGET: { // #140
    fmav_actuator_control_target_t* payload = (fmav_actuator_control_target_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->time_usec, "time_usec", 0);
    lua_checktablenumber(L, payload->group_mlx, "group_mlx", 0);
    lua_pushstring(L, "controls"); // array controls[8]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 8; i++) { 
        lua_checktableinumber(L, payload->controls[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    msg_out->crc_extra = FASTMAVLINK_MSG_ACTUATOR_CONTROL_TARGET_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_ACTUATOR_CONTROL_TARGET_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_ALTITUDE: { // #141
    fmav_altitude_t* payload = (fmav_altitude_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->time_usec, "time_usec", 0);
    lua_checktablenumber(L, payload->altitude_monotonic, "altitude_monotonic", 0);
    lua_checktablenumber(L, payload->altitude_amsl, "altitude_amsl", 0);
    lua_checktablenumber(L, payload->altitude_local, "altitude_local", 0);
    lua_checktablenumber(L, payload->altitude_relative, "altitude_relative", 0);
    lua_checktablenumber(L, payload->altitude_terrain, "altitude_terrain", 0);
    lua_checktablenumber(L, payload->bottom_clearance, "bottom_clearance", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_ALTITUDE_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_ALTITUDE_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_RESOURCE_REQUEST: { // #142
    fmav_resource_request_t* payload = (fmav_resource_request_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->request_id, "request_id", 0);
    lua_checktablenumber(L, payload->uri_type, "uri_type", 0);
    lua_pushstring(L, "uri"); // array uri[120]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 120; i++) { 
        lua_checktableinumber(L, payload->uri[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_checktablenumber(L, payload->transfer_type, "transfer_type", 0);
    lua_pushstring(L, "storage"); // array storage[120]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 120; i++) { 
        lua_checktableinumber(L, payload->storage[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    msg_out->crc_extra = FASTMAVLINK_MSG_RESOURCE_REQUEST_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_RESOURCE_REQUEST_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_SCALED_PRESSURE3: { // #143
    fmav_scaled_pressure3_t* payload = (fmav_scaled_pressure3_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->time_boot_ms, "time_boot_ms", 0);
    lua_checktablenumber(L, payload->press_abs, "press_abs", 0);
    lua_checktablenumber(L, payload->press_diff, "press_diff", 0);
    lua_checktablenumber(L, payload->temperature, "temperature", 0);
    lua_checktablenumber(L, payload->temperature_press_diff, "temperature_press_diff", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_SCALED_PRESSURE3_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_SCALED_PRESSURE3_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_FOLLOW_TARGET: { // #144
    fmav_follow_target_t* payload = (fmav_follow_target_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->timestamp, "timestamp", 0);
    lua_checktablenumber(L, payload->est_capabilities, "est_capabilities", 0);
    lua_checktablenumber(L, payload->lat, "lat", 0);
    lua_checktablenumber(L, payload->lon, "lon", 0);
    lua_checktablenumber(L, payload->alt, "alt", 0);
    lua_pushstring(L, "vel"); // array vel[3]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 3; i++) { 
        lua_checktableinumber(L, payload->vel[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_pushstring(L, "acc"); // array acc[3]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 3; i++) { 
        lua_checktableinumber(L, payload->acc[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_pushstring(L, "attitude_q"); // array attitude_q[4]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 4; i++) { 
        lua_checktableinumber(L, payload->attitude_q[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_pushstring(L, "rates"); // array rates[3]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 3; i++) { 
        lua_checktableinumber(L, payload->rates[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_pushstring(L, "position_cov"); // array position_cov[3]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 3; i++) { 
        lua_checktableinumber(L, payload->position_cov[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_checktablenumber(L, payload->custom_state, "custom_state", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_FOLLOW_TARGET_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_FOLLOW_TARGET_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_CONTROL_SYSTEM_STATE: { // #146
    fmav_control_system_state_t* payload = (fmav_control_system_state_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->time_usec, "time_usec", 0);
    lua_checktablenumber(L, payload->x_acc, "x_acc", 0);
    lua_checktablenumber(L, payload->y_acc, "y_acc", 0);
    lua_checktablenumber(L, payload->z_acc, "z_acc", 0);
    lua_checktablenumber(L, payload->x_vel, "x_vel", 0);
    lua_checktablenumber(L, payload->y_vel, "y_vel", 0);
    lua_checktablenumber(L, payload->z_vel, "z_vel", 0);
    lua_checktablenumber(L, payload->x_pos, "x_pos", 0);
    lua_checktablenumber(L, payload->y_pos, "y_pos", 0);
    lua_checktablenumber(L, payload->z_pos, "z_pos", 0);
    lua_checktablenumber(L, payload->airspeed, "airspeed", -1);
    lua_pushstring(L, "vel_variance"); // array vel_variance[3]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 3; i++) { 
        lua_checktableinumber(L, payload->vel_variance[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_pushstring(L, "pos_variance"); // array pos_variance[3]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 3; i++) { 
        lua_checktableinumber(L, payload->pos_variance[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_pushstring(L, "q"); // array q[4]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 4; i++) { 
        lua_checktableinumber(L, payload->q[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_checktablenumber(L, payload->roll_rate, "roll_rate", 0);
    lua_checktablenumber(L, payload->pitch_rate, "pitch_rate", 0);
    lua_checktablenumber(L, payload->yaw_rate, "yaw_rate", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_CONTROL_SYSTEM_STATE_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_CONTROL_SYSTEM_STATE_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_BATTERY_STATUS: { // #147
    fmav_battery_status_t* payload = (fmav_battery_status_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->id, "id", 0);
    lua_checktablenumber(L, payload->battery_function, "battery_function", 0);
    lua_checktablenumber(L, payload->type, "type", 0);
    lua_checktablenumber(L, payload->temperature, "temperature", INT16_MAX);
    lua_pushstring(L, "voltages"); // array voltages[10]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 10; i++) { 
        lua_checktableinumber(L, payload->voltages[i], i+1, UINT16_MAX); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_checktablenumber(L, payload->current_battery, "current_battery", -1);
    lua_checktablenumber(L, payload->current_consumed, "current_consumed", -1);
    lua_checktablenumber(L, payload->energy_consumed, "energy_consumed", -1);
    lua_checktablenumber(L, payload->battery_remaining, "battery_remaining", -1);
    lua_checktablenumber(L, payload->time_remaining, "time_remaining", 0);
    lua_checktablenumber(L, payload->charge_state, "charge_state", 0);
    lua_pushstring(L, "voltages_ext"); // array voltages_ext[4]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 4; i++) { 
        lua_checktableinumber(L, payload->voltages_ext[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_checktablenumber(L, payload->mode, "mode", 0);
    lua_checktablenumber(L, payload->fault_bitmask, "fault_bitmask", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_BATTERY_STATUS_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_BATTERY_STATUS_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_AUTOPILOT_VERSION: { // #148
    fmav_autopilot_version_t* payload = (fmav_autopilot_version_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->capabilities, "capabilities", 0);
    lua_checktablenumber(L, payload->flight_sw_version, "flight_sw_version", 0);
    lua_checktablenumber(L, payload->middleware_sw_version, "middleware_sw_version", 0);
    lua_checktablenumber(L, payload->os_sw_version, "os_sw_version", 0);
    lua_checktablenumber(L, payload->board_version, "board_version", 0);
    lua_pushstring(L, "flight_custom_version"); // array flight_custom_version[8]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 8; i++) { 
        lua_checktableinumber(L, payload->flight_custom_version[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_pushstring(L, "middleware_custom_version"); // array middleware_custom_version[8]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 8; i++) { 
        lua_checktableinumber(L, payload->middleware_custom_version[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_pushstring(L, "os_custom_version"); // array os_custom_version[8]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 8; i++) { 
        lua_checktableinumber(L, payload->os_custom_version[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_checktablenumber(L, payload->vendor_id, "vendor_id", 0);
    lua_checktablenumber(L, payload->product_id, "product_id", 0);
    lua_checktablenumber(L, payload->uid, "uid", 0);
    lua_pushstring(L, "uid2"); // array uid2[18]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 18; i++) { 
        lua_checktableinumber(L, payload->uid2[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    msg_out->crc_extra = FASTMAVLINK_MSG_AUTOPILOT_VERSION_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_AUTOPILOT_VERSION_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_LANDING_TARGET: { // #149
    fmav_landing_target_t* payload = (fmav_landing_target_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->time_usec, "time_usec", 0);
    lua_checktablenumber(L, payload->target_num, "target_num", 0);
    lua_checktablenumber(L, payload->frame, "frame", 0);
    lua_checktablenumber(L, payload->angle_x, "angle_x", 0);
    lua_checktablenumber(L, payload->angle_y, "angle_y", 0);
    lua_checktablenumber(L, payload->distance, "distance", 0);
    lua_checktablenumber(L, payload->size_x, "size_x", 0);
    lua_checktablenumber(L, payload->size_y, "size_y", 0);
    lua_checktablenumber(L, payload->x, "x", 0);
    lua_checktablenumber(L, payload->y, "y", 0);
    lua_checktablenumber(L, payload->z, "z", 0);
    lua_pushstring(L, "q"); // array q[4]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 4; i++) { 
        lua_checktableinumber(L, payload->q[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_checktablenumber(L, payload->type, "type", 0);
    lua_checktablenumber(L, payload->position_valid, "position_valid", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_LANDING_TARGET_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_LANDING_TARGET_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_SENSOR_OFFSETS: { // #150
    fmav_sensor_offsets_t* payload = (fmav_sensor_offsets_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->mag_ofs_x, "mag_ofs_x", 0);
    lua_checktablenumber(L, payload->mag_ofs_y, "mag_ofs_y", 0);
    lua_checktablenumber(L, payload->mag_ofs_z, "mag_ofs_z", 0);
    lua_checktablenumber(L, payload->mag_declination, "mag_declination", 0);
    lua_checktablenumber(L, payload->raw_press, "raw_press", 0);
    lua_checktablenumber(L, payload->raw_temp, "raw_temp", 0);
    lua_checktablenumber(L, payload->gyro_cal_x, "gyro_cal_x", 0);
    lua_checktablenumber(L, payload->gyro_cal_y, "gyro_cal_y", 0);
    lua_checktablenumber(L, payload->gyro_cal_z, "gyro_cal_z", 0);
    lua_checktablenumber(L, payload->accel_cal_x, "accel_cal_x", 0);
    lua_checktablenumber(L, payload->accel_cal_y, "accel_cal_y", 0);
    lua_checktablenumber(L, payload->accel_cal_z, "accel_cal_z", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_SENSOR_OFFSETS_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_SENSOR_OFFSETS_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_SET_MAG_OFFSETS: { // #151
    fmav_set_mag_offsets_t* payload = (fmav_set_mag_offsets_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->target_system, "target_sysid", 0);
    lua_checktablenumber(L, payload->target_component, "target_compid", 0);
    lua_checktablenumber(L, payload->mag_ofs_x, "mag_ofs_x", 0);
    lua_checktablenumber(L, payload->mag_ofs_y, "mag_ofs_y", 0);
    lua_checktablenumber(L, payload->mag_ofs_z, "mag_ofs_z", 0);
    msg_out->target_sysid = payload->target_system;
    msg_out->target_compid = payload->target_component;
    msg_out->crc_extra = FASTMAVLINK_MSG_SET_MAG_OFFSETS_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_SET_MAG_OFFSETS_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_MEMINFO: { // #152
    fmav_meminfo_t* payload = (fmav_meminfo_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->brkval, "brkval", 0);
    lua_checktablenumber(L, payload->freemem, "freemem", 0);
    lua_checktablenumber(L, payload->freemem32, "freemem32", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_MEMINFO_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_MEMINFO_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_AP_ADC: { // #153
    fmav_ap_adc_t* payload = (fmav_ap_adc_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->adc1, "adc1", 0);
    lua_checktablenumber(L, payload->adc2, "adc2", 0);
    lua_checktablenumber(L, payload->adc3, "adc3", 0);
    lua_checktablenumber(L, payload->adc4, "adc4", 0);
    lua_checktablenumber(L, payload->adc5, "adc5", 0);
    lua_checktablenumber(L, payload->adc6, "adc6", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_AP_ADC_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_AP_ADC_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_DIGICAM_CONFIGURE: { // #154
    fmav_digicam_configure_t* payload = (fmav_digicam_configure_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->target_system, "target_sysid", 0);
    lua_checktablenumber(L, payload->target_component, "target_compid", 0);
    lua_checktablenumber(L, payload->mode, "mode", 0);
    lua_checktablenumber(L, payload->shutter_speed, "shutter_speed", 0);
    lua_checktablenumber(L, payload->aperture, "aperture", 0);
    lua_checktablenumber(L, payload->iso, "iso", 0);
    lua_checktablenumber(L, payload->exposure_type, "exposure_type", 0);
    lua_checktablenumber(L, payload->command_id, "command_id", 0);
    lua_checktablenumber(L, payload->engine_cut_off, "engine_cut_off", 0);
    lua_checktablenumber(L, payload->extra_param, "extra_param", 0);
    lua_checktablenumber(L, payload->extra_value, "extra_value", 0);
    msg_out->target_sysid = payload->target_system;
    msg_out->target_compid = payload->target_component;
    msg_out->crc_extra = FASTMAVLINK_MSG_DIGICAM_CONFIGURE_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_DIGICAM_CONFIGURE_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_DIGICAM_CONTROL: { // #155
    fmav_digicam_control_t* payload = (fmav_digicam_control_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->target_system, "target_sysid", 0);
    lua_checktablenumber(L, payload->target_component, "target_compid", 0);
    lua_checktablenumber(L, payload->session, "session", 0);
    lua_checktablenumber(L, payload->zoom_pos, "zoom_pos", 0);
    lua_checktablenumber(L, payload->zoom_step, "zoom_step", 0);
    lua_checktablenumber(L, payload->focus_lock, "focus_lock", 0);
    lua_checktablenumber(L, payload->shot, "shot", 0);
    lua_checktablenumber(L, payload->command_id, "command_id", 0);
    lua_checktablenumber(L, payload->extra_param, "extra_param", 0);
    lua_checktablenumber(L, payload->extra_value, "extra_value", 0);
    msg_out->target_sysid = payload->target_system;
    msg_out->target_compid = payload->target_component;
    msg_out->crc_extra = FASTMAVLINK_MSG_DIGICAM_CONTROL_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_DIGICAM_CONTROL_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_MOUNT_CONFIGURE: { // #156
    fmav_mount_configure_t* payload = (fmav_mount_configure_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->target_system, "target_sysid", 0);
    lua_checktablenumber(L, payload->target_component, "target_compid", 0);
    lua_checktablenumber(L, payload->mount_mode, "mount_mode", 0);
    lua_checktablenumber(L, payload->stab_roll, "stab_roll", 0);
    lua_checktablenumber(L, payload->stab_pitch, "stab_pitch", 0);
    lua_checktablenumber(L, payload->stab_yaw, "stab_yaw", 0);
    msg_out->target_sysid = payload->target_system;
    msg_out->target_compid = payload->target_component;
    msg_out->crc_extra = FASTMAVLINK_MSG_MOUNT_CONFIGURE_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_MOUNT_CONFIGURE_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_MOUNT_CONTROL: { // #157
    fmav_mount_control_t* payload = (fmav_mount_control_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->target_system, "target_sysid", 0);
    lua_checktablenumber(L, payload->target_component, "target_compid", 0);
    lua_checktablenumber(L, payload->input_a, "input_a", 0);
    lua_checktablenumber(L, payload->input_b, "input_b", 0);
    lua_checktablenumber(L, payload->input_c, "input_c", 0);
    lua_checktablenumber(L, payload->save_position, "save_position", 0);
    msg_out->target_sysid = payload->target_system;
    msg_out->target_compid = payload->target_component;
    msg_out->crc_extra = FASTMAVLINK_MSG_MOUNT_CONTROL_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_MOUNT_CONTROL_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_MOUNT_STATUS: { // #158
    fmav_mount_status_t* payload = (fmav_mount_status_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->target_system, "target_sysid", 0);
    lua_checktablenumber(L, payload->target_component, "target_compid", 0);
    lua_checktablenumber(L, payload->pointing_a, "pointing_a", 0);
    lua_checktablenumber(L, payload->pointing_b, "pointing_b", 0);
    lua_checktablenumber(L, payload->pointing_c, "pointing_c", 0);
    lua_checktablenumber(L, payload->mount_mode, "mount_mode", 0);
    msg_out->target_sysid = payload->target_system;
    msg_out->target_compid = payload->target_component;
    msg_out->crc_extra = FASTMAVLINK_MSG_MOUNT_STATUS_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_MOUNT_STATUS_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_FENCE_POINT: { // #160
    fmav_fence_point_t* payload = (fmav_fence_point_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->target_system, "target_sysid", 0);
    lua_checktablenumber(L, payload->target_component, "target_compid", 0);
    lua_checktablenumber(L, payload->idx, "idx", 0);
    lua_checktablenumber(L, payload->count, "count", 0);
    lua_checktablenumber(L, payload->lat, "lat", 0);
    lua_checktablenumber(L, payload->lng, "lng", 0);
    msg_out->target_sysid = payload->target_system;
    msg_out->target_compid = payload->target_component;
    msg_out->crc_extra = FASTMAVLINK_MSG_FENCE_POINT_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_FENCE_POINT_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_FENCE_FETCH_POINT: { // #161
    fmav_fence_fetch_point_t* payload = (fmav_fence_fetch_point_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->target_system, "target_sysid", 0);
    lua_checktablenumber(L, payload->target_component, "target_compid", 0);
    lua_checktablenumber(L, payload->idx, "idx", 0);
    msg_out->target_sysid = payload->target_system;
    msg_out->target_compid = payload->target_component;
    msg_out->crc_extra = FASTMAVLINK_MSG_FENCE_FETCH_POINT_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_FENCE_FETCH_POINT_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_FENCE_STATUS: { // #162
    fmav_fence_status_t* payload = (fmav_fence_status_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->breach_status, "breach_status", 0);
    lua_checktablenumber(L, payload->breach_count, "breach_count", 0);
    lua_checktablenumber(L, payload->breach_type, "breach_type", 0);
    lua_checktablenumber(L, payload->breach_time, "breach_time", 0);
    lua_checktablenumber(L, payload->breach_mitigation, "breach_mitigation", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_FENCE_STATUS_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_FENCE_STATUS_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_AHRS: { // #163
    fmav_ahrs_t* payload = (fmav_ahrs_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->omegaIx, "omegaIx", 0);
    lua_checktablenumber(L, payload->omegaIy, "omegaIy", 0);
    lua_checktablenumber(L, payload->omegaIz, "omegaIz", 0);
    lua_checktablenumber(L, payload->accel_weight, "accel_weight", 0);
    lua_checktablenumber(L, payload->renorm_val, "renorm_val", 0);
    lua_checktablenumber(L, payload->error_rp, "error_rp", 0);
    lua_checktablenumber(L, payload->error_yaw, "error_yaw", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_AHRS_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_AHRS_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_SIMSTATE: { // #164
    fmav_simstate_t* payload = (fmav_simstate_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->roll, "roll", 0);
    lua_checktablenumber(L, payload->pitch, "pitch", 0);
    lua_checktablenumber(L, payload->yaw, "yaw", 0);
    lua_checktablenumber(L, payload->xacc, "xacc", 0);
    lua_checktablenumber(L, payload->yacc, "yacc", 0);
    lua_checktablenumber(L, payload->zacc, "zacc", 0);
    lua_checktablenumber(L, payload->xgyro, "xgyro", 0);
    lua_checktablenumber(L, payload->ygyro, "ygyro", 0);
    lua_checktablenumber(L, payload->zgyro, "zgyro", 0);
    lua_checktablenumber(L, payload->lat, "lat", 0);
    lua_checktablenumber(L, payload->lng, "lng", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_SIMSTATE_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_SIMSTATE_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_HWSTATUS: { // #165
    fmav_hwstatus_t* payload = (fmav_hwstatus_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->Vcc, "Vcc", 0);
    lua_checktablenumber(L, payload->I2Cerr, "I2Cerr", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_HWSTATUS_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_HWSTATUS_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_RADIO: { // #166
    fmav_radio_t* payload = (fmav_radio_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->rssi, "rssi", 0);
    lua_checktablenumber(L, payload->remrssi, "remrssi", 0);
    lua_checktablenumber(L, payload->txbuf, "txbuf", 0);
    lua_checktablenumber(L, payload->noise, "noise", 0);
    lua_checktablenumber(L, payload->remnoise, "remnoise", 0);
    lua_checktablenumber(L, payload->rxerrors, "rxerrors", 0);
    lua_checktablenumber(L, payload->fixed, "fixed", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_RADIO_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_RADIO_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_LIMITS_STATUS: { // #167
    fmav_limits_status_t* payload = (fmav_limits_status_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->limits_state, "limits_state", 0);
    lua_checktablenumber(L, payload->last_trigger, "last_trigger", 0);
    lua_checktablenumber(L, payload->last_action, "last_action", 0);
    lua_checktablenumber(L, payload->last_recovery, "last_recovery", 0);
    lua_checktablenumber(L, payload->last_clear, "last_clear", 0);
    lua_checktablenumber(L, payload->breach_count, "breach_count", 0);
    lua_checktablenumber(L, payload->mods_enabled, "mods_enabled", 0);
    lua_checktablenumber(L, payload->mods_required, "mods_required", 0);
    lua_checktablenumber(L, payload->mods_triggered, "mods_triggered", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_LIMITS_STATUS_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_LIMITS_STATUS_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_WIND: { // #168
    fmav_wind_t* payload = (fmav_wind_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->direction, "direction", 0);
    lua_checktablenumber(L, payload->speed, "speed", 0);
    lua_checktablenumber(L, payload->speed_z, "speed_z", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_WIND_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_WIND_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_DATA16: { // #169
    fmav_data16_t* payload = (fmav_data16_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->type, "type", 0);
    lua_checktablenumber(L, payload->len, "len", 0);
    lua_pushstring(L, "data"); // array data[16]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 16; i++) { 
        lua_checktableinumber(L, payload->data[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    msg_out->crc_extra = FASTMAVLINK_MSG_DATA16_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_DATA16_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_DATA32: { // #170
    fmav_data32_t* payload = (fmav_data32_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->type, "type", 0);
    lua_checktablenumber(L, payload->len, "len", 0);
    lua_pushstring(L, "data"); // array data[32]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 32; i++) { 
        lua_checktableinumber(L, payload->data[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    msg_out->crc_extra = FASTMAVLINK_MSG_DATA32_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_DATA32_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_DATA64: { // #171
    fmav_data64_t* payload = (fmav_data64_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->type, "type", 0);
    lua_checktablenumber(L, payload->len, "len", 0);
    lua_pushstring(L, "data"); // array data[64]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 64; i++) { 
        lua_checktableinumber(L, payload->data[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    msg_out->crc_extra = FASTMAVLINK_MSG_DATA64_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_DATA64_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_DATA96: { // #172
    fmav_data96_t* payload = (fmav_data96_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->type, "type", 0);
    lua_checktablenumber(L, payload->len, "len", 0);
    lua_pushstring(L, "data"); // array data[96]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 96; i++) { 
        lua_checktableinumber(L, payload->data[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    msg_out->crc_extra = FASTMAVLINK_MSG_DATA96_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_DATA96_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_RANGEFINDER: { // #173
    fmav_rangefinder_t* payload = (fmav_rangefinder_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->distance, "distance", 0);
    lua_checktablenumber(L, payload->voltage, "voltage", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_RANGEFINDER_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_RANGEFINDER_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_AIRSPEED_AUTOCAL: { // #174
    fmav_airspeed_autocal_t* payload = (fmav_airspeed_autocal_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->vx, "vx", 0);
    lua_checktablenumber(L, payload->vy, "vy", 0);
    lua_checktablenumber(L, payload->vz, "vz", 0);
    lua_checktablenumber(L, payload->diff_pressure, "diff_pressure", 0);
    lua_checktablenumber(L, payload->EAS2TAS, "EAS2TAS", 0);
    lua_checktablenumber(L, payload->ratio, "ratio", 0);
    lua_checktablenumber(L, payload->state_x, "state_x", 0);
    lua_checktablenumber(L, payload->state_y, "state_y", 0);
    lua_checktablenumber(L, payload->state_z, "state_z", 0);
    lua_checktablenumber(L, payload->Pax, "Pax", 0);
    lua_checktablenumber(L, payload->Pby, "Pby", 0);
    lua_checktablenumber(L, payload->Pcz, "Pcz", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_AIRSPEED_AUTOCAL_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_AIRSPEED_AUTOCAL_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_RALLY_POINT: { // #175
    fmav_rally_point_t* payload = (fmav_rally_point_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->target_system, "target_sysid", 0);
    lua_checktablenumber(L, payload->target_component, "target_compid", 0);
    lua_checktablenumber(L, payload->idx, "idx", 0);
    lua_checktablenumber(L, payload->count, "count", 0);
    lua_checktablenumber(L, payload->lat, "lat", 0);
    lua_checktablenumber(L, payload->lng, "lng", 0);
    lua_checktablenumber(L, payload->alt, "alt", 0);
    lua_checktablenumber(L, payload->break_alt, "break_alt", 0);
    lua_checktablenumber(L, payload->land_dir, "land_dir", 0);
    lua_checktablenumber(L, payload->flags, "flags", 0);
    msg_out->target_sysid = payload->target_system;
    msg_out->target_compid = payload->target_component;
    msg_out->crc_extra = FASTMAVLINK_MSG_RALLY_POINT_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_RALLY_POINT_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_RALLY_FETCH_POINT: { // #176
    fmav_rally_fetch_point_t* payload = (fmav_rally_fetch_point_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->target_system, "target_sysid", 0);
    lua_checktablenumber(L, payload->target_component, "target_compid", 0);
    lua_checktablenumber(L, payload->idx, "idx", 0);
    msg_out->target_sysid = payload->target_system;
    msg_out->target_compid = payload->target_component;
    msg_out->crc_extra = FASTMAVLINK_MSG_RALLY_FETCH_POINT_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_RALLY_FETCH_POINT_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_COMPASSMOT_STATUS: { // #177
    fmav_compassmot_status_t* payload = (fmav_compassmot_status_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->throttle, "throttle", 0);
    lua_checktablenumber(L, payload->current, "current", 0);
    lua_checktablenumber(L, payload->interference, "interference", 0);
    lua_checktablenumber(L, payload->CompensationX, "CompensationX", 0);
    lua_checktablenumber(L, payload->CompensationY, "CompensationY", 0);
    lua_checktablenumber(L, payload->CompensationZ, "CompensationZ", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_COMPASSMOT_STATUS_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_COMPASSMOT_STATUS_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_AHRS2: { // #178
    fmav_ahrs2_t* payload = (fmav_ahrs2_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->roll, "roll", 0);
    lua_checktablenumber(L, payload->pitch, "pitch", 0);
    lua_checktablenumber(L, payload->yaw, "yaw", 0);
    lua_checktablenumber(L, payload->altitude, "altitude", 0);
    lua_checktablenumber(L, payload->lat, "lat", 0);
    lua_checktablenumber(L, payload->lng, "lng", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_AHRS2_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_AHRS2_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_CAMERA_STATUS: { // #179
    fmav_camera_status_t* payload = (fmav_camera_status_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->time_usec, "time_usec", 0);
    lua_checktablenumber(L, payload->target_system, "target_sysid", 0);
    lua_checktablenumber(L, payload->cam_idx, "cam_idx", 0);
    lua_checktablenumber(L, payload->img_idx, "img_idx", 0);
    lua_checktablenumber(L, payload->event_id, "event_id", 0);
    lua_checktablenumber(L, payload->p1, "p1", 0);
    lua_checktablenumber(L, payload->p2, "p2", 0);
    lua_checktablenumber(L, payload->p3, "p3", 0);
    lua_checktablenumber(L, payload->p4, "p4", 0);
    msg_out->target_sysid = payload->target_system;
    msg_out->crc_extra = FASTMAVLINK_MSG_CAMERA_STATUS_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_CAMERA_STATUS_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_CAMERA_FEEDBACK: { // #180
    fmav_camera_feedback_t* payload = (fmav_camera_feedback_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->time_usec, "time_usec", 0);
    lua_checktablenumber(L, payload->target_system, "target_sysid", 0);
    lua_checktablenumber(L, payload->cam_idx, "cam_idx", 0);
    lua_checktablenumber(L, payload->img_idx, "img_idx", 0);
    lua_checktablenumber(L, payload->lat, "lat", 0);
    lua_checktablenumber(L, payload->lng, "lng", 0);
    lua_checktablenumber(L, payload->alt_msl, "alt_msl", 0);
    lua_checktablenumber(L, payload->alt_rel, "alt_rel", 0);
    lua_checktablenumber(L, payload->roll, "roll", 0);
    lua_checktablenumber(L, payload->pitch, "pitch", 0);
    lua_checktablenumber(L, payload->yaw, "yaw", 0);
    lua_checktablenumber(L, payload->foc_len, "foc_len", 0);
    lua_checktablenumber(L, payload->flags, "flags", 0);
    lua_checktablenumber(L, payload->completed_captures, "completed_captures", 0);
    msg_out->target_sysid = payload->target_system;
    msg_out->crc_extra = FASTMAVLINK_MSG_CAMERA_FEEDBACK_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_CAMERA_FEEDBACK_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_BATTERY2: { // #181
    fmav_battery2_t* payload = (fmav_battery2_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->voltage, "voltage", 0);
    lua_checktablenumber(L, payload->current_battery, "current_battery", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_BATTERY2_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_BATTERY2_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_AHRS3: { // #182
    fmav_ahrs3_t* payload = (fmav_ahrs3_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->roll, "roll", 0);
    lua_checktablenumber(L, payload->pitch, "pitch", 0);
    lua_checktablenumber(L, payload->yaw, "yaw", 0);
    lua_checktablenumber(L, payload->altitude, "altitude", 0);
    lua_checktablenumber(L, payload->lat, "lat", 0);
    lua_checktablenumber(L, payload->lng, "lng", 0);
    lua_checktablenumber(L, payload->v1, "v1", 0);
    lua_checktablenumber(L, payload->v2, "v2", 0);
    lua_checktablenumber(L, payload->v3, "v3", 0);
    lua_checktablenumber(L, payload->v4, "v4", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_AHRS3_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_AHRS3_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_AUTOPILOT_VERSION_REQUEST: { // #183
    fmav_autopilot_version_request_t* payload = (fmav_autopilot_version_request_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->target_system, "target_sysid", 0);
    lua_checktablenumber(L, payload->target_component, "target_compid", 0);
    msg_out->target_sysid = payload->target_system;
    msg_out->target_compid = payload->target_component;
    msg_out->crc_extra = FASTMAVLINK_MSG_AUTOPILOT_VERSION_REQUEST_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_AUTOPILOT_VERSION_REQUEST_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_REMOTE_LOG_DATA_BLOCK: { // #184
    fmav_remote_log_data_block_t* payload = (fmav_remote_log_data_block_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->target_system, "target_sysid", 0);
    lua_checktablenumber(L, payload->target_component, "target_compid", 0);
    lua_checktablenumber(L, payload->seqno, "seqno", 0);
    lua_pushstring(L, "data"); // array data[200]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 200; i++) { 
        lua_checktableinumber(L, payload->data[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    msg_out->target_sysid = payload->target_system;
    msg_out->target_compid = payload->target_component;
    msg_out->crc_extra = FASTMAVLINK_MSG_REMOTE_LOG_DATA_BLOCK_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_REMOTE_LOG_DATA_BLOCK_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_REMOTE_LOG_BLOCK_STATUS: { // #185
    fmav_remote_log_block_status_t* payload = (fmav_remote_log_block_status_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->target_system, "target_sysid", 0);
    lua_checktablenumber(L, payload->target_component, "target_compid", 0);
    lua_checktablenumber(L, payload->seqno, "seqno", 0);
    lua_checktablenumber(L, payload->status, "status", 0);
    msg_out->target_sysid = payload->target_system;
    msg_out->target_compid = payload->target_component;
    msg_out->crc_extra = FASTMAVLINK_MSG_REMOTE_LOG_BLOCK_STATUS_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_REMOTE_LOG_BLOCK_STATUS_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_LED_CONTROL: { // #186
    fmav_led_control_t* payload = (fmav_led_control_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->target_system, "target_sysid", 0);
    lua_checktablenumber(L, payload->target_component, "target_compid", 0);
    lua_checktablenumber(L, payload->instance, "instance", 0);
    lua_checktablenumber(L, payload->pattern, "pattern", 0);
    lua_checktablenumber(L, payload->custom_len, "custom_len", 0);
    lua_pushstring(L, "custom_bytes"); // array custom_bytes[24]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 24; i++) { 
        lua_checktableinumber(L, payload->custom_bytes[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    msg_out->target_sysid = payload->target_system;
    msg_out->target_compid = payload->target_component;
    msg_out->crc_extra = FASTMAVLINK_MSG_LED_CONTROL_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_LED_CONTROL_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_MAG_CAL_PROGRESS: { // #191
    fmav_mag_cal_progress_t* payload = (fmav_mag_cal_progress_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->compass_id, "compass_id", 0);
    lua_checktablenumber(L, payload->cal_mask, "cal_mask", 0);
    lua_checktablenumber(L, payload->cal_status, "cal_status", 0);
    lua_checktablenumber(L, payload->attempt, "attempt", 0);
    lua_checktablenumber(L, payload->completion_pct, "completion_pct", 0);
    lua_pushstring(L, "completion_mask"); // array completion_mask[10]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 10; i++) { 
        lua_checktableinumber(L, payload->completion_mask[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_checktablenumber(L, payload->direction_x, "direction_x", 0);
    lua_checktablenumber(L, payload->direction_y, "direction_y", 0);
    lua_checktablenumber(L, payload->direction_z, "direction_z", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_MAG_CAL_PROGRESS_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_MAG_CAL_PROGRESS_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_MAG_CAL_REPORT: { // #192
    fmav_mag_cal_report_t* payload = (fmav_mag_cal_report_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->compass_id, "compass_id", 0);
    lua_checktablenumber(L, payload->cal_mask, "cal_mask", 0);
    lua_checktablenumber(L, payload->cal_status, "cal_status", 0);
    lua_checktablenumber(L, payload->autosaved, "autosaved", 0);
    lua_checktablenumber(L, payload->fitness, "fitness", 0);
    lua_checktablenumber(L, payload->ofs_x, "ofs_x", 0);
    lua_checktablenumber(L, payload->ofs_y, "ofs_y", 0);
    lua_checktablenumber(L, payload->ofs_z, "ofs_z", 0);
    lua_checktablenumber(L, payload->diag_x, "diag_x", 0);
    lua_checktablenumber(L, payload->diag_y, "diag_y", 0);
    lua_checktablenumber(L, payload->diag_z, "diag_z", 0);
    lua_checktablenumber(L, payload->offdiag_x, "offdiag_x", 0);
    lua_checktablenumber(L, payload->offdiag_y, "offdiag_y", 0);
    lua_checktablenumber(L, payload->offdiag_z, "offdiag_z", 0);
    lua_checktablenumber(L, payload->orientation_confidence, "orientation_confidence", 0);
    lua_checktablenumber(L, payload->old_orientation, "old_orientation", 0);
    lua_checktablenumber(L, payload->new_orientation, "new_orientation", 0);
    lua_checktablenumber(L, payload->scale_factor, "scale_factor", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_MAG_CAL_REPORT_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_MAG_CAL_REPORT_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_EKF_STATUS_REPORT: { // #193
    fmav_ekf_status_report_t* payload = (fmav_ekf_status_report_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->flags, "flags", 0);
    lua_checktablenumber(L, payload->velocity_variance, "velocity_variance", 0);
    lua_checktablenumber(L, payload->pos_horiz_variance, "pos_horiz_variance", 0);
    lua_checktablenumber(L, payload->pos_vert_variance, "pos_vert_variance", 0);
    lua_checktablenumber(L, payload->compass_variance, "compass_variance", 0);
    lua_checktablenumber(L, payload->terrain_alt_variance, "terrain_alt_variance", 0);
    lua_checktablenumber(L, payload->airspeed_variance, "airspeed_variance", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_EKF_STATUS_REPORT_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_EKF_STATUS_REPORT_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_PID_TUNING: { // #194
    fmav_pid_tuning_t* payload = (fmav_pid_tuning_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->axis, "axis", 0);
    lua_checktablenumber(L, payload->desired, "desired", 0);
    lua_checktablenumber(L, payload->achieved, "achieved", 0);
    lua_checktablenumber(L, payload->FF, "FF", 0);
    lua_checktablenumber(L, payload->P, "P", 0);
    lua_checktablenumber(L, payload->I, "I", 0);
    lua_checktablenumber(L, payload->D, "D", 0);
    lua_checktablenumber(L, payload->SRate, "SRate", 0);
    lua_checktablenumber(L, payload->PDmod, "PDmod", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_PID_TUNING_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_PID_TUNING_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_DEEPSTALL: { // #195
    fmav_deepstall_t* payload = (fmav_deepstall_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->landing_lat, "landing_lat", 0);
    lua_checktablenumber(L, payload->landing_lon, "landing_lon", 0);
    lua_checktablenumber(L, payload->path_lat, "path_lat", 0);
    lua_checktablenumber(L, payload->path_lon, "path_lon", 0);
    lua_checktablenumber(L, payload->arc_entry_lat, "arc_entry_lat", 0);
    lua_checktablenumber(L, payload->arc_entry_lon, "arc_entry_lon", 0);
    lua_checktablenumber(L, payload->altitude, "altitude", 0);
    lua_checktablenumber(L, payload->expected_travel_distance, "expected_travel_distance", 0);
    lua_checktablenumber(L, payload->cross_track_error, "cross_track_error", 0);
    lua_checktablenumber(L, payload->stage, "stage", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_DEEPSTALL_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_DEEPSTALL_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_GIMBAL_REPORT: { // #200
    fmav_gimbal_report_t* payload = (fmav_gimbal_report_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->target_system, "target_sysid", 0);
    lua_checktablenumber(L, payload->target_component, "target_compid", 0);
    lua_checktablenumber(L, payload->delta_time, "delta_time", 0);
    lua_checktablenumber(L, payload->delta_angle_x, "delta_angle_x", 0);
    lua_checktablenumber(L, payload->delta_angle_y, "delta_angle_y", 0);
    lua_checktablenumber(L, payload->delta_angle_z, "delta_angle_z", 0);
    lua_checktablenumber(L, payload->delta_velocity_x, "delta_velocity_x", 0);
    lua_checktablenumber(L, payload->delta_velocity_y, "delta_velocity_y", 0);
    lua_checktablenumber(L, payload->delta_velocity_z, "delta_velocity_z", 0);
    lua_checktablenumber(L, payload->joint_roll, "joint_roll", 0);
    lua_checktablenumber(L, payload->joint_el, "joint_el", 0);
    lua_checktablenumber(L, payload->joint_az, "joint_az", 0);
    msg_out->target_sysid = payload->target_system;
    msg_out->target_compid = payload->target_component;
    msg_out->crc_extra = FASTMAVLINK_MSG_GIMBAL_REPORT_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_GIMBAL_REPORT_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_GIMBAL_CONTROL: { // #201
    fmav_gimbal_control_t* payload = (fmav_gimbal_control_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->target_system, "target_sysid", 0);
    lua_checktablenumber(L, payload->target_component, "target_compid", 0);
    lua_checktablenumber(L, payload->demanded_rate_x, "demanded_rate_x", 0);
    lua_checktablenumber(L, payload->demanded_rate_y, "demanded_rate_y", 0);
    lua_checktablenumber(L, payload->demanded_rate_z, "demanded_rate_z", 0);
    msg_out->target_sysid = payload->target_system;
    msg_out->target_compid = payload->target_component;
    msg_out->crc_extra = FASTMAVLINK_MSG_GIMBAL_CONTROL_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_GIMBAL_CONTROL_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_GIMBAL_TORQUE_CMD_REPORT: { // #214
    fmav_gimbal_torque_cmd_report_t* payload = (fmav_gimbal_torque_cmd_report_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->target_system, "target_sysid", 0);
    lua_checktablenumber(L, payload->target_component, "target_compid", 0);
    lua_checktablenumber(L, payload->rl_torque_cmd, "rl_torque_cmd", 0);
    lua_checktablenumber(L, payload->el_torque_cmd, "el_torque_cmd", 0);
    lua_checktablenumber(L, payload->az_torque_cmd, "az_torque_cmd", 0);
    msg_out->target_sysid = payload->target_system;
    msg_out->target_compid = payload->target_component;
    msg_out->crc_extra = FASTMAVLINK_MSG_GIMBAL_TORQUE_CMD_REPORT_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_GIMBAL_TORQUE_CMD_REPORT_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_GOPRO_HEARTBEAT: { // #215
    fmav_gopro_heartbeat_t* payload = (fmav_gopro_heartbeat_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->status, "status", 0);
    lua_checktablenumber(L, payload->capture_mode, "capture_mode", 0);
    lua_checktablenumber(L, payload->flags, "flags", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_GOPRO_HEARTBEAT_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_GOPRO_HEARTBEAT_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_GOPRO_GET_REQUEST: { // #216
    fmav_gopro_get_request_t* payload = (fmav_gopro_get_request_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->target_system, "target_sysid", 0);
    lua_checktablenumber(L, payload->target_component, "target_compid", 0);
    lua_checktablenumber(L, payload->cmd_id, "cmd_id", 0);
    msg_out->target_sysid = payload->target_system;
    msg_out->target_compid = payload->target_component;
    msg_out->crc_extra = FASTMAVLINK_MSG_GOPRO_GET_REQUEST_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_GOPRO_GET_REQUEST_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_GOPRO_GET_RESPONSE: { // #217
    fmav_gopro_get_response_t* payload = (fmav_gopro_get_response_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->cmd_id, "cmd_id", 0);
    lua_checktablenumber(L, payload->status, "status", 0);
    lua_pushstring(L, "value"); // array value[4]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 4; i++) { 
        lua_checktableinumber(L, payload->value[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    msg_out->crc_extra = FASTMAVLINK_MSG_GOPRO_GET_RESPONSE_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_GOPRO_GET_RESPONSE_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_GOPRO_SET_REQUEST: { // #218
    fmav_gopro_set_request_t* payload = (fmav_gopro_set_request_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->target_system, "target_sysid", 0);
    lua_checktablenumber(L, payload->target_component, "target_compid", 0);
    lua_checktablenumber(L, payload->cmd_id, "cmd_id", 0);
    lua_pushstring(L, "value"); // array value[4]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 4; i++) { 
        lua_checktableinumber(L, payload->value[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    msg_out->target_sysid = payload->target_system;
    msg_out->target_compid = payload->target_component;
    msg_out->crc_extra = FASTMAVLINK_MSG_GOPRO_SET_REQUEST_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_GOPRO_SET_REQUEST_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_GOPRO_SET_RESPONSE: { // #219
    fmav_gopro_set_response_t* payload = (fmav_gopro_set_response_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->cmd_id, "cmd_id", 0);
    lua_checktablenumber(L, payload->status, "status", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_GOPRO_SET_RESPONSE_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_GOPRO_SET_RESPONSE_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_EFI_STATUS: { // #225
    fmav_efi_status_t* payload = (fmav_efi_status_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->health, "health", 0);
    lua_checktablenumber(L, payload->ecu_index, "ecu_index", 0);
    lua_checktablenumber(L, payload->rpm, "rpm", 0);
    lua_checktablenumber(L, payload->fuel_consumed, "fuel_consumed", 0);
    lua_checktablenumber(L, payload->fuel_flow, "fuel_flow", 0);
    lua_checktablenumber(L, payload->engine_load, "engine_load", 0);
    lua_checktablenumber(L, payload->throttle_position, "throttle_position", 0);
    lua_checktablenumber(L, payload->spark_dwell_time, "spark_dwell_time", 0);
    lua_checktablenumber(L, payload->barometric_pressure, "barometric_pressure", 0);
    lua_checktablenumber(L, payload->intake_manifold_pressure, "intake_manifold_pressure", 0);
    lua_checktablenumber(L, payload->intake_manifold_temperature, "intake_manifold_temperature", 0);
    lua_checktablenumber(L, payload->cylinder_head_temperature, "cylinder_head_temperature", 0);
    lua_checktablenumber(L, payload->ignition_timing, "ignition_timing", 0);
    lua_checktablenumber(L, payload->injection_time, "injection_time", 0);
    lua_checktablenumber(L, payload->exhaust_gas_temperature, "exhaust_gas_temperature", 0);
    lua_checktablenumber(L, payload->throttle_out, "throttle_out", 0);
    lua_checktablenumber(L, payload->pt_compensation, "pt_compensation", 0);
    lua_checktablenumber(L, payload->ignition_voltage, "ignition_voltage", 0);
    lua_checktablenumber(L, payload->fuel_pressure, "fuel_pressure", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_EFI_STATUS_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_EFI_STATUS_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_RPM: { // #226
    fmav_rpm_t* payload = (fmav_rpm_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->rpm1, "rpm1", 0);
    lua_checktablenumber(L, payload->rpm2, "rpm2", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_RPM_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_RPM_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_ESTIMATOR_STATUS: { // #230
    fmav_estimator_status_t* payload = (fmav_estimator_status_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->time_usec, "time_usec", 0);
    lua_checktablenumber(L, payload->flags, "flags", 0);
    lua_checktablenumber(L, payload->vel_ratio, "vel_ratio", 0);
    lua_checktablenumber(L, payload->pos_horiz_ratio, "pos_horiz_ratio", 0);
    lua_checktablenumber(L, payload->pos_vert_ratio, "pos_vert_ratio", 0);
    lua_checktablenumber(L, payload->mag_ratio, "mag_ratio", 0);
    lua_checktablenumber(L, payload->hagl_ratio, "hagl_ratio", 0);
    lua_checktablenumber(L, payload->tas_ratio, "tas_ratio", 0);
    lua_checktablenumber(L, payload->pos_horiz_accuracy, "pos_horiz_accuracy", 0);
    lua_checktablenumber(L, payload->pos_vert_accuracy, "pos_vert_accuracy", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_ESTIMATOR_STATUS_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_ESTIMATOR_STATUS_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_WIND_COV: { // #231
    fmav_wind_cov_t* payload = (fmav_wind_cov_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->time_usec, "time_usec", 0);
    lua_checktablenumber(L, payload->wind_x, "wind_x", NAN);
    lua_checktablenumber(L, payload->wind_y, "wind_y", NAN);
    lua_checktablenumber(L, payload->wind_z, "wind_z", NAN);
    lua_checktablenumber(L, payload->var_horiz, "var_horiz", NAN);
    lua_checktablenumber(L, payload->var_vert, "var_vert", NAN);
    lua_checktablenumber(L, payload->wind_alt, "wind_alt", NAN);
    lua_checktablenumber(L, payload->horiz_accuracy, "horiz_accuracy", 0);
    lua_checktablenumber(L, payload->vert_accuracy, "vert_accuracy", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_WIND_COV_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_WIND_COV_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_GPS_INPUT: { // #232
    fmav_gps_input_t* payload = (fmav_gps_input_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->time_usec, "time_usec", 0);
    lua_checktablenumber(L, payload->gps_id, "gps_id", 0);
    lua_checktablenumber(L, payload->ignore_flags, "ignore_flags", 0);
    lua_checktablenumber(L, payload->time_week_ms, "time_week_ms", 0);
    lua_checktablenumber(L, payload->time_week, "time_week", 0);
    lua_checktablenumber(L, payload->fix_type, "fix_type", 0);
    lua_checktablenumber(L, payload->lat, "lat", 0);
    lua_checktablenumber(L, payload->lon, "lon", 0);
    lua_checktablenumber(L, payload->alt, "alt", 0);
    lua_checktablenumber(L, payload->hdop, "hdop", UINT16_MAX);
    lua_checktablenumber(L, payload->vdop, "vdop", UINT16_MAX);
    lua_checktablenumber(L, payload->vn, "vn", 0);
    lua_checktablenumber(L, payload->ve, "ve", 0);
    lua_checktablenumber(L, payload->vd, "vd", 0);
    lua_checktablenumber(L, payload->speed_accuracy, "speed_accuracy", 0);
    lua_checktablenumber(L, payload->horiz_accuracy, "horiz_accuracy", 0);
    lua_checktablenumber(L, payload->vert_accuracy, "vert_accuracy", 0);
    lua_checktablenumber(L, payload->satellites_visible, "satellites_visible", 0);
    lua_checktablenumber(L, payload->yaw, "yaw", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_GPS_INPUT_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_GPS_INPUT_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_GPS_RTCM_DATA: { // #233
    fmav_gps_rtcm_data_t* payload = (fmav_gps_rtcm_data_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->flags, "flags", 0);
    lua_checktablenumber(L, payload->len, "len", 0);
    lua_pushstring(L, "data"); // array data[180]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 180; i++) { 
        lua_checktableinumber(L, payload->data[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    msg_out->crc_extra = FASTMAVLINK_MSG_GPS_RTCM_DATA_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_GPS_RTCM_DATA_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_HIGH_LATENCY: { // #234
    fmav_high_latency_t* payload = (fmav_high_latency_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->base_mode, "base_mode", 0);
    lua_checktablenumber(L, payload->custom_mode, "custom_mode", 0);
    lua_checktablenumber(L, payload->landed_state, "landed_state", 0);
    lua_checktablenumber(L, payload->roll, "roll", 0);
    lua_checktablenumber(L, payload->pitch, "pitch", 0);
    lua_checktablenumber(L, payload->heading, "heading", 0);
    lua_checktablenumber(L, payload->throttle, "throttle", 0);
    lua_checktablenumber(L, payload->heading_sp, "heading_sp", 0);
    lua_checktablenumber(L, payload->latitude, "latitude", 0);
    lua_checktablenumber(L, payload->longitude, "longitude", 0);
    lua_checktablenumber(L, payload->altitude_amsl, "altitude_amsl", 0);
    lua_checktablenumber(L, payload->altitude_sp, "altitude_sp", 0);
    lua_checktablenumber(L, payload->airspeed, "airspeed", 0);
    lua_checktablenumber(L, payload->airspeed_sp, "airspeed_sp", 0);
    lua_checktablenumber(L, payload->groundspeed, "groundspeed", 0);
    lua_checktablenumber(L, payload->climb_rate, "climb_rate", 0);
    lua_checktablenumber(L, payload->gps_nsat, "gps_nsat", UINT8_MAX);
    lua_checktablenumber(L, payload->gps_fix_type, "gps_fix_type", 0);
    lua_checktablenumber(L, payload->battery_remaining, "battery_remaining", 0);
    lua_checktablenumber(L, payload->temperature, "temperature", 0);
    lua_checktablenumber(L, payload->temperature_air, "temperature_air", 0);
    lua_checktablenumber(L, payload->failsafe, "failsafe", 0);
    lua_checktablenumber(L, payload->wp_num, "wp_num", 0);
    lua_checktablenumber(L, payload->wp_distance, "wp_distance", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_HIGH_LATENCY_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_HIGH_LATENCY_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_HIGH_LATENCY2: { // #235
    fmav_high_latency2_t* payload = (fmav_high_latency2_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->timestamp, "timestamp", 0);
    lua_checktablenumber(L, payload->type, "type", 0);
    lua_checktablenumber(L, payload->autopilot, "autopilot", 0);
    lua_checktablenumber(L, payload->custom_mode, "custom_mode", 0);
    lua_checktablenumber(L, payload->latitude, "latitude", 0);
    lua_checktablenumber(L, payload->longitude, "longitude", 0);
    lua_checktablenumber(L, payload->altitude, "altitude", 0);
    lua_checktablenumber(L, payload->target_altitude, "target_altitude", 0);
    lua_checktablenumber(L, payload->heading, "heading", 0);
    lua_checktablenumber(L, payload->target_heading, "target_heading", 0);
    lua_checktablenumber(L, payload->target_distance, "target_distance", 0);
    lua_checktablenumber(L, payload->throttle, "throttle", 0);
    lua_checktablenumber(L, payload->airspeed, "airspeed", 0);
    lua_checktablenumber(L, payload->airspeed_sp, "airspeed_sp", 0);
    lua_checktablenumber(L, payload->groundspeed, "groundspeed", 0);
    lua_checktablenumber(L, payload->windspeed, "windspeed", 0);
    lua_checktablenumber(L, payload->wind_heading, "wind_heading", 0);
    lua_checktablenumber(L, payload->eph, "eph", 0);
    lua_checktablenumber(L, payload->epv, "epv", 0);
    lua_checktablenumber(L, payload->temperature_air, "temperature_air", 0);
    lua_checktablenumber(L, payload->climb_rate, "climb_rate", 0);
    lua_checktablenumber(L, payload->battery, "battery", -1);
    lua_checktablenumber(L, payload->wp_num, "wp_num", 0);
    lua_checktablenumber(L, payload->failure_flags, "failure_flags", 0);
    lua_checktablenumber(L, payload->custom0, "custom0", 0);
    lua_checktablenumber(L, payload->custom1, "custom1", 0);
    lua_checktablenumber(L, payload->custom2, "custom2", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_HIGH_LATENCY2_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_HIGH_LATENCY2_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_VIBRATION: { // #241
    fmav_vibration_t* payload = (fmav_vibration_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->time_usec, "time_usec", 0);
    lua_checktablenumber(L, payload->vibration_x, "vibration_x", 0);
    lua_checktablenumber(L, payload->vibration_y, "vibration_y", 0);
    lua_checktablenumber(L, payload->vibration_z, "vibration_z", 0);
    lua_checktablenumber(L, payload->clipping_0, "clipping_0", 0);
    lua_checktablenumber(L, payload->clipping_1, "clipping_1", 0);
    lua_checktablenumber(L, payload->clipping_2, "clipping_2", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_VIBRATION_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_VIBRATION_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_HOME_POSITION: { // #242
    fmav_home_position_t* payload = (fmav_home_position_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->latitude, "latitude", 0);
    lua_checktablenumber(L, payload->longitude, "longitude", 0);
    lua_checktablenumber(L, payload->altitude, "altitude", 0);
    lua_checktablenumber(L, payload->x, "x", 0);
    lua_checktablenumber(L, payload->y, "y", 0);
    lua_checktablenumber(L, payload->z, "z", 0);
    lua_pushstring(L, "q"); // array q[4]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 4; i++) { 
        lua_checktableinumber(L, payload->q[i], i+1, NAN); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_checktablenumber(L, payload->approach_x, "approach_x", 0);
    lua_checktablenumber(L, payload->approach_y, "approach_y", 0);
    lua_checktablenumber(L, payload->approach_z, "approach_z", 0);
    lua_checktablenumber(L, payload->time_usec, "time_usec", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_HOME_POSITION_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_HOME_POSITION_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_SET_HOME_POSITION: { // #243
    fmav_set_home_position_t* payload = (fmav_set_home_position_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->target_system, "target_sysid", 0);
    lua_checktablenumber(L, payload->latitude, "latitude", 0);
    lua_checktablenumber(L, payload->longitude, "longitude", 0);
    lua_checktablenumber(L, payload->altitude, "altitude", 0);
    lua_checktablenumber(L, payload->x, "x", 0);
    lua_checktablenumber(L, payload->y, "y", 0);
    lua_checktablenumber(L, payload->z, "z", 0);
    lua_pushstring(L, "q"); // array q[4]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 4; i++) { 
        lua_checktableinumber(L, payload->q[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_checktablenumber(L, payload->approach_x, "approach_x", 0);
    lua_checktablenumber(L, payload->approach_y, "approach_y", 0);
    lua_checktablenumber(L, payload->approach_z, "approach_z", 0);
    lua_checktablenumber(L, payload->time_usec, "time_usec", 0);
    msg_out->target_sysid = payload->target_system;
    msg_out->crc_extra = FASTMAVLINK_MSG_SET_HOME_POSITION_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_SET_HOME_POSITION_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_MESSAGE_INTERVAL: { // #244
    fmav_message_interval_t* payload = (fmav_message_interval_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->message_id, "message_id", 0);
    lua_checktablenumber(L, payload->interval_us, "interval_us", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_MESSAGE_INTERVAL_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_MESSAGE_INTERVAL_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_EXTENDED_SYS_STATE: { // #245
    fmav_extended_sys_state_t* payload = (fmav_extended_sys_state_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->vtol_state, "vtol_state", 0);
    lua_checktablenumber(L, payload->landed_state, "landed_state", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_EXTENDED_SYS_STATE_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_EXTENDED_SYS_STATE_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_ADSB_VEHICLE: { // #246
    fmav_adsb_vehicle_t* payload = (fmav_adsb_vehicle_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->ICAO_address, "ICAO_address", 0);
    lua_checktablenumber(L, payload->lat, "lat", 0);
    lua_checktablenumber(L, payload->lon, "lon", 0);
    lua_checktablenumber(L, payload->altitude_type, "altitude_type", 0);
    lua_checktablenumber(L, payload->altitude, "altitude", 0);
    lua_checktablenumber(L, payload->heading, "heading", 0);
    lua_checktablenumber(L, payload->hor_velocity, "hor_velocity", 0);
    lua_checktablenumber(L, payload->ver_velocity, "ver_velocity", 0);
    lua_pushstring(L, "callsign"); // array callsign[9]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 9; i++) { 
        lua_checktableinumber(L, payload->callsign[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_checktablenumber(L, payload->emitter_type, "emitter_type", 0);
    lua_checktablenumber(L, payload->tslc, "tslc", 0);
    lua_checktablenumber(L, payload->flags, "flags", 0);
    lua_checktablenumber(L, payload->squawk, "squawk", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_ADSB_VEHICLE_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_ADSB_VEHICLE_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_COLLISION: { // #247
    fmav_collision_t* payload = (fmav_collision_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->src, "src", 0);
    lua_checktablenumber(L, payload->id, "id", 0);
    lua_checktablenumber(L, payload->action, "action", 0);
    lua_checktablenumber(L, payload->threat_level, "threat_level", 0);
    lua_checktablenumber(L, payload->time_to_minimum_delta, "time_to_minimum_delta", 0);
    lua_checktablenumber(L, payload->altitude_minimum_delta, "altitude_minimum_delta", 0);
    lua_checktablenumber(L, payload->horizontal_minimum_delta, "horizontal_minimum_delta", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_COLLISION_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_COLLISION_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_V2_EXTENSION: { // #248
    fmav_v2_extension_t* payload = (fmav_v2_extension_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->target_network, "target_network", 0);
    lua_checktablenumber(L, payload->target_system, "target_sysid", 0);
    lua_checktablenumber(L, payload->target_component, "target_compid", 0);
    lua_checktablenumber(L, payload->message_type, "message_type", 0);
    lua_pushstring(L, "payload"); // array payload[249]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 249; i++) { 
        lua_checktableinumber(L, payload->payload[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    msg_out->target_sysid = payload->target_system;
    msg_out->target_compid = payload->target_component;
    msg_out->crc_extra = FASTMAVLINK_MSG_V2_EXTENSION_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_V2_EXTENSION_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_MEMORY_VECT: { // #249
    fmav_memory_vect_t* payload = (fmav_memory_vect_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->address, "address", 0);
    lua_checktablenumber(L, payload->ver, "ver", 0);
    lua_checktablenumber(L, payload->type, "type", 0);
    lua_pushstring(L, "value"); // array value[32]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 32; i++) { 
        lua_checktableinumber(L, payload->value[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    msg_out->crc_extra = FASTMAVLINK_MSG_MEMORY_VECT_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_MEMORY_VECT_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_DEBUG_VECT: { // #250
    fmav_debug_vect_t* payload = (fmav_debug_vect_t*)(msg_out->payload);
    lua_pushstring(L, "name"); // array name[10]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 10; i++) { 
        lua_checktableinumber(L, payload->name[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_checktablenumber(L, payload->time_usec, "time_usec", 0);
    lua_checktablenumber(L, payload->x, "x", 0);
    lua_checktablenumber(L, payload->y, "y", 0);
    lua_checktablenumber(L, payload->z, "z", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_DEBUG_VECT_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_DEBUG_VECT_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_NAMED_VALUE_FLOAT: { // #251
    fmav_named_value_float_t* payload = (fmav_named_value_float_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->time_boot_ms, "time_boot_ms", 0);
    lua_pushstring(L, "name"); // array name[10]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 10; i++) { 
        lua_checktableinumber(L, payload->name[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_checktablenumber(L, payload->value, "value", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_NAMED_VALUE_FLOAT_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_NAMED_VALUE_FLOAT_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_NAMED_VALUE_INT: { // #252
    fmav_named_value_int_t* payload = (fmav_named_value_int_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->time_boot_ms, "time_boot_ms", 0);
    lua_pushstring(L, "name"); // array name[10]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 10; i++) { 
        lua_checktableinumber(L, payload->name[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_checktablenumber(L, payload->value, "value", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_NAMED_VALUE_INT_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_NAMED_VALUE_INT_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_STATUSTEXT: { // #253
    fmav_statustext_t* payload = (fmav_statustext_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->severity, "severity", 0);
    lua_pushstring(L, "text"); // array text[50]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 50; i++) { 
        lua_checktableinumber(L, payload->text[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_checktablenumber(L, payload->id, "id", 0);
    lua_checktablenumber(L, payload->chunk_seq, "chunk_seq", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_STATUSTEXT_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_STATUSTEXT_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_DEBUG: { // #254
    fmav_debug_t* payload = (fmav_debug_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->time_boot_ms, "time_boot_ms", 0);
    lua_checktablenumber(L, payload->ind, "ind", 0);
    lua_checktablenumber(L, payload->value, "value", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_DEBUG_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_DEBUG_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_SETUP_SIGNING: { // #256
    fmav_setup_signing_t* payload = (fmav_setup_signing_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->target_system, "target_sysid", 0);
    lua_checktablenumber(L, payload->target_component, "target_compid", 0);
    lua_pushstring(L, "secret_key"); // array secret_key[32]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 32; i++) { 
        lua_checktableinumber(L, payload->secret_key[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_checktablenumber(L, payload->initial_timestamp, "initial_timestamp", 0);
    msg_out->target_sysid = payload->target_system;
    msg_out->target_compid = payload->target_component;
    msg_out->crc_extra = FASTMAVLINK_MSG_SETUP_SIGNING_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_SETUP_SIGNING_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_BUTTON_CHANGE: { // #257
    fmav_button_change_t* payload = (fmav_button_change_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->time_boot_ms, "time_boot_ms", 0);
    lua_checktablenumber(L, payload->last_change_ms, "last_change_ms", 0);
    lua_checktablenumber(L, payload->state, "state", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_BUTTON_CHANGE_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_BUTTON_CHANGE_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_PLAY_TUNE: { // #258
    fmav_play_tune_t* payload = (fmav_play_tune_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->target_system, "target_sysid", 0);
    lua_checktablenumber(L, payload->target_component, "target_compid", 0);
    lua_pushstring(L, "tune"); // array tune[30]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 30; i++) { 
        lua_checktableinumber(L, payload->tune[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_pushstring(L, "tune2"); // array tune2[200]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 200; i++) { 
        lua_checktableinumber(L, payload->tune2[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    msg_out->target_sysid = payload->target_system;
    msg_out->target_compid = payload->target_component;
    msg_out->crc_extra = FASTMAVLINK_MSG_PLAY_TUNE_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_PLAY_TUNE_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_CAMERA_INFORMATION: { // #259
    fmav_camera_information_t* payload = (fmav_camera_information_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->time_boot_ms, "time_boot_ms", 0);
    lua_pushstring(L, "vendor_name"); // array vendor_name[32]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 32; i++) { 
        lua_checktableinumber(L, payload->vendor_name[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_pushstring(L, "model_name"); // array model_name[32]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 32; i++) { 
        lua_checktableinumber(L, payload->model_name[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_checktablenumber(L, payload->firmware_version, "firmware_version", 0);
    lua_checktablenumber(L, payload->focal_length, "focal_length", NAN);
    lua_checktablenumber(L, payload->sensor_size_h, "sensor_size_h", NAN);
    lua_checktablenumber(L, payload->sensor_size_v, "sensor_size_v", NAN);
    lua_checktablenumber(L, payload->resolution_h, "resolution_h", 0);
    lua_checktablenumber(L, payload->resolution_v, "resolution_v", 0);
    lua_checktablenumber(L, payload->lens_id, "lens_id", 0);
    lua_checktablenumber(L, payload->flags, "flags", 0);
    lua_checktablenumber(L, payload->cam_definition_version, "cam_definition_version", 0);
    lua_pushstring(L, "cam_definition_uri"); // array cam_definition_uri[140]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 140; i++) { 
        lua_checktableinumber(L, payload->cam_definition_uri[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_checktablenumber(L, payload->gimbal_device_id, "gimbal_device_id", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_CAMERA_INFORMATION_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_CAMERA_INFORMATION_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_CAMERA_SETTINGS: { // #260
    fmav_camera_settings_t* payload = (fmav_camera_settings_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->time_boot_ms, "time_boot_ms", 0);
    lua_checktablenumber(L, payload->mode_id, "mode_id", 0);
    lua_checktablenumber(L, payload->zoomLevel, "zoomLevel", NAN);
    lua_checktablenumber(L, payload->focusLevel, "focusLevel", NAN);
    msg_out->crc_extra = FASTMAVLINK_MSG_CAMERA_SETTINGS_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_CAMERA_SETTINGS_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_STORAGE_INFORMATION: { // #261
    fmav_storage_information_t* payload = (fmav_storage_information_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->time_boot_ms, "time_boot_ms", 0);
    lua_checktablenumber(L, payload->storage_id, "storage_id", 0);
    lua_checktablenumber(L, payload->storage_count, "storage_count", 0);
    lua_checktablenumber(L, payload->status, "status", 0);
    lua_checktablenumber(L, payload->total_capacity, "total_capacity", 0);
    lua_checktablenumber(L, payload->used_capacity, "used_capacity", 0);
    lua_checktablenumber(L, payload->available_capacity, "available_capacity", 0);
    lua_checktablenumber(L, payload->read_speed, "read_speed", 0);
    lua_checktablenumber(L, payload->write_speed, "write_speed", 0);
    lua_checktablenumber(L, payload->type, "type", 0);
    lua_pushstring(L, "name"); // array name[32]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 32; i++) { 
        lua_checktableinumber(L, payload->name[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_checktablenumber(L, payload->storage_usage, "storage_usage", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_STORAGE_INFORMATION_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_STORAGE_INFORMATION_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS: { // #262
    fmav_camera_capture_status_t* payload = (fmav_camera_capture_status_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->time_boot_ms, "time_boot_ms", 0);
    lua_checktablenumber(L, payload->image_status, "image_status", 0);
    lua_checktablenumber(L, payload->video_status, "video_status", 0);
    lua_checktablenumber(L, payload->image_interval, "image_interval", 0);
    lua_checktablenumber(L, payload->recording_time_ms, "recording_time_ms", 0);
    lua_checktablenumber(L, payload->available_capacity, "available_capacity", 0);
    lua_checktablenumber(L, payload->image_count, "image_count", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_CAMERA_CAPTURE_STATUS_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_CAMERA_CAPTURE_STATUS_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_CAMERA_IMAGE_CAPTURED: { // #263
    fmav_camera_image_captured_t* payload = (fmav_camera_image_captured_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->time_boot_ms, "time_boot_ms", 0);
    lua_checktablenumber(L, payload->time_utc, "time_utc", 0);
    lua_checktablenumber(L, payload->camera_id, "camera_id", 0);
    lua_checktablenumber(L, payload->lat, "lat", 0);
    lua_checktablenumber(L, payload->lon, "lon", 0);
    lua_checktablenumber(L, payload->alt, "alt", 0);
    lua_checktablenumber(L, payload->relative_alt, "relative_alt", 0);
    lua_pushstring(L, "q"); // array q[4]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 4; i++) { 
        lua_checktableinumber(L, payload->q[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_checktablenumber(L, payload->image_index, "image_index", 0);
    lua_checktablenumber(L, payload->capture_result, "capture_result", 0);
    lua_pushstring(L, "file_url"); // array file_url[205]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 205; i++) { 
        lua_checktableinumber(L, payload->file_url[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    msg_out->crc_extra = FASTMAVLINK_MSG_CAMERA_IMAGE_CAPTURED_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_CAMERA_IMAGE_CAPTURED_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_FLIGHT_INFORMATION: { // #264
    fmav_flight_information_t* payload = (fmav_flight_information_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->time_boot_ms, "time_boot_ms", 0);
    lua_checktablenumber(L, payload->arming_time_utc, "arming_time_utc", 0);
    lua_checktablenumber(L, payload->takeoff_time_utc, "takeoff_time_utc", 0);
    lua_checktablenumber(L, payload->flight_uuid, "flight_uuid", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_FLIGHT_INFORMATION_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_FLIGHT_INFORMATION_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_MOUNT_ORIENTATION: { // #265
    fmav_mount_orientation_t* payload = (fmav_mount_orientation_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->time_boot_ms, "time_boot_ms", 0);
    lua_checktablenumber(L, payload->roll, "roll", NAN);
    lua_checktablenumber(L, payload->pitch, "pitch", NAN);
    lua_checktablenumber(L, payload->yaw, "yaw", NAN);
    lua_checktablenumber(L, payload->yaw_absolute, "yaw_absolute", NAN);
    msg_out->crc_extra = FASTMAVLINK_MSG_MOUNT_ORIENTATION_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_MOUNT_ORIENTATION_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_LOGGING_DATA: { // #266
    fmav_logging_data_t* payload = (fmav_logging_data_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->target_system, "target_sysid", 0);
    lua_checktablenumber(L, payload->target_component, "target_compid", 0);
    lua_checktablenumber(L, payload->sequence, "sequence", 0);
    lua_checktablenumber(L, payload->length, "length", 0);
    lua_checktablenumber(L, payload->first_message_offset, "first_message_offset", UINT8_MAX);
    lua_pushstring(L, "data"); // array data[249]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 249; i++) { 
        lua_checktableinumber(L, payload->data[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    msg_out->target_sysid = payload->target_system;
    msg_out->target_compid = payload->target_component;
    msg_out->crc_extra = FASTMAVLINK_MSG_LOGGING_DATA_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_LOGGING_DATA_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_LOGGING_DATA_ACKED: { // #267
    fmav_logging_data_acked_t* payload = (fmav_logging_data_acked_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->target_system, "target_sysid", 0);
    lua_checktablenumber(L, payload->target_component, "target_compid", 0);
    lua_checktablenumber(L, payload->sequence, "sequence", 0);
    lua_checktablenumber(L, payload->length, "length", 0);
    lua_checktablenumber(L, payload->first_message_offset, "first_message_offset", UINT8_MAX);
    lua_pushstring(L, "data"); // array data[249]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 249; i++) { 
        lua_checktableinumber(L, payload->data[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    msg_out->target_sysid = payload->target_system;
    msg_out->target_compid = payload->target_component;
    msg_out->crc_extra = FASTMAVLINK_MSG_LOGGING_DATA_ACKED_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_LOGGING_DATA_ACKED_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_LOGGING_ACK: { // #268
    fmav_logging_ack_t* payload = (fmav_logging_ack_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->target_system, "target_sysid", 0);
    lua_checktablenumber(L, payload->target_component, "target_compid", 0);
    lua_checktablenumber(L, payload->sequence, "sequence", 0);
    msg_out->target_sysid = payload->target_system;
    msg_out->target_compid = payload->target_component;
    msg_out->crc_extra = FASTMAVLINK_MSG_LOGGING_ACK_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_LOGGING_ACK_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_VIDEO_STREAM_INFORMATION: { // #269
    fmav_video_stream_information_t* payload = (fmav_video_stream_information_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->stream_id, "stream_id", 0);
    lua_checktablenumber(L, payload->count, "count", 0);
    lua_checktablenumber(L, payload->type, "type", 0);
    lua_checktablenumber(L, payload->flags, "flags", 0);
    lua_checktablenumber(L, payload->framerate, "framerate", 0);
    lua_checktablenumber(L, payload->resolution_h, "resolution_h", 0);
    lua_checktablenumber(L, payload->resolution_v, "resolution_v", 0);
    lua_checktablenumber(L, payload->bitrate, "bitrate", 0);
    lua_checktablenumber(L, payload->rotation, "rotation", 0);
    lua_checktablenumber(L, payload->hfov, "hfov", 0);
    lua_pushstring(L, "name"); // array name[32]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 32; i++) { 
        lua_checktableinumber(L, payload->name[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_pushstring(L, "uri"); // array uri[160]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 160; i++) { 
        lua_checktableinumber(L, payload->uri[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    msg_out->crc_extra = FASTMAVLINK_MSG_VIDEO_STREAM_INFORMATION_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_VIDEO_STREAM_INFORMATION_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_VIDEO_STREAM_STATUS: { // #270
    fmav_video_stream_status_t* payload = (fmav_video_stream_status_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->stream_id, "stream_id", 0);
    lua_checktablenumber(L, payload->flags, "flags", 0);
    lua_checktablenumber(L, payload->framerate, "framerate", 0);
    lua_checktablenumber(L, payload->resolution_h, "resolution_h", 0);
    lua_checktablenumber(L, payload->resolution_v, "resolution_v", 0);
    lua_checktablenumber(L, payload->bitrate, "bitrate", 0);
    lua_checktablenumber(L, payload->rotation, "rotation", 0);
    lua_checktablenumber(L, payload->hfov, "hfov", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_VIDEO_STREAM_STATUS_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_VIDEO_STREAM_STATUS_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_CAMERA_FOV_STATUS: { // #271
    fmav_camera_fov_status_t* payload = (fmav_camera_fov_status_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->time_boot_ms, "time_boot_ms", 0);
    lua_checktablenumber(L, payload->lat_camera, "lat_camera", INT32_MAX);
    lua_checktablenumber(L, payload->lon_camera, "lon_camera", INT32_MAX);
    lua_checktablenumber(L, payload->alt_camera, "alt_camera", INT32_MAX);
    lua_checktablenumber(L, payload->lat_image, "lat_image", INT32_MAX);
    lua_checktablenumber(L, payload->lon_image, "lon_image", INT32_MAX);
    lua_checktablenumber(L, payload->alt_image, "alt_image", INT32_MAX);
    lua_pushstring(L, "q"); // array q[4]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 4; i++) { 
        lua_checktableinumber(L, payload->q[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_checktablenumber(L, payload->hfov, "hfov", NAN);
    lua_checktablenumber(L, payload->vfov, "vfov", NAN);
    msg_out->crc_extra = FASTMAVLINK_MSG_CAMERA_FOV_STATUS_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_CAMERA_FOV_STATUS_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_CAMERA_TRACKING_IMAGE_STATUS: { // #275
    fmav_camera_tracking_image_status_t* payload = (fmav_camera_tracking_image_status_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->tracking_status, "tracking_status", 0);
    lua_checktablenumber(L, payload->tracking_mode, "tracking_mode", 0);
    lua_checktablenumber(L, payload->target_data, "target_data", 0);
    lua_checktablenumber(L, payload->point_x, "point_x", NAN);
    lua_checktablenumber(L, payload->point_y, "point_y", NAN);
    lua_checktablenumber(L, payload->radius, "radius", NAN);
    lua_checktablenumber(L, payload->rec_top_x, "rec_top_x", NAN);
    lua_checktablenumber(L, payload->rec_top_y, "rec_top_y", NAN);
    lua_checktablenumber(L, payload->rec_bottom_x, "rec_bottom_x", NAN);
    lua_checktablenumber(L, payload->rec_bottom_y, "rec_bottom_y", NAN);
    msg_out->crc_extra = FASTMAVLINK_MSG_CAMERA_TRACKING_IMAGE_STATUS_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_CAMERA_TRACKING_IMAGE_STATUS_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_CAMERA_TRACKING_GEO_STATUS: { // #276
    fmav_camera_tracking_geo_status_t* payload = (fmav_camera_tracking_geo_status_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->tracking_status, "tracking_status", 0);
    lua_checktablenumber(L, payload->lat, "lat", 0);
    lua_checktablenumber(L, payload->lon, "lon", 0);
    lua_checktablenumber(L, payload->alt, "alt", 0);
    lua_checktablenumber(L, payload->h_acc, "h_acc", NAN);
    lua_checktablenumber(L, payload->v_acc, "v_acc", NAN);
    lua_checktablenumber(L, payload->vel_n, "vel_n", NAN);
    lua_checktablenumber(L, payload->vel_e, "vel_e", NAN);
    lua_checktablenumber(L, payload->vel_d, "vel_d", NAN);
    lua_checktablenumber(L, payload->vel_acc, "vel_acc", NAN);
    lua_checktablenumber(L, payload->dist, "dist", NAN);
    lua_checktablenumber(L, payload->hdg, "hdg", NAN);
    lua_checktablenumber(L, payload->hdg_acc, "hdg_acc", NAN);
    msg_out->crc_extra = FASTMAVLINK_MSG_CAMERA_TRACKING_GEO_STATUS_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_CAMERA_TRACKING_GEO_STATUS_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_GIMBAL_MANAGER_INFORMATION: { // #280
    fmav_gimbal_manager_information_t* payload = (fmav_gimbal_manager_information_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->time_boot_ms, "time_boot_ms", 0);
    lua_checktablenumber(L, payload->cap_flags, "cap_flags", 0);
    lua_checktablenumber(L, payload->gimbal_device_id, "gimbal_device_id", 0);
    lua_checktablenumber(L, payload->roll_min, "roll_min", 0);
    lua_checktablenumber(L, payload->roll_max, "roll_max", 0);
    lua_checktablenumber(L, payload->pitch_min, "pitch_min", 0);
    lua_checktablenumber(L, payload->pitch_max, "pitch_max", 0);
    lua_checktablenumber(L, payload->yaw_min, "yaw_min", 0);
    lua_checktablenumber(L, payload->yaw_max, "yaw_max", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_GIMBAL_MANAGER_INFORMATION_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_GIMBAL_MANAGER_INFORMATION_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS: { // #281
    fmav_gimbal_manager_status_t* payload = (fmav_gimbal_manager_status_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->time_boot_ms, "time_boot_ms", 0);
    lua_checktablenumber(L, payload->flags, "flags", 0);
    lua_checktablenumber(L, payload->gimbal_device_id, "gimbal_device_id", 0);
    lua_checktablenumber(L, payload->primary_control_sysid, "primary_control_sysid", 0);
    lua_checktablenumber(L, payload->primary_control_compid, "primary_control_compid", 0);
    lua_checktablenumber(L, payload->secondary_control_sysid, "secondary_control_sysid", 0);
    lua_checktablenumber(L, payload->secondary_control_compid, "secondary_control_compid", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_GIMBAL_MANAGER_STATUS_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_GIMBAL_MANAGER_STATUS_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_GIMBAL_MANAGER_SET_ATTITUDE: { // #282
    fmav_gimbal_manager_set_attitude_t* payload = (fmav_gimbal_manager_set_attitude_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->target_system, "target_sysid", 0);
    lua_checktablenumber(L, payload->target_component, "target_compid", 0);
    lua_checktablenumber(L, payload->flags, "flags", 0);
    lua_checktablenumber(L, payload->gimbal_device_id, "gimbal_device_id", 0);
    lua_pushstring(L, "q"); // array q[4]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 4; i++) { 
        lua_checktableinumber(L, payload->q[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_checktablenumber(L, payload->angular_velocity_x, "angular_velocity_x", NAN);
    lua_checktablenumber(L, payload->angular_velocity_y, "angular_velocity_y", NAN);
    lua_checktablenumber(L, payload->angular_velocity_z, "angular_velocity_z", NAN);
    msg_out->target_sysid = payload->target_system;
    msg_out->target_compid = payload->target_component;
    msg_out->crc_extra = FASTMAVLINK_MSG_GIMBAL_MANAGER_SET_ATTITUDE_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_GIMBAL_MANAGER_SET_ATTITUDE_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_GIMBAL_DEVICE_INFORMATION: { // #283
    fmav_gimbal_device_information_t* payload = (fmav_gimbal_device_information_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->time_boot_ms, "time_boot_ms", 0);
    lua_pushstring(L, "vendor_name"); // array vendor_name[32]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 32; i++) { 
        lua_checktableinumber(L, payload->vendor_name[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_pushstring(L, "model_name"); // array model_name[32]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 32; i++) { 
        lua_checktableinumber(L, payload->model_name[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_pushstring(L, "custom_name"); // array custom_name[32]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 32; i++) { 
        lua_checktableinumber(L, payload->custom_name[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_checktablenumber(L, payload->firmware_version, "firmware_version", 0);
    lua_checktablenumber(L, payload->hardware_version, "hardware_version", 0);
    lua_checktablenumber(L, payload->uid, "uid", 0);
    lua_checktablenumber(L, payload->cap_flags, "cap_flags", 0);
    lua_checktablenumber(L, payload->custom_cap_flags, "custom_cap_flags", 0);
    lua_checktablenumber(L, payload->roll_min, "roll_min", NAN);
    lua_checktablenumber(L, payload->roll_max, "roll_max", NAN);
    lua_checktablenumber(L, payload->pitch_min, "pitch_min", NAN);
    lua_checktablenumber(L, payload->pitch_max, "pitch_max", NAN);
    lua_checktablenumber(L, payload->yaw_min, "yaw_min", NAN);
    lua_checktablenumber(L, payload->yaw_max, "yaw_max", NAN);
    lua_checktablenumber(L, payload->gimbal_device_id, "gimbal_device_id", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_GIMBAL_DEVICE_INFORMATION_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_GIMBAL_DEVICE_INFORMATION_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_GIMBAL_DEVICE_SET_ATTITUDE: { // #284
    fmav_gimbal_device_set_attitude_t* payload = (fmav_gimbal_device_set_attitude_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->target_system, "target_sysid", 0);
    lua_checktablenumber(L, payload->target_component, "target_compid", 0);
    lua_checktablenumber(L, payload->flags, "flags", 0);
    lua_pushstring(L, "q"); // array q[4]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 4; i++) { 
        lua_checktableinumber(L, payload->q[i], i+1, NAN); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_checktablenumber(L, payload->angular_velocity_x, "angular_velocity_x", NAN);
    lua_checktablenumber(L, payload->angular_velocity_y, "angular_velocity_y", NAN);
    lua_checktablenumber(L, payload->angular_velocity_z, "angular_velocity_z", NAN);
    msg_out->target_sysid = payload->target_system;
    msg_out->target_compid = payload->target_component;
    msg_out->crc_extra = FASTMAVLINK_MSG_GIMBAL_DEVICE_SET_ATTITUDE_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_GIMBAL_DEVICE_SET_ATTITUDE_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS: { // #285
    fmav_gimbal_device_attitude_status_t* payload = (fmav_gimbal_device_attitude_status_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->target_system, "target_sysid", 0);
    lua_checktablenumber(L, payload->target_component, "target_compid", 0);
    lua_checktablenumber(L, payload->time_boot_ms, "time_boot_ms", 0);
    lua_checktablenumber(L, payload->flags, "flags", 0);
    lua_pushstring(L, "q"); // array q[4]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 4; i++) { 
        lua_checktableinumber(L, payload->q[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_checktablenumber(L, payload->angular_velocity_x, "angular_velocity_x", NAN);
    lua_checktablenumber(L, payload->angular_velocity_y, "angular_velocity_y", NAN);
    lua_checktablenumber(L, payload->angular_velocity_z, "angular_velocity_z", NAN);
    lua_checktablenumber(L, payload->failure_flags, "failure_flags", 0);
    lua_checktablenumber(L, payload->delta_yaw, "delta_yaw", NAN);
    lua_checktablenumber(L, payload->delta_yaw_velocity, "delta_yaw_velocity", NAN);
    lua_checktablenumber(L, payload->gimbal_device_id, "gimbal_device_id", 0);
    msg_out->target_sysid = payload->target_system;
    msg_out->target_compid = payload->target_component;
    msg_out->crc_extra = FASTMAVLINK_MSG_GIMBAL_DEVICE_ATTITUDE_STATUS_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_GIMBAL_DEVICE_ATTITUDE_STATUS_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE: { // #286
    fmav_autopilot_state_for_gimbal_device_t* payload = (fmav_autopilot_state_for_gimbal_device_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->target_system, "target_sysid", 0);
    lua_checktablenumber(L, payload->target_component, "target_compid", 0);
    lua_checktablenumber(L, payload->time_boot_us, "time_boot_us", 0);
    lua_pushstring(L, "q"); // array q[4]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 4; i++) { 
        lua_checktableinumber(L, payload->q[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_checktablenumber(L, payload->q_estimated_delay_us, "q_estimated_delay_us", 0);
    lua_checktablenumber(L, payload->vx, "vx", NAN);
    lua_checktablenumber(L, payload->vy, "vy", NAN);
    lua_checktablenumber(L, payload->vz, "vz", NAN);
    lua_checktablenumber(L, payload->v_estimated_delay_us, "v_estimated_delay_us", 0);
    lua_checktablenumber(L, payload->feed_forward_angular_velocity_z, "feed_forward_angular_velocity_z", NAN);
    lua_checktablenumber(L, payload->estimator_status, "estimator_status", 0);
    lua_checktablenumber(L, payload->landed_state, "landed_state", MAV_LANDED_STATE_UNDEFINED);
    lua_checktablenumber(L, payload->angular_velocity_z, "angular_velocity_z", NAN);
    msg_out->target_sysid = payload->target_system;
    msg_out->target_compid = payload->target_component;
    msg_out->crc_extra = FASTMAVLINK_MSG_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_GIMBAL_MANAGER_SET_PITCHYAW: { // #287
    fmav_gimbal_manager_set_pitchyaw_t* payload = (fmav_gimbal_manager_set_pitchyaw_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->target_system, "target_sysid", 0);
    lua_checktablenumber(L, payload->target_component, "target_compid", 0);
    lua_checktablenumber(L, payload->flags, "flags", 0);
    lua_checktablenumber(L, payload->gimbal_device_id, "gimbal_device_id", 0);
    lua_checktablenumber(L, payload->pitch, "pitch", NAN);
    lua_checktablenumber(L, payload->yaw, "yaw", NAN);
    lua_checktablenumber(L, payload->pitch_rate, "pitch_rate", NAN);
    lua_checktablenumber(L, payload->yaw_rate, "yaw_rate", NAN);
    msg_out->target_sysid = payload->target_system;
    msg_out->target_compid = payload->target_component;
    msg_out->crc_extra = FASTMAVLINK_MSG_GIMBAL_MANAGER_SET_PITCHYAW_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_GIMBAL_MANAGER_SET_PITCHYAW_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_GIMBAL_MANAGER_SET_MANUAL_CONTROL: { // #288
    fmav_gimbal_manager_set_manual_control_t* payload = (fmav_gimbal_manager_set_manual_control_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->target_system, "target_sysid", 0);
    lua_checktablenumber(L, payload->target_component, "target_compid", 0);
    lua_checktablenumber(L, payload->flags, "flags", 0);
    lua_checktablenumber(L, payload->gimbal_device_id, "gimbal_device_id", 0);
    lua_checktablenumber(L, payload->pitch, "pitch", NAN);
    lua_checktablenumber(L, payload->yaw, "yaw", NAN);
    lua_checktablenumber(L, payload->pitch_rate, "pitch_rate", NAN);
    lua_checktablenumber(L, payload->yaw_rate, "yaw_rate", NAN);
    msg_out->target_sysid = payload->target_system;
    msg_out->target_compid = payload->target_component;
    msg_out->crc_extra = FASTMAVLINK_MSG_GIMBAL_MANAGER_SET_MANUAL_CONTROL_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_GIMBAL_MANAGER_SET_MANUAL_CONTROL_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_ESC_INFO: { // #290
    fmav_esc_info_t* payload = (fmav_esc_info_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->index, "index", 0);
    lua_checktablenumber(L, payload->time_usec, "time_usec", 0);
    lua_checktablenumber(L, payload->counter, "counter", 0);
    lua_checktablenumber(L, payload->count, "count", 0);
    lua_checktablenumber(L, payload->connection_type, "connection_type", 0);
    lua_checktablenumber(L, payload->info, "info", 0);
    lua_pushstring(L, "failure_flags"); // array failure_flags[4]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 4; i++) { 
        lua_checktableinumber(L, payload->failure_flags[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_pushstring(L, "error_count"); // array error_count[4]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 4; i++) { 
        lua_checktableinumber(L, payload->error_count[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_pushstring(L, "temperature"); // array temperature[4]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 4; i++) { 
        lua_checktableinumber(L, payload->temperature[i], i+1, INT16_MAX); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    msg_out->crc_extra = FASTMAVLINK_MSG_ESC_INFO_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_ESC_INFO_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_ESC_STATUS: { // #291
    fmav_esc_status_t* payload = (fmav_esc_status_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->index, "index", 0);
    lua_checktablenumber(L, payload->time_usec, "time_usec", 0);
    lua_pushstring(L, "rpm"); // array rpm[4]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 4; i++) { 
        lua_checktableinumber(L, payload->rpm[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_pushstring(L, "voltage"); // array voltage[4]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 4; i++) { 
        lua_checktableinumber(L, payload->voltage[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_pushstring(L, "current"); // array current[4]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 4; i++) { 
        lua_checktableinumber(L, payload->current[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    msg_out->crc_extra = FASTMAVLINK_MSG_ESC_STATUS_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_ESC_STATUS_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_WIFI_CONFIG_AP: { // #299
    fmav_wifi_config_ap_t* payload = (fmav_wifi_config_ap_t*)(msg_out->payload);
    lua_pushstring(L, "ssid"); // array ssid[32]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 32; i++) { 
        lua_checktableinumber(L, payload->ssid[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_pushstring(L, "password"); // array password[64]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 64; i++) { 
        lua_checktableinumber(L, payload->password[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_checktablenumber(L, payload->mode, "mode", 0);
    lua_checktablenumber(L, payload->response, "response", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_WIFI_CONFIG_AP_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_WIFI_CONFIG_AP_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_PROTOCOL_VERSION: { // #300
    fmav_protocol_version_t* payload = (fmav_protocol_version_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->version, "version", 0);
    lua_checktablenumber(L, payload->min_version, "min_version", 0);
    lua_checktablenumber(L, payload->max_version, "max_version", 0);
    lua_pushstring(L, "spec_version_hash"); // array spec_version_hash[8]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 8; i++) { 
        lua_checktableinumber(L, payload->spec_version_hash[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_pushstring(L, "library_version_hash"); // array library_version_hash[8]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 8; i++) { 
        lua_checktableinumber(L, payload->library_version_hash[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    msg_out->crc_extra = FASTMAVLINK_MSG_PROTOCOL_VERSION_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_PROTOCOL_VERSION_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_AIS_VESSEL: { // #301
    fmav_ais_vessel_t* payload = (fmav_ais_vessel_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->MMSI, "MMSI", 0);
    lua_checktablenumber(L, payload->lat, "lat", 0);
    lua_checktablenumber(L, payload->lon, "lon", 0);
    lua_checktablenumber(L, payload->COG, "COG", 0);
    lua_checktablenumber(L, payload->heading, "heading", 0);
    lua_checktablenumber(L, payload->velocity, "velocity", 0);
    lua_checktablenumber(L, payload->turn_rate, "turn_rate", 0);
    lua_checktablenumber(L, payload->navigational_status, "navigational_status", 0);
    lua_checktablenumber(L, payload->type, "type", 0);
    lua_checktablenumber(L, payload->dimension_bow, "dimension_bow", 0);
    lua_checktablenumber(L, payload->dimension_stern, "dimension_stern", 0);
    lua_checktablenumber(L, payload->dimension_port, "dimension_port", 0);
    lua_checktablenumber(L, payload->dimension_starboard, "dimension_starboard", 0);
    lua_pushstring(L, "callsign"); // array callsign[7]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 7; i++) { 
        lua_checktableinumber(L, payload->callsign[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_pushstring(L, "name"); // array name[20]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 20; i++) { 
        lua_checktableinumber(L, payload->name[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_checktablenumber(L, payload->tslc, "tslc", 0);
    lua_checktablenumber(L, payload->flags, "flags", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_AIS_VESSEL_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_AIS_VESSEL_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_UAVCAN_NODE_STATUS: { // #310
    fmav_uavcan_node_status_t* payload = (fmav_uavcan_node_status_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->time_usec, "time_usec", 0);
    lua_checktablenumber(L, payload->uptime_sec, "uptime_sec", 0);
    lua_checktablenumber(L, payload->health, "health", 0);
    lua_checktablenumber(L, payload->mode, "mode", 0);
    lua_checktablenumber(L, payload->sub_mode, "sub_mode", 0);
    lua_checktablenumber(L, payload->vendor_specific_status_code, "vendor_specific_status_code", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_UAVCAN_NODE_STATUS_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_UAVCAN_NODE_STATUS_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_UAVCAN_NODE_INFO: { // #311
    fmav_uavcan_node_info_t* payload = (fmav_uavcan_node_info_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->time_usec, "time_usec", 0);
    lua_checktablenumber(L, payload->uptime_sec, "uptime_sec", 0);
    lua_pushstring(L, "name"); // array name[80]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 80; i++) { 
        lua_checktableinumber(L, payload->name[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_checktablenumber(L, payload->hw_version_major, "hw_version_major", 0);
    lua_checktablenumber(L, payload->hw_version_minor, "hw_version_minor", 0);
    lua_pushstring(L, "hw_unique_id"); // array hw_unique_id[16]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 16; i++) { 
        lua_checktableinumber(L, payload->hw_unique_id[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_checktablenumber(L, payload->sw_version_major, "sw_version_major", 0);
    lua_checktablenumber(L, payload->sw_version_minor, "sw_version_minor", 0);
    lua_checktablenumber(L, payload->sw_vcs_commit, "sw_vcs_commit", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_UAVCAN_NODE_INFO_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_UAVCAN_NODE_INFO_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_PARAM_EXT_REQUEST_READ: { // #320
    fmav_param_ext_request_read_t* payload = (fmav_param_ext_request_read_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->target_system, "target_sysid", 0);
    lua_checktablenumber(L, payload->target_component, "target_compid", 0);
    lua_pushstring(L, "param_id"); // array param_id[16]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 16; i++) { 
        lua_checktableinumber(L, payload->param_id[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_checktablenumber(L, payload->param_index, "param_index", -1);
    msg_out->target_sysid = payload->target_system;
    msg_out->target_compid = payload->target_component;
    msg_out->crc_extra = FASTMAVLINK_MSG_PARAM_EXT_REQUEST_READ_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_PARAM_EXT_REQUEST_READ_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_PARAM_EXT_REQUEST_LIST: { // #321
    fmav_param_ext_request_list_t* payload = (fmav_param_ext_request_list_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->target_system, "target_sysid", 0);
    lua_checktablenumber(L, payload->target_component, "target_compid", 0);
    msg_out->target_sysid = payload->target_system;
    msg_out->target_compid = payload->target_component;
    msg_out->crc_extra = FASTMAVLINK_MSG_PARAM_EXT_REQUEST_LIST_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_PARAM_EXT_REQUEST_LIST_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_PARAM_EXT_VALUE: { // #322
    fmav_param_ext_value_t* payload = (fmav_param_ext_value_t*)(msg_out->payload);
    lua_pushstring(L, "param_id"); // array param_id[16]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 16; i++) { 
        lua_checktableinumber(L, payload->param_id[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_pushstring(L, "param_value"); // array param_value[128]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 128; i++) { 
        lua_checktableinumber(L, payload->param_value[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_checktablenumber(L, payload->param_type, "param_type", 0);
    lua_checktablenumber(L, payload->param_count, "param_count", 0);
    lua_checktablenumber(L, payload->param_index, "param_index", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_PARAM_EXT_VALUE_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_PARAM_EXT_VALUE_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_PARAM_EXT_SET: { // #323
    fmav_param_ext_set_t* payload = (fmav_param_ext_set_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->target_system, "target_sysid", 0);
    lua_checktablenumber(L, payload->target_component, "target_compid", 0);
    lua_pushstring(L, "param_id"); // array param_id[16]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 16; i++) { 
        lua_checktableinumber(L, payload->param_id[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_pushstring(L, "param_value"); // array param_value[128]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 128; i++) { 
        lua_checktableinumber(L, payload->param_value[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_checktablenumber(L, payload->param_type, "param_type", 0);
    msg_out->target_sysid = payload->target_system;
    msg_out->target_compid = payload->target_component;
    msg_out->crc_extra = FASTMAVLINK_MSG_PARAM_EXT_SET_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_PARAM_EXT_SET_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_PARAM_EXT_ACK: { // #324
    fmav_param_ext_ack_t* payload = (fmav_param_ext_ack_t*)(msg_out->payload);
    lua_pushstring(L, "param_id"); // array param_id[16]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 16; i++) { 
        lua_checktableinumber(L, payload->param_id[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_pushstring(L, "param_value"); // array param_value[128]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 128; i++) { 
        lua_checktableinumber(L, payload->param_value[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_checktablenumber(L, payload->param_type, "param_type", 0);
    lua_checktablenumber(L, payload->param_result, "param_result", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_PARAM_EXT_ACK_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_PARAM_EXT_ACK_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_OBSTACLE_DISTANCE: { // #330
    fmav_obstacle_distance_t* payload = (fmav_obstacle_distance_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->time_usec, "time_usec", 0);
    lua_checktablenumber(L, payload->sensor_type, "sensor_type", 0);
    lua_pushstring(L, "distances"); // array distances[72]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 72; i++) { 
        lua_checktableinumber(L, payload->distances[i], i+1, UINT16_MAX); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_checktablenumber(L, payload->increment, "increment", 0);
    lua_checktablenumber(L, payload->min_distance, "min_distance", 0);
    lua_checktablenumber(L, payload->max_distance, "max_distance", 0);
    lua_checktablenumber(L, payload->increment_f, "increment_f", 0);
    lua_checktablenumber(L, payload->angle_offset, "angle_offset", 0);
    lua_checktablenumber(L, payload->frame, "frame", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_OBSTACLE_DISTANCE_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_OBSTACLE_DISTANCE_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_ODOMETRY: { // #331
    fmav_odometry_t* payload = (fmav_odometry_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->time_usec, "time_usec", 0);
    lua_checktablenumber(L, payload->frame_id, "frame_id", 0);
    lua_checktablenumber(L, payload->child_frame_id, "child_frame_id", 0);
    lua_checktablenumber(L, payload->x, "x", 0);
    lua_checktablenumber(L, payload->y, "y", 0);
    lua_checktablenumber(L, payload->z, "z", 0);
    lua_pushstring(L, "q"); // array q[4]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 4; i++) { 
        lua_checktableinumber(L, payload->q[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_checktablenumber(L, payload->vx, "vx", 0);
    lua_checktablenumber(L, payload->vy, "vy", 0);
    lua_checktablenumber(L, payload->vz, "vz", 0);
    lua_checktablenumber(L, payload->rollspeed, "rollspeed", 0);
    lua_checktablenumber(L, payload->pitchspeed, "pitchspeed", 0);
    lua_checktablenumber(L, payload->yawspeed, "yawspeed", 0);
    lua_pushstring(L, "pose_covariance"); // array pose_covariance[21]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      lua_checktableinumber(L, payload->pose_covariance[0], 1, NAN); // lua is 1 indexed
      for (int i = 1; i < 21; i++) { 
        lua_checktableinumber(L, payload->pose_covariance[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_pushstring(L, "velocity_covariance"); // array velocity_covariance[21]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      lua_checktableinumber(L, payload->velocity_covariance[0], 1, NAN); // lua is 1 indexed
      for (int i = 1; i < 21; i++) { 
        lua_checktableinumber(L, payload->velocity_covariance[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_checktablenumber(L, payload->reset_counter, "reset_counter", 0);
    lua_checktablenumber(L, payload->estimator_type, "estimator_type", 0);
    lua_checktablenumber(L, payload->quality, "quality", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_ODOMETRY_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_ODOMETRY_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_TRAJECTORY_REPRESENTATION_WAYPOINTS: { // #332
    fmav_trajectory_representation_waypoints_t* payload = (fmav_trajectory_representation_waypoints_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->time_usec, "time_usec", 0);
    lua_checktablenumber(L, payload->valid_points, "valid_points", 0);
    lua_pushstring(L, "pos_x"); // array pos_x[5]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 5; i++) { 
        lua_checktableinumber(L, payload->pos_x[i], i+1, NAN); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_pushstring(L, "pos_y"); // array pos_y[5]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 5; i++) { 
        lua_checktableinumber(L, payload->pos_y[i], i+1, NAN); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_pushstring(L, "pos_z"); // array pos_z[5]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 5; i++) { 
        lua_checktableinumber(L, payload->pos_z[i], i+1, NAN); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_pushstring(L, "vel_x"); // array vel_x[5]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 5; i++) { 
        lua_checktableinumber(L, payload->vel_x[i], i+1, NAN); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_pushstring(L, "vel_y"); // array vel_y[5]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 5; i++) { 
        lua_checktableinumber(L, payload->vel_y[i], i+1, NAN); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_pushstring(L, "vel_z"); // array vel_z[5]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 5; i++) { 
        lua_checktableinumber(L, payload->vel_z[i], i+1, NAN); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_pushstring(L, "acc_x"); // array acc_x[5]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 5; i++) { 
        lua_checktableinumber(L, payload->acc_x[i], i+1, NAN); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_pushstring(L, "acc_y"); // array acc_y[5]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 5; i++) { 
        lua_checktableinumber(L, payload->acc_y[i], i+1, NAN); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_pushstring(L, "acc_z"); // array acc_z[5]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 5; i++) { 
        lua_checktableinumber(L, payload->acc_z[i], i+1, NAN); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_pushstring(L, "pos_yaw"); // array pos_yaw[5]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 5; i++) { 
        lua_checktableinumber(L, payload->pos_yaw[i], i+1, NAN); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_pushstring(L, "vel_yaw"); // array vel_yaw[5]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 5; i++) { 
        lua_checktableinumber(L, payload->vel_yaw[i], i+1, NAN); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_pushstring(L, "command"); // array command[5]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 5; i++) { 
        lua_checktableinumber(L, payload->command[i], i+1, UINT16_MAX); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    msg_out->crc_extra = FASTMAVLINK_MSG_TRAJECTORY_REPRESENTATION_WAYPOINTS_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_TRAJECTORY_REPRESENTATION_WAYPOINTS_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_TRAJECTORY_REPRESENTATION_BEZIER: { // #333
    fmav_trajectory_representation_bezier_t* payload = (fmav_trajectory_representation_bezier_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->time_usec, "time_usec", 0);
    lua_checktablenumber(L, payload->valid_points, "valid_points", 0);
    lua_pushstring(L, "pos_x"); // array pos_x[5]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 5; i++) { 
        lua_checktableinumber(L, payload->pos_x[i], i+1, NAN); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_pushstring(L, "pos_y"); // array pos_y[5]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 5; i++) { 
        lua_checktableinumber(L, payload->pos_y[i], i+1, NAN); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_pushstring(L, "pos_z"); // array pos_z[5]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 5; i++) { 
        lua_checktableinumber(L, payload->pos_z[i], i+1, NAN); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_pushstring(L, "delta"); // array delta[5]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 5; i++) { 
        lua_checktableinumber(L, payload->delta[i], i+1, NAN); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_pushstring(L, "pos_yaw"); // array pos_yaw[5]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 5; i++) { 
        lua_checktableinumber(L, payload->pos_yaw[i], i+1, NAN); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    msg_out->crc_extra = FASTMAVLINK_MSG_TRAJECTORY_REPRESENTATION_BEZIER_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_TRAJECTORY_REPRESENTATION_BEZIER_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_CELLULAR_STATUS: { // #334
    fmav_cellular_status_t* payload = (fmav_cellular_status_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->status, "status", 0);
    lua_checktablenumber(L, payload->failure_reason, "failure_reason", 0);
    lua_checktablenumber(L, payload->type, "type", 0);
    lua_checktablenumber(L, payload->quality, "quality", UINT8_MAX);
    lua_checktablenumber(L, payload->mcc, "mcc", UINT16_MAX);
    lua_checktablenumber(L, payload->mnc, "mnc", UINT16_MAX);
    lua_checktablenumber(L, payload->lac, "lac", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_CELLULAR_STATUS_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_CELLULAR_STATUS_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_ISBD_LINK_STATUS: { // #335
    fmav_isbd_link_status_t* payload = (fmav_isbd_link_status_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->timestamp, "timestamp", 0);
    lua_checktablenumber(L, payload->last_heartbeat, "last_heartbeat", 0);
    lua_checktablenumber(L, payload->failed_sessions, "failed_sessions", 0);
    lua_checktablenumber(L, payload->successful_sessions, "successful_sessions", 0);
    lua_checktablenumber(L, payload->signal_quality, "signal_quality", 0);
    lua_checktablenumber(L, payload->ring_pending, "ring_pending", 0);
    lua_checktablenumber(L, payload->tx_session_pending, "tx_session_pending", 0);
    lua_checktablenumber(L, payload->rx_session_pending, "rx_session_pending", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_ISBD_LINK_STATUS_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_ISBD_LINK_STATUS_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_CELLULAR_CONFIG: { // #336
    fmav_cellular_config_t* payload = (fmav_cellular_config_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->enable_lte, "enable_lte", 0);
    lua_checktablenumber(L, payload->enable_pin, "enable_pin", 0);
    lua_pushstring(L, "pin"); // array pin[16]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 16; i++) { 
        lua_checktableinumber(L, payload->pin[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_pushstring(L, "new_pin"); // array new_pin[16]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 16; i++) { 
        lua_checktableinumber(L, payload->new_pin[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_pushstring(L, "apn"); // array apn[32]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 32; i++) { 
        lua_checktableinumber(L, payload->apn[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_pushstring(L, "puk"); // array puk[16]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 16; i++) { 
        lua_checktableinumber(L, payload->puk[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_checktablenumber(L, payload->roaming, "roaming", 0);
    lua_checktablenumber(L, payload->response, "response", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_CELLULAR_CONFIG_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_CELLULAR_CONFIG_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_RAW_RPM: { // #339
    fmav_raw_rpm_t* payload = (fmav_raw_rpm_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->index, "index", 0);
    lua_checktablenumber(L, payload->frequency, "frequency", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_RAW_RPM_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_RAW_RPM_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_UTM_GLOBAL_POSITION: { // #340
    fmav_utm_global_position_t* payload = (fmav_utm_global_position_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->time, "time", 0);
    lua_pushstring(L, "uas_id"); // array uas_id[18]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 18; i++) { 
        lua_checktableinumber(L, payload->uas_id[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_checktablenumber(L, payload->lat, "lat", 0);
    lua_checktablenumber(L, payload->lon, "lon", 0);
    lua_checktablenumber(L, payload->alt, "alt", 0);
    lua_checktablenumber(L, payload->relative_alt, "relative_alt", 0);
    lua_checktablenumber(L, payload->vx, "vx", 0);
    lua_checktablenumber(L, payload->vy, "vy", 0);
    lua_checktablenumber(L, payload->vz, "vz", 0);
    lua_checktablenumber(L, payload->h_acc, "h_acc", 0);
    lua_checktablenumber(L, payload->v_acc, "v_acc", 0);
    lua_checktablenumber(L, payload->vel_acc, "vel_acc", 0);
    lua_checktablenumber(L, payload->next_lat, "next_lat", 0);
    lua_checktablenumber(L, payload->next_lon, "next_lon", 0);
    lua_checktablenumber(L, payload->next_alt, "next_alt", 0);
    lua_checktablenumber(L, payload->update_rate, "update_rate", 0);
    lua_checktablenumber(L, payload->flight_state, "flight_state", 0);
    lua_checktablenumber(L, payload->flags, "flags", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_UTM_GLOBAL_POSITION_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_UTM_GLOBAL_POSITION_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY: { // #350
    fmav_debug_float_array_t* payload = (fmav_debug_float_array_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->time_usec, "time_usec", 0);
    lua_pushstring(L, "name"); // array name[10]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 10; i++) { 
        lua_checktableinumber(L, payload->name[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_checktablenumber(L, payload->array_id, "array_id", 0);
    lua_pushstring(L, "data"); // array data[58]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 58; i++) { 
        lua_checktableinumber(L, payload->data[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    msg_out->crc_extra = FASTMAVLINK_MSG_DEBUG_FLOAT_ARRAY_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_DEBUG_FLOAT_ARRAY_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_ORBIT_EXECUTION_STATUS: { // #360
    fmav_orbit_execution_status_t* payload = (fmav_orbit_execution_status_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->time_usec, "time_usec", 0);
    lua_checktablenumber(L, payload->radius, "radius", 0);
    lua_checktablenumber(L, payload->frame, "frame", 0);
    lua_checktablenumber(L, payload->x, "x", 0);
    lua_checktablenumber(L, payload->y, "y", 0);
    lua_checktablenumber(L, payload->z, "z", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_ORBIT_EXECUTION_STATUS_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_ORBIT_EXECUTION_STATUS_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_SMART_BATTERY_INFO: { // #370
    fmav_smart_battery_info_t* payload = (fmav_smart_battery_info_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->id, "id", 0);
    lua_checktablenumber(L, payload->battery_function, "battery_function", 0);
    lua_checktablenumber(L, payload->type, "type", 0);
    lua_checktablenumber(L, payload->capacity_full_specification, "capacity_full_specification", -1);
    lua_checktablenumber(L, payload->capacity_full, "capacity_full", -1);
    lua_checktablenumber(L, payload->cycle_count, "cycle_count", UINT16_MAX);
    lua_pushstring(L, "serial_number"); // array serial_number[16]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 16; i++) { 
        lua_checktableinumber(L, payload->serial_number[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_pushstring(L, "device_name"); // array device_name[50]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 50; i++) { 
        lua_checktableinumber(L, payload->device_name[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_checktablenumber(L, payload->weight, "weight", 0);
    lua_checktablenumber(L, payload->discharge_minimum_voltage, "discharge_minimum_voltage", UINT16_MAX);
    lua_checktablenumber(L, payload->charging_minimum_voltage, "charging_minimum_voltage", UINT16_MAX);
    lua_checktablenumber(L, payload->resting_minimum_voltage, "resting_minimum_voltage", UINT16_MAX);
    lua_checktablenumber(L, payload->charging_maximum_voltage, "charging_maximum_voltage", 0);
    lua_checktablenumber(L, payload->cells_in_series, "cells_in_series", 0);
    lua_checktablenumber(L, payload->discharge_maximum_current, "discharge_maximum_current", 0);
    lua_checktablenumber(L, payload->discharge_maximum_burst_current, "discharge_maximum_burst_current", 0);
    lua_pushstring(L, "manufacture_date"); // array manufacture_date[11]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 11; i++) { 
        lua_checktableinumber(L, payload->manufacture_date[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    msg_out->crc_extra = FASTMAVLINK_MSG_SMART_BATTERY_INFO_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_SMART_BATTERY_INFO_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_GENERATOR_STATUS: { // #373
    fmav_generator_status_t* payload = (fmav_generator_status_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->status, "status", 0);
    lua_checktablenumber(L, payload->generator_speed, "generator_speed", UINT16_MAX);
    lua_checktablenumber(L, payload->battery_current, "battery_current", NAN);
    lua_checktablenumber(L, payload->load_current, "load_current", NAN);
    lua_checktablenumber(L, payload->power_generated, "power_generated", NAN);
    lua_checktablenumber(L, payload->bus_voltage, "bus_voltage", 0);
    lua_checktablenumber(L, payload->rectifier_temperature, "rectifier_temperature", INT16_MAX);
    lua_checktablenumber(L, payload->bat_current_setpoint, "bat_current_setpoint", NAN);
    lua_checktablenumber(L, payload->generator_temperature, "generator_temperature", INT16_MAX);
    lua_checktablenumber(L, payload->runtime, "runtime", UINT32_MAX);
    lua_checktablenumber(L, payload->time_until_maintenance, "time_until_maintenance", INT32_MAX);
    msg_out->crc_extra = FASTMAVLINK_MSG_GENERATOR_STATUS_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_GENERATOR_STATUS_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_ACTUATOR_OUTPUT_STATUS: { // #375
    fmav_actuator_output_status_t* payload = (fmav_actuator_output_status_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->time_usec, "time_usec", 0);
    lua_checktablenumber(L, payload->active, "active", 0);
    lua_pushstring(L, "actuator"); // array actuator[32]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 32; i++) { 
        lua_checktableinumber(L, payload->actuator[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    msg_out->crc_extra = FASTMAVLINK_MSG_ACTUATOR_OUTPUT_STATUS_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_ACTUATOR_OUTPUT_STATUS_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_TIME_ESTIMATE_TO_TARGET: { // #380
    fmav_time_estimate_to_target_t* payload = (fmav_time_estimate_to_target_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->safe_return, "safe_return", 0);
    lua_checktablenumber(L, payload->land, "land", 0);
    lua_checktablenumber(L, payload->mission_next_item, "mission_next_item", -1);
    lua_checktablenumber(L, payload->mission_end, "mission_end", -1);
    lua_checktablenumber(L, payload->commanded_action, "commanded_action", -1);
    msg_out->crc_extra = FASTMAVLINK_MSG_TIME_ESTIMATE_TO_TARGET_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_TIME_ESTIMATE_TO_TARGET_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_TUNNEL: { // #385
    fmav_tunnel_t* payload = (fmav_tunnel_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->target_system, "target_sysid", 0);
    lua_checktablenumber(L, payload->target_component, "target_compid", 0);
    lua_checktablenumber(L, payload->payload_type, "payload_type", 0);
    lua_checktablenumber(L, payload->payload_length, "payload_length", 0);
    lua_pushstring(L, "payload"); // array payload[128]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 128; i++) { 
        lua_checktableinumber(L, payload->payload[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    msg_out->target_sysid = payload->target_system;
    msg_out->target_compid = payload->target_component;
    msg_out->crc_extra = FASTMAVLINK_MSG_TUNNEL_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_TUNNEL_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_CAN_FRAME: { // #386
    fmav_can_frame_t* payload = (fmav_can_frame_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->target_system, "target_sysid", 0);
    lua_checktablenumber(L, payload->target_component, "target_compid", 0);
    lua_checktablenumber(L, payload->bus, "bus", 0);
    lua_checktablenumber(L, payload->len, "len", 0);
    lua_checktablenumber(L, payload->id, "id", 0);
    lua_pushstring(L, "data"); // array data[8]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 8; i++) { 
        lua_checktableinumber(L, payload->data[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    msg_out->target_sysid = payload->target_system;
    msg_out->target_compid = payload->target_component;
    msg_out->crc_extra = FASTMAVLINK_MSG_CAN_FRAME_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_CAN_FRAME_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_CANFD_FRAME: { // #387
    fmav_canfd_frame_t* payload = (fmav_canfd_frame_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->target_system, "target_sysid", 0);
    lua_checktablenumber(L, payload->target_component, "target_compid", 0);
    lua_checktablenumber(L, payload->bus, "bus", 0);
    lua_checktablenumber(L, payload->len, "len", 0);
    lua_checktablenumber(L, payload->id, "id", 0);
    lua_pushstring(L, "data"); // array data[64]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 64; i++) { 
        lua_checktableinumber(L, payload->data[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    msg_out->target_sysid = payload->target_system;
    msg_out->target_compid = payload->target_component;
    msg_out->crc_extra = FASTMAVLINK_MSG_CANFD_FRAME_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_CANFD_FRAME_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_CAN_FILTER_MODIFY: { // #388
    fmav_can_filter_modify_t* payload = (fmav_can_filter_modify_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->target_system, "target_sysid", 0);
    lua_checktablenumber(L, payload->target_component, "target_compid", 0);
    lua_checktablenumber(L, payload->bus, "bus", 0);
    lua_checktablenumber(L, payload->operation, "operation", 0);
    lua_checktablenumber(L, payload->num_ids, "num_ids", 0);
    lua_pushstring(L, "ids"); // array ids[16]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 16; i++) { 
        lua_checktableinumber(L, payload->ids[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    msg_out->target_sysid = payload->target_system;
    msg_out->target_compid = payload->target_component;
    msg_out->crc_extra = FASTMAVLINK_MSG_CAN_FILTER_MODIFY_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_CAN_FILTER_MODIFY_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_ONBOARD_COMPUTER_STATUS: { // #390
    fmav_onboard_computer_status_t* payload = (fmav_onboard_computer_status_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->time_usec, "time_usec", 0);
    lua_checktablenumber(L, payload->uptime, "uptime", 0);
    lua_checktablenumber(L, payload->type, "type", 0);
    lua_pushstring(L, "cpu_cores"); // array cpu_cores[8]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 8; i++) { 
        lua_checktableinumber(L, payload->cpu_cores[i], i+1, UINT8_MAX); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_pushstring(L, "cpu_combined"); // array cpu_combined[10]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 10; i++) { 
        lua_checktableinumber(L, payload->cpu_combined[i], i+1, UINT8_MAX); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_pushstring(L, "gpu_cores"); // array gpu_cores[4]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 4; i++) { 
        lua_checktableinumber(L, payload->gpu_cores[i], i+1, UINT8_MAX); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_pushstring(L, "gpu_combined"); // array gpu_combined[10]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 10; i++) { 
        lua_checktableinumber(L, payload->gpu_combined[i], i+1, UINT8_MAX); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_checktablenumber(L, payload->temperature_board, "temperature_board", INT8_MAX);
    lua_pushstring(L, "temperature_core"); // array temperature_core[8]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 8; i++) { 
        lua_checktableinumber(L, payload->temperature_core[i], i+1, INT8_MAX); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_pushstring(L, "fan_speed"); // array fan_speed[4]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 4; i++) { 
        lua_checktableinumber(L, payload->fan_speed[i], i+1, INT16_MAX); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_checktablenumber(L, payload->ram_usage, "ram_usage", UINT32_MAX);
    lua_checktablenumber(L, payload->ram_total, "ram_total", UINT32_MAX);
    lua_pushstring(L, "storage_type"); // array storage_type[4]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 4; i++) { 
        lua_checktableinumber(L, payload->storage_type[i], i+1, UINT32_MAX); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_pushstring(L, "storage_usage"); // array storage_usage[4]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 4; i++) { 
        lua_checktableinumber(L, payload->storage_usage[i], i+1, UINT32_MAX); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_pushstring(L, "storage_total"); // array storage_total[4]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 4; i++) { 
        lua_checktableinumber(L, payload->storage_total[i], i+1, UINT32_MAX); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_pushstring(L, "link_type"); // array link_type[6]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 6; i++) { 
        lua_checktableinumber(L, payload->link_type[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_pushstring(L, "link_tx_rate"); // array link_tx_rate[6]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 6; i++) { 
        lua_checktableinumber(L, payload->link_tx_rate[i], i+1, UINT32_MAX); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_pushstring(L, "link_rx_rate"); // array link_rx_rate[6]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 6; i++) { 
        lua_checktableinumber(L, payload->link_rx_rate[i], i+1, UINT32_MAX); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_pushstring(L, "link_tx_max"); // array link_tx_max[6]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 6; i++) { 
        lua_checktableinumber(L, payload->link_tx_max[i], i+1, UINT32_MAX); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_pushstring(L, "link_rx_max"); // array link_rx_max[6]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 6; i++) { 
        lua_checktableinumber(L, payload->link_rx_max[i], i+1, UINT32_MAX); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    msg_out->crc_extra = FASTMAVLINK_MSG_ONBOARD_COMPUTER_STATUS_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_ONBOARD_COMPUTER_STATUS_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_COMPONENT_INFORMATION: { // #395
    fmav_component_information_t* payload = (fmav_component_information_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->time_boot_ms, "time_boot_ms", 0);
    lua_checktablenumber(L, payload->general_metadata_file_crc, "general_metadata_file_crc", 0);
    lua_pushstring(L, "general_metadata_uri"); // array general_metadata_uri[100]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 100; i++) { 
        lua_checktableinumber(L, payload->general_metadata_uri[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_checktablenumber(L, payload->peripherals_metadata_file_crc, "peripherals_metadata_file_crc", 0);
    lua_pushstring(L, "peripherals_metadata_uri"); // array peripherals_metadata_uri[100]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 100; i++) { 
        lua_checktableinumber(L, payload->peripherals_metadata_uri[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    msg_out->crc_extra = FASTMAVLINK_MSG_COMPONENT_INFORMATION_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_COMPONENT_INFORMATION_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_COMPONENT_METADATA: { // #397
    fmav_component_metadata_t* payload = (fmav_component_metadata_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->time_boot_ms, "time_boot_ms", 0);
    lua_checktablenumber(L, payload->file_crc, "file_crc", 0);
    lua_pushstring(L, "uri"); // array uri[100]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 100; i++) { 
        lua_checktableinumber(L, payload->uri[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    msg_out->crc_extra = FASTMAVLINK_MSG_COMPONENT_METADATA_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_COMPONENT_METADATA_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_PLAY_TUNE_V2: { // #400
    fmav_play_tune_v2_t* payload = (fmav_play_tune_v2_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->target_system, "target_sysid", 0);
    lua_checktablenumber(L, payload->target_component, "target_compid", 0);
    lua_checktablenumber(L, payload->format, "format", 0);
    lua_pushstring(L, "tune"); // array tune[248]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 248; i++) { 
        lua_checktableinumber(L, payload->tune[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    msg_out->target_sysid = payload->target_system;
    msg_out->target_compid = payload->target_component;
    msg_out->crc_extra = FASTMAVLINK_MSG_PLAY_TUNE_V2_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_PLAY_TUNE_V2_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_SUPPORTED_TUNES: { // #401
    fmav_supported_tunes_t* payload = (fmav_supported_tunes_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->target_system, "target_sysid", 0);
    lua_checktablenumber(L, payload->target_component, "target_compid", 0);
    lua_checktablenumber(L, payload->format, "format", 0);
    msg_out->target_sysid = payload->target_system;
    msg_out->target_compid = payload->target_component;
    msg_out->crc_extra = FASTMAVLINK_MSG_SUPPORTED_TUNES_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_SUPPORTED_TUNES_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_EVENT: { // #410
    fmav_event_t* payload = (fmav_event_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->destination_component, "destination_component", 0);
    lua_checktablenumber(L, payload->destination_system, "destination_system", 0);
    lua_checktablenumber(L, payload->id, "id", 0);
    lua_checktablenumber(L, payload->event_time_boot_ms, "event_time_boot_ms", 0);
    lua_checktablenumber(L, payload->sequence, "sequence", 0);
    lua_checktablenumber(L, payload->log_levels, "log_levels", 0);
    lua_pushstring(L, "arguments"); // array arguments[40]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 40; i++) { 
        lua_checktableinumber(L, payload->arguments[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    msg_out->crc_extra = FASTMAVLINK_MSG_EVENT_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_EVENT_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_CURRENT_EVENT_SEQUENCE: { // #411
    fmav_current_event_sequence_t* payload = (fmav_current_event_sequence_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->sequence, "sequence", 0);
    lua_checktablenumber(L, payload->flags, "flags", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_CURRENT_EVENT_SEQUENCE_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_CURRENT_EVENT_SEQUENCE_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_REQUEST_EVENT: { // #412
    fmav_request_event_t* payload = (fmav_request_event_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->target_system, "target_sysid", 0);
    lua_checktablenumber(L, payload->target_component, "target_compid", 0);
    lua_checktablenumber(L, payload->first_sequence, "first_sequence", 0);
    lua_checktablenumber(L, payload->last_sequence, "last_sequence", 0);
    msg_out->target_sysid = payload->target_system;
    msg_out->target_compid = payload->target_component;
    msg_out->crc_extra = FASTMAVLINK_MSG_REQUEST_EVENT_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_REQUEST_EVENT_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_RESPONSE_EVENT_ERROR: { // #413
    fmav_response_event_error_t* payload = (fmav_response_event_error_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->target_system, "target_sysid", 0);
    lua_checktablenumber(L, payload->target_component, "target_compid", 0);
    lua_checktablenumber(L, payload->sequence, "sequence", 0);
    lua_checktablenumber(L, payload->sequence_oldest_available, "sequence_oldest_available", 0);
    lua_checktablenumber(L, payload->reason, "reason", 0);
    msg_out->target_sysid = payload->target_system;
    msg_out->target_compid = payload->target_component;
    msg_out->crc_extra = FASTMAVLINK_MSG_RESPONSE_EVENT_ERROR_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_RESPONSE_EVENT_ERROR_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_RADIO_RC_CHANNELS_DEV: { // #420
    fmav_radio_rc_channels_dev_t* payload = (fmav_radio_rc_channels_dev_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->target_system, "target_sysid", 0);
    lua_checktablenumber(L, payload->target_component, "target_compid", 0);
    lua_checktablenumber(L, payload->count, "count", 0);
    lua_checktablenumber(L, payload->flags, "flags", 0);
    lua_pushstring(L, "channels"); // array channels[32]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 32; i++) { 
        lua_checktableinumber(L, payload->channels[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    msg_out->target_sysid = payload->target_system;
    msg_out->target_compid = payload->target_component;
    msg_out->crc_extra = FASTMAVLINK_MSG_RADIO_RC_CHANNELS_DEV_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_RADIO_RC_CHANNELS_DEV_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_RADIO_LINK_STATS_DEV: { // #421
    fmav_radio_link_stats_dev_t* payload = (fmav_radio_link_stats_dev_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->target_system, "target_sysid", 0);
    lua_checktablenumber(L, payload->target_component, "target_compid", 0);
    lua_checktablenumber(L, payload->flags, "flags", 0);
    lua_checktablenumber(L, payload->rx_LQ_rc, "rx_LQ_rc", UINT8_MAX);
    lua_checktablenumber(L, payload->rx_LQ_ser, "rx_LQ_ser", UINT8_MAX);
    lua_checktablenumber(L, payload->rx_rssi1, "rx_rssi1", UINT8_MAX);
    lua_checktablenumber(L, payload->rx_snr1, "rx_snr1", INT8_MAX);
    lua_checktablenumber(L, payload->rx_rssi2, "rx_rssi2", UINT8_MAX);
    lua_checktablenumber(L, payload->rx_snr2, "rx_snr2", INT8_MAX);
    lua_checktablenumber(L, payload->rx_receive_antenna, "rx_receive_antenna", UINT8_MAX);
    lua_checktablenumber(L, payload->rx_transmit_antenna, "rx_transmit_antenna", UINT8_MAX);
    lua_checktablenumber(L, payload->rx_power, "rx_power", INT8_MAX);
    lua_checktablenumber(L, payload->tx_LQ_ser, "tx_LQ_ser", UINT8_MAX);
    lua_checktablenumber(L, payload->tx_rssi1, "tx_rssi1", UINT8_MAX);
    lua_checktablenumber(L, payload->tx_snr1, "tx_snr1", INT8_MAX);
    lua_checktablenumber(L, payload->tx_rssi2, "tx_rssi2", UINT8_MAX);
    lua_checktablenumber(L, payload->tx_snr2, "tx_snr2", INT8_MAX);
    lua_checktablenumber(L, payload->tx_receive_antenna, "tx_receive_antenna", UINT8_MAX);
    lua_checktablenumber(L, payload->tx_transmit_antenna, "tx_transmit_antenna", UINT8_MAX);
    lua_checktablenumber(L, payload->tx_power, "tx_power", INT8_MAX);
    msg_out->target_sysid = payload->target_system;
    msg_out->target_compid = payload->target_component;
    msg_out->crc_extra = FASTMAVLINK_MSG_RADIO_LINK_STATS_DEV_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_RADIO_LINK_STATS_DEV_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_RADIO_LINK_INFORMATION_DEV: { // #422
    fmav_radio_link_information_dev_t* payload = (fmav_radio_link_information_dev_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->target_system, "target_sysid", 0);
    lua_checktablenumber(L, payload->target_component, "target_compid", 0);
    lua_checktablenumber(L, payload->type, "type", 0);
    lua_checktablenumber(L, payload->mode, "mode", UINT8_MAX);
    lua_checktablenumber(L, payload->tx_rate, "tx_rate", 0);
    lua_checktablenumber(L, payload->rx_rate, "rx_rate", 0);
    lua_checktablenumber(L, payload->tx_receive_sensitivity, "tx_receive_sensitivity", 0);
    lua_checktablenumber(L, payload->rx_receive_sensitivity, "rx_receive_sensitivity", 0);
    msg_out->target_sysid = payload->target_system;
    msg_out->target_compid = payload->target_component;
    msg_out->crc_extra = FASTMAVLINK_MSG_RADIO_LINK_INFORMATION_DEV_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_RADIO_LINK_INFORMATION_DEV_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_WHEEL_DISTANCE: { // #9000
    fmav_wheel_distance_t* payload = (fmav_wheel_distance_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->time_usec, "time_usec", 0);
    lua_checktablenumber(L, payload->count, "count", 0);
    lua_pushstring(L, "distance"); // array distance[16]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 16; i++) { 
        lua_checktableinumber(L, payload->distance[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    msg_out->crc_extra = FASTMAVLINK_MSG_WHEEL_DISTANCE_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_WHEEL_DISTANCE_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_WINCH_STATUS: { // #9005
    fmav_winch_status_t* payload = (fmav_winch_status_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->time_usec, "time_usec", 0);
    lua_checktablenumber(L, payload->line_length, "line_length", NAN);
    lua_checktablenumber(L, payload->speed, "speed", NAN);
    lua_checktablenumber(L, payload->tension, "tension", NAN);
    lua_checktablenumber(L, payload->voltage, "voltage", NAN);
    lua_checktablenumber(L, payload->current, "current", NAN);
    lua_checktablenumber(L, payload->temperature, "temperature", INT16_MAX);
    lua_checktablenumber(L, payload->status, "status", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_WINCH_STATUS_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_WINCH_STATUS_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG: { // #10001
    fmav_uavionix_adsb_out_cfg_t* payload = (fmav_uavionix_adsb_out_cfg_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->ICAO, "ICAO", 0);
    lua_pushstring(L, "callsign"); // array callsign[9]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 9; i++) { 
        lua_checktableinumber(L, payload->callsign[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_checktablenumber(L, payload->emitterType, "emitterType", 0);
    lua_checktablenumber(L, payload->aircraftSize, "aircraftSize", 0);
    lua_checktablenumber(L, payload->gpsOffsetLat, "gpsOffsetLat", 0);
    lua_checktablenumber(L, payload->gpsOffsetLon, "gpsOffsetLon", 0);
    lua_checktablenumber(L, payload->stallSpeed, "stallSpeed", 0);
    lua_checktablenumber(L, payload->rfSelect, "rfSelect", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_CFG_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_CFG_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_DYNAMIC: { // #10002
    fmav_uavionix_adsb_out_dynamic_t* payload = (fmav_uavionix_adsb_out_dynamic_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->utcTime, "utcTime", 0);
    lua_checktablenumber(L, payload->gpsLat, "gpsLat", 0);
    lua_checktablenumber(L, payload->gpsLon, "gpsLon", 0);
    lua_checktablenumber(L, payload->gpsAlt, "gpsAlt", 0);
    lua_checktablenumber(L, payload->gpsFix, "gpsFix", 0);
    lua_checktablenumber(L, payload->numSats, "numSats", 0);
    lua_checktablenumber(L, payload->baroAltMSL, "baroAltMSL", 0);
    lua_checktablenumber(L, payload->accuracyHor, "accuracyHor", 0);
    lua_checktablenumber(L, payload->accuracyVert, "accuracyVert", 0);
    lua_checktablenumber(L, payload->accuracyVel, "accuracyVel", 0);
    lua_checktablenumber(L, payload->velVert, "velVert", 0);
    lua_checktablenumber(L, payload->velNS, "velNS", 0);
    lua_checktablenumber(L, payload->VelEW, "VelEW", 0);
    lua_checktablenumber(L, payload->emergencyStatus, "emergencyStatus", 0);
    lua_checktablenumber(L, payload->state, "state", 0);
    lua_checktablenumber(L, payload->squawk, "squawk", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_DYNAMIC_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_DYNAMIC_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT: { // #10003
    fmav_uavionix_adsb_transceiver_health_report_t* payload = (fmav_uavionix_adsb_transceiver_health_report_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->rfHealth, "rfHealth", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_DEVICE_OP_READ: { // #11000
    fmav_device_op_read_t* payload = (fmav_device_op_read_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->target_system, "target_sysid", 0);
    lua_checktablenumber(L, payload->target_component, "target_compid", 0);
    lua_checktablenumber(L, payload->request_id, "request_id", 0);
    lua_checktablenumber(L, payload->bustype, "bustype", 0);
    lua_checktablenumber(L, payload->bus, "bus", 0);
    lua_checktablenumber(L, payload->address, "address", 0);
    lua_pushstring(L, "busname"); // array busname[40]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 40; i++) { 
        lua_checktableinumber(L, payload->busname[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_checktablenumber(L, payload->regstart, "regstart", 0);
    lua_checktablenumber(L, payload->count, "count", 0);
    lua_checktablenumber(L, payload->bank, "bank", 0);
    msg_out->target_sysid = payload->target_system;
    msg_out->target_compid = payload->target_component;
    msg_out->crc_extra = FASTMAVLINK_MSG_DEVICE_OP_READ_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_DEVICE_OP_READ_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_DEVICE_OP_READ_REPLY: { // #11001
    fmav_device_op_read_reply_t* payload = (fmav_device_op_read_reply_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->request_id, "request_id", 0);
    lua_checktablenumber(L, payload->result, "result", 0);
    lua_checktablenumber(L, payload->regstart, "regstart", 0);
    lua_checktablenumber(L, payload->count, "count", 0);
    lua_pushstring(L, "data"); // array data[128]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 128; i++) { 
        lua_checktableinumber(L, payload->data[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_checktablenumber(L, payload->bank, "bank", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_DEVICE_OP_READ_REPLY_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_DEVICE_OP_READ_REPLY_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_DEVICE_OP_WRITE: { // #11002
    fmav_device_op_write_t* payload = (fmav_device_op_write_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->target_system, "target_sysid", 0);
    lua_checktablenumber(L, payload->target_component, "target_compid", 0);
    lua_checktablenumber(L, payload->request_id, "request_id", 0);
    lua_checktablenumber(L, payload->bustype, "bustype", 0);
    lua_checktablenumber(L, payload->bus, "bus", 0);
    lua_checktablenumber(L, payload->address, "address", 0);
    lua_pushstring(L, "busname"); // array busname[40]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 40; i++) { 
        lua_checktableinumber(L, payload->busname[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_checktablenumber(L, payload->regstart, "regstart", 0);
    lua_checktablenumber(L, payload->count, "count", 0);
    lua_pushstring(L, "data"); // array data[128]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 128; i++) { 
        lua_checktableinumber(L, payload->data[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_checktablenumber(L, payload->bank, "bank", 0);
    msg_out->target_sysid = payload->target_system;
    msg_out->target_compid = payload->target_component;
    msg_out->crc_extra = FASTMAVLINK_MSG_DEVICE_OP_WRITE_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_DEVICE_OP_WRITE_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_DEVICE_OP_WRITE_REPLY: { // #11003
    fmav_device_op_write_reply_t* payload = (fmav_device_op_write_reply_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->request_id, "request_id", 0);
    lua_checktablenumber(L, payload->result, "result", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_DEVICE_OP_WRITE_REPLY_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_DEVICE_OP_WRITE_REPLY_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_ADAP_TUNING: { // #11010
    fmav_adap_tuning_t* payload = (fmav_adap_tuning_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->axis, "axis", 0);
    lua_checktablenumber(L, payload->desired, "desired", 0);
    lua_checktablenumber(L, payload->achieved, "achieved", 0);
    lua_checktablenumber(L, payload->error, "error", 0);
    lua_checktablenumber(L, payload->theta, "theta", 0);
    lua_checktablenumber(L, payload->omega, "omega", 0);
    lua_checktablenumber(L, payload->sigma, "sigma", 0);
    lua_checktablenumber(L, payload->theta_dot, "theta_dot", 0);
    lua_checktablenumber(L, payload->omega_dot, "omega_dot", 0);
    lua_checktablenumber(L, payload->sigma_dot, "sigma_dot", 0);
    lua_checktablenumber(L, payload->f, "f", 0);
    lua_checktablenumber(L, payload->f_dot, "f_dot", 0);
    lua_checktablenumber(L, payload->u, "u", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_ADAP_TUNING_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_ADAP_TUNING_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_VISION_POSITION_DELTA: { // #11011
    fmav_vision_position_delta_t* payload = (fmav_vision_position_delta_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->time_usec, "time_usec", 0);
    lua_checktablenumber(L, payload->time_delta_usec, "time_delta_usec", 0);
    lua_pushstring(L, "angle_delta"); // array angle_delta[3]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 3; i++) { 
        lua_checktableinumber(L, payload->angle_delta[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_pushstring(L, "position_delta"); // array position_delta[3]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 3; i++) { 
        lua_checktableinumber(L, payload->position_delta[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_checktablenumber(L, payload->confidence, "confidence", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_VISION_POSITION_DELTA_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_VISION_POSITION_DELTA_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_AOA_SSA: { // #11020
    fmav_aoa_ssa_t* payload = (fmav_aoa_ssa_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->time_usec, "time_usec", 0);
    lua_checktablenumber(L, payload->AOA, "AOA", 0);
    lua_checktablenumber(L, payload->SSA, "SSA", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_AOA_SSA_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_AOA_SSA_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_ESC_TELEMETRY_1_TO_4: { // #11030
    fmav_esc_telemetry_1_to_4_t* payload = (fmav_esc_telemetry_1_to_4_t*)(msg_out->payload);
    lua_pushstring(L, "temperature"); // array temperature[4]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 4; i++) { 
        lua_checktableinumber(L, payload->temperature[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_pushstring(L, "voltage"); // array voltage[4]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 4; i++) { 
        lua_checktableinumber(L, payload->voltage[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_pushstring(L, "current"); // array current[4]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 4; i++) { 
        lua_checktableinumber(L, payload->current[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_pushstring(L, "totalcurrent"); // array totalcurrent[4]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 4; i++) { 
        lua_checktableinumber(L, payload->totalcurrent[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_pushstring(L, "rpm"); // array rpm[4]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 4; i++) { 
        lua_checktableinumber(L, payload->rpm[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_pushstring(L, "count"); // array count[4]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 4; i++) { 
        lua_checktableinumber(L, payload->count[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    msg_out->crc_extra = FASTMAVLINK_MSG_ESC_TELEMETRY_1_TO_4_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_ESC_TELEMETRY_1_TO_4_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_ESC_TELEMETRY_5_TO_8: { // #11031
    fmav_esc_telemetry_5_to_8_t* payload = (fmav_esc_telemetry_5_to_8_t*)(msg_out->payload);
    lua_pushstring(L, "temperature"); // array temperature[4]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 4; i++) { 
        lua_checktableinumber(L, payload->temperature[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_pushstring(L, "voltage"); // array voltage[4]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 4; i++) { 
        lua_checktableinumber(L, payload->voltage[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_pushstring(L, "current"); // array current[4]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 4; i++) { 
        lua_checktableinumber(L, payload->current[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_pushstring(L, "totalcurrent"); // array totalcurrent[4]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 4; i++) { 
        lua_checktableinumber(L, payload->totalcurrent[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_pushstring(L, "rpm"); // array rpm[4]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 4; i++) { 
        lua_checktableinumber(L, payload->rpm[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_pushstring(L, "count"); // array count[4]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 4; i++) { 
        lua_checktableinumber(L, payload->count[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    msg_out->crc_extra = FASTMAVLINK_MSG_ESC_TELEMETRY_5_TO_8_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_ESC_TELEMETRY_5_TO_8_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_ESC_TELEMETRY_9_TO_12: { // #11032
    fmav_esc_telemetry_9_to_12_t* payload = (fmav_esc_telemetry_9_to_12_t*)(msg_out->payload);
    lua_pushstring(L, "temperature"); // array temperature[4]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 4; i++) { 
        lua_checktableinumber(L, payload->temperature[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_pushstring(L, "voltage"); // array voltage[4]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 4; i++) { 
        lua_checktableinumber(L, payload->voltage[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_pushstring(L, "current"); // array current[4]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 4; i++) { 
        lua_checktableinumber(L, payload->current[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_pushstring(L, "totalcurrent"); // array totalcurrent[4]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 4; i++) { 
        lua_checktableinumber(L, payload->totalcurrent[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_pushstring(L, "rpm"); // array rpm[4]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 4; i++) { 
        lua_checktableinumber(L, payload->rpm[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_pushstring(L, "count"); // array count[4]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 4; i++) { 
        lua_checktableinumber(L, payload->count[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    msg_out->crc_extra = FASTMAVLINK_MSG_ESC_TELEMETRY_9_TO_12_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_ESC_TELEMETRY_9_TO_12_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_OSD_PARAM_CONFIG: { // #11033
    fmav_osd_param_config_t* payload = (fmav_osd_param_config_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->target_system, "target_sysid", 0);
    lua_checktablenumber(L, payload->target_component, "target_compid", 0);
    lua_checktablenumber(L, payload->request_id, "request_id", 0);
    lua_checktablenumber(L, payload->osd_screen, "osd_screen", 0);
    lua_checktablenumber(L, payload->osd_index, "osd_index", 0);
    lua_pushstring(L, "param_id"); // array param_id[16]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 16; i++) { 
        lua_checktableinumber(L, payload->param_id[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_checktablenumber(L, payload->config_type, "config_type", 0);
    lua_checktablenumber(L, payload->min_value, "min_value", 0);
    lua_checktablenumber(L, payload->max_value, "max_value", 0);
    lua_checktablenumber(L, payload->increment, "increment", 0);
    msg_out->target_sysid = payload->target_system;
    msg_out->target_compid = payload->target_component;
    msg_out->crc_extra = FASTMAVLINK_MSG_OSD_PARAM_CONFIG_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_OSD_PARAM_CONFIG_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_OSD_PARAM_CONFIG_REPLY: { // #11034
    fmav_osd_param_config_reply_t* payload = (fmav_osd_param_config_reply_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->request_id, "request_id", 0);
    lua_checktablenumber(L, payload->result, "result", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_OSD_PARAM_CONFIG_REPLY_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_OSD_PARAM_CONFIG_REPLY_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_OSD_PARAM_SHOW_CONFIG: { // #11035
    fmav_osd_param_show_config_t* payload = (fmav_osd_param_show_config_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->target_system, "target_sysid", 0);
    lua_checktablenumber(L, payload->target_component, "target_compid", 0);
    lua_checktablenumber(L, payload->request_id, "request_id", 0);
    lua_checktablenumber(L, payload->osd_screen, "osd_screen", 0);
    lua_checktablenumber(L, payload->osd_index, "osd_index", 0);
    msg_out->target_sysid = payload->target_system;
    msg_out->target_compid = payload->target_component;
    msg_out->crc_extra = FASTMAVLINK_MSG_OSD_PARAM_SHOW_CONFIG_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_OSD_PARAM_SHOW_CONFIG_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_OSD_PARAM_SHOW_CONFIG_REPLY: { // #11036
    fmav_osd_param_show_config_reply_t* payload = (fmav_osd_param_show_config_reply_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->request_id, "request_id", 0);
    lua_checktablenumber(L, payload->result, "result", 0);
    lua_pushstring(L, "param_id"); // array param_id[16]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 16; i++) { 
        lua_checktableinumber(L, payload->param_id[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_checktablenumber(L, payload->config_type, "config_type", 0);
    lua_checktablenumber(L, payload->min_value, "min_value", 0);
    lua_checktablenumber(L, payload->max_value, "max_value", 0);
    lua_checktablenumber(L, payload->increment, "increment", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_OSD_PARAM_SHOW_CONFIG_REPLY_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_OSD_PARAM_SHOW_CONFIG_REPLY_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_OBSTACLE_DISTANCE_3D: { // #11037
    fmav_obstacle_distance_3d_t* payload = (fmav_obstacle_distance_3d_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->time_boot_ms, "time_boot_ms", 0);
    lua_checktablenumber(L, payload->sensor_type, "sensor_type", 0);
    lua_checktablenumber(L, payload->frame, "frame", 0);
    lua_checktablenumber(L, payload->obstacle_id, "obstacle_id", 0);
    lua_checktablenumber(L, payload->x, "x", 0);
    lua_checktablenumber(L, payload->y, "y", 0);
    lua_checktablenumber(L, payload->z, "z", 0);
    lua_checktablenumber(L, payload->min_distance, "min_distance", 0);
    lua_checktablenumber(L, payload->max_distance, "max_distance", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_OBSTACLE_DISTANCE_3D_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_OBSTACLE_DISTANCE_3D_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_WATER_DEPTH: { // #11038
    fmav_water_depth_t* payload = (fmav_water_depth_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->time_boot_ms, "time_boot_ms", 0);
    lua_checktablenumber(L, payload->id, "id", 0);
    lua_checktablenumber(L, payload->healthy, "healthy", 0);
    lua_checktablenumber(L, payload->lat, "lat", 0);
    lua_checktablenumber(L, payload->lng, "lng", 0);
    lua_checktablenumber(L, payload->alt, "alt", 0);
    lua_checktablenumber(L, payload->roll, "roll", 0);
    lua_checktablenumber(L, payload->pitch, "pitch", 0);
    lua_checktablenumber(L, payload->yaw, "yaw", 0);
    lua_checktablenumber(L, payload->distance, "distance", 0);
    lua_checktablenumber(L, payload->temperature, "temperature", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_WATER_DEPTH_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_WATER_DEPTH_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_MCU_STATUS: { // #11039
    fmav_mcu_status_t* payload = (fmav_mcu_status_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->id, "id", 0);
    lua_checktablenumber(L, payload->MCU_temperature, "MCU_temperature", 0);
    lua_checktablenumber(L, payload->MCU_voltage, "MCU_voltage", 0);
    lua_checktablenumber(L, payload->MCU_voltage_min, "MCU_voltage_min", 0);
    lua_checktablenumber(L, payload->MCU_voltage_max, "MCU_voltage_max", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_MCU_STATUS_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_MCU_STATUS_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_OPEN_DRONE_ID_BASIC_ID: { // #12900
    fmav_open_drone_id_basic_id_t* payload = (fmav_open_drone_id_basic_id_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->target_system, "target_sysid", 0);
    lua_checktablenumber(L, payload->target_component, "target_compid", 0);
    lua_pushstring(L, "id_or_mac"); // array id_or_mac[20]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 20; i++) { 
        lua_checktableinumber(L, payload->id_or_mac[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_checktablenumber(L, payload->id_type, "id_type", 0);
    lua_checktablenumber(L, payload->ua_type, "ua_type", 0);
    lua_pushstring(L, "uas_id"); // array uas_id[20]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 20; i++) { 
        lua_checktableinumber(L, payload->uas_id[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    msg_out->target_sysid = payload->target_system;
    msg_out->target_compid = payload->target_component;
    msg_out->crc_extra = FASTMAVLINK_MSG_OPEN_DRONE_ID_BASIC_ID_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_OPEN_DRONE_ID_BASIC_ID_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_OPEN_DRONE_ID_LOCATION: { // #12901
    fmav_open_drone_id_location_t* payload = (fmav_open_drone_id_location_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->target_system, "target_sysid", 0);
    lua_checktablenumber(L, payload->target_component, "target_compid", 0);
    lua_pushstring(L, "id_or_mac"); // array id_or_mac[20]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 20; i++) { 
        lua_checktableinumber(L, payload->id_or_mac[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_checktablenumber(L, payload->status, "status", 0);
    lua_checktablenumber(L, payload->direction, "direction", 36100);
    lua_checktablenumber(L, payload->speed_horizontal, "speed_horizontal", 0);
    lua_checktablenumber(L, payload->speed_vertical, "speed_vertical", 0);
    lua_checktablenumber(L, payload->latitude, "latitude", 0);
    lua_checktablenumber(L, payload->longitude, "longitude", 0);
    lua_checktablenumber(L, payload->altitude_barometric, "altitude_barometric", -1000);
    lua_checktablenumber(L, payload->altitude_geodetic, "altitude_geodetic", -1000);
    lua_checktablenumber(L, payload->height_reference, "height_reference", 0);
    lua_checktablenumber(L, payload->height, "height", -1000);
    lua_checktablenumber(L, payload->horizontal_accuracy, "horizontal_accuracy", 0);
    lua_checktablenumber(L, payload->vertical_accuracy, "vertical_accuracy", 0);
    lua_checktablenumber(L, payload->barometer_accuracy, "barometer_accuracy", 0);
    lua_checktablenumber(L, payload->speed_accuracy, "speed_accuracy", 0);
    lua_checktablenumber(L, payload->timestamp, "timestamp", 0xFFFF);
    lua_checktablenumber(L, payload->timestamp_accuracy, "timestamp_accuracy", 0);
    msg_out->target_sysid = payload->target_system;
    msg_out->target_compid = payload->target_component;
    msg_out->crc_extra = FASTMAVLINK_MSG_OPEN_DRONE_ID_LOCATION_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_OPEN_DRONE_ID_LOCATION_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_OPEN_DRONE_ID_AUTHENTICATION: { // #12902
    fmav_open_drone_id_authentication_t* payload = (fmav_open_drone_id_authentication_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->target_system, "target_sysid", 0);
    lua_checktablenumber(L, payload->target_component, "target_compid", 0);
    lua_pushstring(L, "id_or_mac"); // array id_or_mac[20]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 20; i++) { 
        lua_checktableinumber(L, payload->id_or_mac[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_checktablenumber(L, payload->authentication_type, "authentication_type", 0);
    lua_checktablenumber(L, payload->data_page, "data_page", 0);
    lua_checktablenumber(L, payload->last_page_index, "last_page_index", 0);
    lua_checktablenumber(L, payload->length, "length", 0);
    lua_checktablenumber(L, payload->timestamp, "timestamp", 0);
    lua_pushstring(L, "authentication_data"); // array authentication_data[23]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 23; i++) { 
        lua_checktableinumber(L, payload->authentication_data[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    msg_out->target_sysid = payload->target_system;
    msg_out->target_compid = payload->target_component;
    msg_out->crc_extra = FASTMAVLINK_MSG_OPEN_DRONE_ID_AUTHENTICATION_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_OPEN_DRONE_ID_AUTHENTICATION_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_OPEN_DRONE_ID_SELF_ID: { // #12903
    fmav_open_drone_id_self_id_t* payload = (fmav_open_drone_id_self_id_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->target_system, "target_sysid", 0);
    lua_checktablenumber(L, payload->target_component, "target_compid", 0);
    lua_pushstring(L, "id_or_mac"); // array id_or_mac[20]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 20; i++) { 
        lua_checktableinumber(L, payload->id_or_mac[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_checktablenumber(L, payload->description_type, "description_type", 0);
    lua_pushstring(L, "description"); // array description[23]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 23; i++) { 
        lua_checktableinumber(L, payload->description[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    msg_out->target_sysid = payload->target_system;
    msg_out->target_compid = payload->target_component;
    msg_out->crc_extra = FASTMAVLINK_MSG_OPEN_DRONE_ID_SELF_ID_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_OPEN_DRONE_ID_SELF_ID_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM: { // #12904
    fmav_open_drone_id_system_t* payload = (fmav_open_drone_id_system_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->target_system, "target_sysid", 0);
    lua_checktablenumber(L, payload->target_component, "target_compid", 0);
    lua_pushstring(L, "id_or_mac"); // array id_or_mac[20]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 20; i++) { 
        lua_checktableinumber(L, payload->id_or_mac[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_checktablenumber(L, payload->operator_location_type, "operator_location_type", 0);
    lua_checktablenumber(L, payload->classification_type, "classification_type", 0);
    lua_checktablenumber(L, payload->operator_latitude, "operator_latitude", 0);
    lua_checktablenumber(L, payload->operator_longitude, "operator_longitude", 0);
    lua_checktablenumber(L, payload->area_count, "area_count", 0);
    lua_checktablenumber(L, payload->area_radius, "area_radius", 0);
    lua_checktablenumber(L, payload->area_ceiling, "area_ceiling", -1000);
    lua_checktablenumber(L, payload->area_floor, "area_floor", -1000);
    lua_checktablenumber(L, payload->category_eu, "category_eu", 0);
    lua_checktablenumber(L, payload->class_eu, "class_eu", 0);
    lua_checktablenumber(L, payload->operator_altitude_geo, "operator_altitude_geo", -1000);
    lua_checktablenumber(L, payload->timestamp, "timestamp", 0);
    msg_out->target_sysid = payload->target_system;
    msg_out->target_compid = payload->target_component;
    msg_out->crc_extra = FASTMAVLINK_MSG_OPEN_DRONE_ID_SYSTEM_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_OPEN_DRONE_ID_SYSTEM_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_OPEN_DRONE_ID_OPERATOR_ID: { // #12905
    fmav_open_drone_id_operator_id_t* payload = (fmav_open_drone_id_operator_id_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->target_system, "target_sysid", 0);
    lua_checktablenumber(L, payload->target_component, "target_compid", 0);
    lua_pushstring(L, "id_or_mac"); // array id_or_mac[20]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 20; i++) { 
        lua_checktableinumber(L, payload->id_or_mac[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_checktablenumber(L, payload->operator_id_type, "operator_id_type", 0);
    lua_pushstring(L, "operator_id"); // array operator_id[20]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 20; i++) { 
        lua_checktableinumber(L, payload->operator_id[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    msg_out->target_sysid = payload->target_system;
    msg_out->target_compid = payload->target_component;
    msg_out->crc_extra = FASTMAVLINK_MSG_OPEN_DRONE_ID_OPERATOR_ID_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_OPEN_DRONE_ID_OPERATOR_ID_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_OPEN_DRONE_ID_MESSAGE_PACK: { // #12915
    fmav_open_drone_id_message_pack_t* payload = (fmav_open_drone_id_message_pack_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->target_system, "target_sysid", 0);
    lua_checktablenumber(L, payload->target_component, "target_compid", 0);
    lua_pushstring(L, "id_or_mac"); // array id_or_mac[20]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 20; i++) { 
        lua_checktableinumber(L, payload->id_or_mac[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_checktablenumber(L, payload->single_message_size, "single_message_size", 0);
    lua_checktablenumber(L, payload->msg_pack_size, "msg_pack_size", 0);
    lua_pushstring(L, "messages"); // array messages[225]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 225; i++) { 
        lua_checktableinumber(L, payload->messages[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    msg_out->target_sysid = payload->target_system;
    msg_out->target_compid = payload->target_component;
    msg_out->crc_extra = FASTMAVLINK_MSG_OPEN_DRONE_ID_MESSAGE_PACK_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_OPEN_DRONE_ID_MESSAGE_PACK_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_OPEN_DRONE_ID_ARM_STATUS: { // #12918
    fmav_open_drone_id_arm_status_t* payload = (fmav_open_drone_id_arm_status_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->status, "status", 0);
    lua_pushstring(L, "error"); // array error[50]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 50; i++) { 
        lua_checktableinumber(L, payload->error[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    msg_out->crc_extra = FASTMAVLINK_MSG_OPEN_DRONE_ID_ARM_STATUS_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_OPEN_DRONE_ID_ARM_STATUS_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM_UPDATE: { // #12919
    fmav_open_drone_id_system_update_t* payload = (fmav_open_drone_id_system_update_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->target_system, "target_sysid", 0);
    lua_checktablenumber(L, payload->target_component, "target_compid", 0);
    lua_checktablenumber(L, payload->operator_latitude, "operator_latitude", 0);
    lua_checktablenumber(L, payload->operator_longitude, "operator_longitude", 0);
    lua_checktablenumber(L, payload->operator_altitude_geo, "operator_altitude_geo", -1000);
    lua_checktablenumber(L, payload->timestamp, "timestamp", 0);
    msg_out->target_sysid = payload->target_system;
    msg_out->target_compid = payload->target_component;
    msg_out->crc_extra = FASTMAVLINK_MSG_OPEN_DRONE_ID_SYSTEM_UPDATE_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_OPEN_DRONE_ID_SYSTEM_UPDATE_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_HYGROMETER_SENSOR: { // #12920
    fmav_hygrometer_sensor_t* payload = (fmav_hygrometer_sensor_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->id, "id", 0);
    lua_checktablenumber(L, payload->temperature, "temperature", 0);
    lua_checktablenumber(L, payload->humidity, "humidity", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_HYGROMETER_SENSOR_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_HYGROMETER_SENSOR_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_ICAROUS_HEARTBEAT: { // #42000
    fmav_icarous_heartbeat_t* payload = (fmav_icarous_heartbeat_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->status, "status", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_ICAROUS_HEARTBEAT_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_ICAROUS_HEARTBEAT_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_ICAROUS_KINEMATIC_BANDS: { // #42001
    fmav_icarous_kinematic_bands_t* payload = (fmav_icarous_kinematic_bands_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->numBands, "numBands", 0);
    lua_checktablenumber(L, payload->type1, "type1", 0);
    lua_checktablenumber(L, payload->min1, "min1", 0);
    lua_checktablenumber(L, payload->max1, "max1", 0);
    lua_checktablenumber(L, payload->type2, "type2", 0);
    lua_checktablenumber(L, payload->min2, "min2", 0);
    lua_checktablenumber(L, payload->max2, "max2", 0);
    lua_checktablenumber(L, payload->type3, "type3", 0);
    lua_checktablenumber(L, payload->min3, "min3", 0);
    lua_checktablenumber(L, payload->max3, "max3", 0);
    lua_checktablenumber(L, payload->type4, "type4", 0);
    lua_checktablenumber(L, payload->min4, "min4", 0);
    lua_checktablenumber(L, payload->max4, "max4", 0);
    lua_checktablenumber(L, payload->type5, "type5", 0);
    lua_checktablenumber(L, payload->min5, "min5", 0);
    lua_checktablenumber(L, payload->max5, "max5", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_ICAROUS_KINEMATIC_BANDS_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_ICAROUS_KINEMATIC_BANDS_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_CUBEPILOT_RAW_RC: { // #50001
    fmav_cubepilot_raw_rc_t* payload = (fmav_cubepilot_raw_rc_t*)(msg_out->payload);
    lua_pushstring(L, "rc_raw"); // array rc_raw[32]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 32; i++) { 
        lua_checktableinumber(L, payload->rc_raw[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    msg_out->crc_extra = FASTMAVLINK_MSG_CUBEPILOT_RAW_RC_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_CUBEPILOT_RAW_RC_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_HERELINK_VIDEO_STREAM_INFORMATION: { // #50002
    fmav_herelink_video_stream_information_t* payload = (fmav_herelink_video_stream_information_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->camera_id, "camera_id", 0);
    lua_checktablenumber(L, payload->status, "status", 0);
    lua_checktablenumber(L, payload->framerate, "framerate", 0);
    lua_checktablenumber(L, payload->resolution_h, "resolution_h", 0);
    lua_checktablenumber(L, payload->resolution_v, "resolution_v", 0);
    lua_checktablenumber(L, payload->bitrate, "bitrate", 0);
    lua_checktablenumber(L, payload->rotation, "rotation", 0);
    lua_pushstring(L, "uri"); // array uri[230]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 230; i++) { 
        lua_checktableinumber(L, payload->uri[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    msg_out->crc_extra = FASTMAVLINK_MSG_HERELINK_VIDEO_STREAM_INFORMATION_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_HERELINK_VIDEO_STREAM_INFORMATION_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_HERELINK_TELEM: { // #50003
    fmav_herelink_telem_t* payload = (fmav_herelink_telem_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->rssi, "rssi", 0);
    lua_checktablenumber(L, payload->snr, "snr", 0);
    lua_checktablenumber(L, payload->rf_freq, "rf_freq", 0);
    lua_checktablenumber(L, payload->link_bw, "link_bw", 0);
    lua_checktablenumber(L, payload->link_rate, "link_rate", 0);
    lua_checktablenumber(L, payload->cpu_temp, "cpu_temp", 0);
    lua_checktablenumber(L, payload->board_temp, "board_temp", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_HERELINK_TELEM_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_HERELINK_TELEM_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_START: { // #50004
    fmav_cubepilot_firmware_update_start_t* payload = (fmav_cubepilot_firmware_update_start_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->target_system, "target_sysid", 0);
    lua_checktablenumber(L, payload->target_component, "target_compid", 0);
    lua_checktablenumber(L, payload->size, "size", 0);
    lua_checktablenumber(L, payload->crc, "crc", 0);
    msg_out->target_sysid = payload->target_system;
    msg_out->target_compid = payload->target_component;
    msg_out->crc_extra = FASTMAVLINK_MSG_CUBEPILOT_FIRMWARE_UPDATE_START_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_CUBEPILOT_FIRMWARE_UPDATE_START_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_RESP: { // #50005
    fmav_cubepilot_firmware_update_resp_t* payload = (fmav_cubepilot_firmware_update_resp_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->target_system, "target_sysid", 0);
    lua_checktablenumber(L, payload->target_component, "target_compid", 0);
    lua_checktablenumber(L, payload->offset, "offset", 0);
    msg_out->target_sysid = payload->target_system;
    msg_out->target_compid = payload->target_component;
    msg_out->crc_extra = FASTMAVLINK_MSG_CUBEPILOT_FIRMWARE_UPDATE_RESP_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_CUBEPILOT_FIRMWARE_UPDATE_RESP_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_AIRLINK_AUTH: { // #52000
    fmav_airlink_auth_t* payload = (fmav_airlink_auth_t*)(msg_out->payload);
    lua_pushstring(L, "login"); // array login[50]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 50; i++) { 
        lua_checktableinumber(L, payload->login[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_pushstring(L, "password"); // array password[50]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 50; i++) { 
        lua_checktableinumber(L, payload->password[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    msg_out->crc_extra = FASTMAVLINK_MSG_AIRLINK_AUTH_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_AIRLINK_AUTH_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_AIRLINK_AUTH_RESPONSE: { // #52001
    fmav_airlink_auth_response_t* payload = (fmav_airlink_auth_response_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->resp_type, "resp_type", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_AIRLINK_AUTH_RESPONSE_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_AIRLINK_AUTH_RESPONSE_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_AIRLINK_EYE_GS_HOLE_PUSH_REQUEST: { // #52002
    fmav_airlink_eye_gs_hole_push_request_t* payload = (fmav_airlink_eye_gs_hole_push_request_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->resp_type, "resp_type", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_AIRLINK_EYE_GS_HOLE_PUSH_REQUEST_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_AIRLINK_EYE_GS_HOLE_PUSH_REQUEST_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_AIRLINK_EYE_GS_HOLE_PUSH_RESPONSE: { // #52003
    fmav_airlink_eye_gs_hole_push_response_t* payload = (fmav_airlink_eye_gs_hole_push_response_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->resp_type, "resp_type", 0);
    lua_checktablenumber(L, payload->ip_version, "ip_version", 0);
    lua_pushstring(L, "ip_address_4"); // array ip_address_4[4]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 4; i++) { 
        lua_checktableinumber(L, payload->ip_address_4[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_pushstring(L, "ip_address_6"); // array ip_address_6[16]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 16; i++) { 
        lua_checktableinumber(L, payload->ip_address_6[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_checktablenumber(L, payload->ip_port, "ip_port", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_AIRLINK_EYE_GS_HOLE_PUSH_RESPONSE_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_AIRLINK_EYE_GS_HOLE_PUSH_RESPONSE_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_AIRLINK_EYE_HP: { // #52004
    fmav_airlink_eye_hp_t* payload = (fmav_airlink_eye_hp_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->resp_type, "resp_type", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_AIRLINK_EYE_HP_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_AIRLINK_EYE_HP_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_AIRLINK_EYE_TURN_INIT: { // #52005
    fmav_airlink_eye_turn_init_t* payload = (fmav_airlink_eye_turn_init_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->resp_type, "resp_type", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_AIRLINK_EYE_TURN_INIT_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_AIRLINK_EYE_TURN_INIT_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_EXT: { // #60000
    fmav_autopilot_state_for_gimbal_device_ext_t* payload = (fmav_autopilot_state_for_gimbal_device_ext_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->target_system, "target_sysid", 0);
    lua_checktablenumber(L, payload->target_component, "target_compid", 0);
    lua_checktablenumber(L, payload->time_boot_us, "time_boot_us", 0);
    lua_checktablenumber(L, payload->wind_x, "wind_x", NAN);
    lua_checktablenumber(L, payload->wind_y, "wind_y", NAN);
    lua_checktablenumber(L, payload->wind_correction_angle, "wind_correction_angle", NAN);
    msg_out->target_sysid = payload->target_system;
    msg_out->target_compid = payload->target_component;
    msg_out->crc_extra = FASTMAVLINK_MSG_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_EXT_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_EXT_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_INFORMATION: { // #60010
    fmav_storm32_gimbal_manager_information_t* payload = (fmav_storm32_gimbal_manager_information_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->gimbal_id, "gimbal_id", 0);
    lua_checktablenumber(L, payload->device_cap_flags, "device_cap_flags", 0);
    lua_checktablenumber(L, payload->manager_cap_flags, "manager_cap_flags", 0);
    lua_checktablenumber(L, payload->roll_min, "roll_min", NAN);
    lua_checktablenumber(L, payload->roll_max, "roll_max", NAN);
    lua_checktablenumber(L, payload->pitch_min, "pitch_min", NAN);
    lua_checktablenumber(L, payload->pitch_max, "pitch_max", NAN);
    lua_checktablenumber(L, payload->yaw_min, "yaw_min", NAN);
    lua_checktablenumber(L, payload->yaw_max, "yaw_max", NAN);
    msg_out->crc_extra = FASTMAVLINK_MSG_STORM32_GIMBAL_MANAGER_INFORMATION_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_STORM32_GIMBAL_MANAGER_INFORMATION_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_STATUS: { // #60011
    fmav_storm32_gimbal_manager_status_t* payload = (fmav_storm32_gimbal_manager_status_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->gimbal_id, "gimbal_id", 0);
    lua_checktablenumber(L, payload->supervisor, "supervisor", 0);
    lua_checktablenumber(L, payload->device_flags, "device_flags", 0);
    lua_checktablenumber(L, payload->manager_flags, "manager_flags", 0);
    lua_checktablenumber(L, payload->profile, "profile", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_STORM32_GIMBAL_MANAGER_STATUS_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_STORM32_GIMBAL_MANAGER_STATUS_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CONTROL: { // #60012
    fmav_storm32_gimbal_manager_control_t* payload = (fmav_storm32_gimbal_manager_control_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->target_system, "target_sysid", 0);
    lua_checktablenumber(L, payload->target_component, "target_compid", 0);
    lua_checktablenumber(L, payload->gimbal_id, "gimbal_id", 0);
    lua_checktablenumber(L, payload->client, "client", 0);
    lua_checktablenumber(L, payload->device_flags, "device_flags", UINT16_MAX);
    lua_checktablenumber(L, payload->manager_flags, "manager_flags", 0);
    lua_pushstring(L, "q"); // array q[4]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      lua_checktableinumber(L, payload->q[0], 1, NAN); // lua is 1 indexed
      for (int i = 1; i < 4; i++) { 
        lua_checktableinumber(L, payload->q[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    lua_checktablenumber(L, payload->angular_velocity_x, "angular_velocity_x", NAN);
    lua_checktablenumber(L, payload->angular_velocity_y, "angular_velocity_y", NAN);
    lua_checktablenumber(L, payload->angular_velocity_z, "angular_velocity_z", NAN);
    msg_out->target_sysid = payload->target_system;
    msg_out->target_compid = payload->target_component;
    msg_out->crc_extra = FASTMAVLINK_MSG_STORM32_GIMBAL_MANAGER_CONTROL_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_STORM32_GIMBAL_MANAGER_CONTROL_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CONTROL_PITCHYAW: { // #60013
    fmav_storm32_gimbal_manager_control_pitchyaw_t* payload = (fmav_storm32_gimbal_manager_control_pitchyaw_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->target_system, "target_sysid", 0);
    lua_checktablenumber(L, payload->target_component, "target_compid", 0);
    lua_checktablenumber(L, payload->gimbal_id, "gimbal_id", 0);
    lua_checktablenumber(L, payload->client, "client", 0);
    lua_checktablenumber(L, payload->device_flags, "device_flags", UINT16_MAX);
    lua_checktablenumber(L, payload->manager_flags, "manager_flags", 0);
    lua_checktablenumber(L, payload->pitch, "pitch", NAN);
    lua_checktablenumber(L, payload->yaw, "yaw", NAN);
    lua_checktablenumber(L, payload->pitch_rate, "pitch_rate", NAN);
    lua_checktablenumber(L, payload->yaw_rate, "yaw_rate", NAN);
    msg_out->target_sysid = payload->target_system;
    msg_out->target_compid = payload->target_component;
    msg_out->crc_extra = FASTMAVLINK_MSG_STORM32_GIMBAL_MANAGER_CONTROL_PITCHYAW_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_STORM32_GIMBAL_MANAGER_CONTROL_PITCHYAW_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CORRECT_ROLL: { // #60014
    fmav_storm32_gimbal_manager_correct_roll_t* payload = (fmav_storm32_gimbal_manager_correct_roll_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->target_system, "target_sysid", 0);
    lua_checktablenumber(L, payload->target_component, "target_compid", 0);
    lua_checktablenumber(L, payload->gimbal_id, "gimbal_id", 0);
    lua_checktablenumber(L, payload->client, "client", 0);
    lua_checktablenumber(L, payload->roll, "roll", 0);
    msg_out->target_sysid = payload->target_system;
    msg_out->target_compid = payload->target_component;
    msg_out->crc_extra = FASTMAVLINK_MSG_STORM32_GIMBAL_MANAGER_CORRECT_ROLL_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_STORM32_GIMBAL_MANAGER_CORRECT_ROLL_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_QSHOT_STATUS: { // #60020
    fmav_qshot_status_t* payload = (fmav_qshot_status_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->mode, "mode", 0);
    lua_checktablenumber(L, payload->shot_state, "shot_state", 0);
    msg_out->crc_extra = FASTMAVLINK_MSG_QSHOT_STATUS_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_QSHOT_STATUS_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_FRSKY_PASSTHROUGH_ARRAY: { // #60040
    fmav_frsky_passthrough_array_t* payload = (fmav_frsky_passthrough_array_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->time_boot_ms, "time_boot_ms", 0);
    lua_checktablenumber(L, payload->count, "count", 0);
    lua_pushstring(L, "packet_buf"); // array packet_buf[240]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 240; i++) { 
        lua_checktableinumber(L, payload->packet_buf[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    msg_out->crc_extra = FASTMAVLINK_MSG_FRSKY_PASSTHROUGH_ARRAY_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_FRSKY_PASSTHROUGH_ARRAY_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_PARAM_VALUE_ARRAY: { // #60041
    fmav_param_value_array_t* payload = (fmav_param_value_array_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->param_count, "param_count", 0);
    lua_checktablenumber(L, payload->param_index_first, "param_index_first", 0);
    lua_checktablenumber(L, payload->param_array_len, "param_array_len", 0);
    lua_checktablenumber(L, payload->flags, "flags", 0);
    lua_pushstring(L, "packet_buf"); // array packet_buf[248]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 248; i++) { 
        lua_checktableinumber(L, payload->packet_buf[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    msg_out->crc_extra = FASTMAVLINK_MSG_PARAM_VALUE_ARRAY_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_PARAM_VALUE_ARRAY_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_RADIO_RC_CHANNELS: { // #60045
    fmav_radio_rc_channels_t* payload = (fmav_radio_rc_channels_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->count, "count", 0);
    lua_checktablenumber(L, payload->flags, "flags", 0);
    lua_pushstring(L, "channels"); // array channels[24]
    lua_gettable(L, -2);
    if (lua_istable(L,-1)) {
      for (int i = 0; i < 24; i++) { 
        lua_checktableinumber(L, payload->channels[i], i+1, 0); // lua is 1 indexed
      }
    }
    lua_pop(L, 1);
    msg_out->crc_extra = FASTMAVLINK_MSG_RADIO_RC_CHANNELS_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_RADIO_RC_CHANNELS_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_RADIO_LINK_STATS: { // #60046
    fmav_radio_link_stats_t* payload = (fmav_radio_link_stats_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->flags, "flags", 0);
    lua_checktablenumber(L, payload->rx_LQ, "rx_LQ", UINT8_MAX);
    lua_checktablenumber(L, payload->rx_rssi1, "rx_rssi1", UINT8_MAX);
    lua_checktablenumber(L, payload->rx_snr1, "rx_snr1", INT8_MAX);
    lua_checktablenumber(L, payload->rx_rssi2, "rx_rssi2", UINT8_MAX);
    lua_checktablenumber(L, payload->rx_snr2, "rx_snr2", INT8_MAX);
    lua_checktablenumber(L, payload->rx_receive_antenna, "rx_receive_antenna", UINT8_MAX);
    lua_checktablenumber(L, payload->rx_transmit_antenna, "rx_transmit_antenna", UINT8_MAX);
    lua_checktablenumber(L, payload->tx_LQ, "tx_LQ", UINT8_MAX);
    lua_checktablenumber(L, payload->tx_rssi1, "tx_rssi1", UINT8_MAX);
    lua_checktablenumber(L, payload->tx_snr1, "tx_snr1", INT8_MAX);
    lua_checktablenumber(L, payload->tx_rssi2, "tx_rssi2", UINT8_MAX);
    lua_checktablenumber(L, payload->tx_snr2, "tx_snr2", INT8_MAX);
    lua_checktablenumber(L, payload->tx_receive_antenna, "tx_receive_antenna", UINT8_MAX);
    lua_checktablenumber(L, payload->tx_transmit_antenna, "tx_transmit_antenna", UINT8_MAX);
    msg_out->crc_extra = FASTMAVLINK_MSG_RADIO_LINK_STATS_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_RADIO_LINK_STATS_PAYLOAD_LEN_MAX;
    return 1;
    }
  case FASTMAVLINK_MSG_ID_RADIO_LINK_FLOW_CONTROL: { // #60047
    fmav_radio_link_flow_control_t* payload = (fmav_radio_link_flow_control_t*)(msg_out->payload);
    lua_checktablenumber(L, payload->tx_rate, "tx_rate", UINT16_MAX);
    lua_checktablenumber(L, payload->rx_rate, "rx_rate", UINT16_MAX);
    lua_checktablenumber(L, payload->tx_used_bandwidth, "tx_used_bandwidth", UINT8_MAX);
    lua_checktablenumber(L, payload->rx_used_bandwidth, "rx_used_bandwidth", UINT8_MAX);
    lua_checktablenumber(L, payload->txbuf, "txbuf", UINT8_MAX);
    msg_out->crc_extra = FASTMAVLINK_MSG_RADIO_LINK_FLOW_CONTROL_CRCEXTRA;
    msg_out->payload_max_len = FASTMAVLINK_MSG_RADIO_LINK_FLOW_CONTROL_PAYLOAD_LEN_MAX;
    return 1;
    }
  }
  return 0;
}


