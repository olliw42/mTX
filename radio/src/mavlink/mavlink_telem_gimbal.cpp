//*******************************************************
// mTX: MAVLink for OpenTx Project
// Copyright (c) OlliW, OlliW42, www.olliw.eu
// LGPL3
// https://www.gnu.org/licenses/lgpl-3.0.en.html
// MavTelem Library
//*******************************************************

#include "opentx.h"

constexpr float FPI = 3.141592653589793f;
constexpr float FDEGTORAD = FPI/180.0f;
constexpr float FRADTODEG = 180.0f/FPI;

#define INCU8(x)    if ((x) < UINT8_MAX) { (x)++; }

// quaternions and euler angles
// we do not use NED (roll-pitch-yaw) to convert quaternion to Euler angles and vice versa
// we use pitch-roll-yaw instead
// when the roll angle is zero, both are equivalent, this should be the majority of cases anyhow
// also, for most gimbals pitch-roll-yaw is appropriate
// the issue with NED is the gimbal lock at pitch +-90�, but pitch +-90� is a common operation point for gimbals
// the angles we store in this lib are thus pitch-roll-yaw Euler

//NED
void calc_q_from_NED_angles_deg(float* q, float roll_deg, float pitch_deg, float yaw_deg)
{
  float cr2 = cosf(roll_deg * 0.5f * FDEGTORAD);
  float sr2 = sinf(roll_deg * 0.5f * FDEGTORAD);
  float cp2 = cosf(pitch_deg * 0.5f * FDEGTORAD);
  float sp2 = sinf(pitch_deg * 0.5f * FDEGTORAD);
  float cy2 = cosf(yaw_deg * 0.5f * FDEGTORAD);
  float sy2 = sinf(yaw_deg * 0.5f * FDEGTORAD);

  q[0] = cr2*cp2*cy2 + sr2*sp2*sy2;
  q[1] = sr2*cp2*cy2 - cr2*sp2*sy2;
  q[2] = cr2*sp2*cy2 + sr2*cp2*sy2;
  q[3] = cr2*cp2*sy2 - sr2*sp2*cy2;
}

//equal for NED and G
void calc_q_from_pitchyaw_deg(float* q, float pitch_deg, float yaw_deg)
{
  float cp2 = cosf(pitch_deg * 0.5f * FDEGTORAD);
  float sp2 = sinf(pitch_deg * 0.5f * FDEGTORAD);
  float cy2 = cosf(yaw_deg * 0.5f * FDEGTORAD);
  float sy2 = sinf(yaw_deg * 0.5f * FDEGTORAD);

  q[0] = cp2*cy2;
  q[1] = -sp2*sy2;
  q[2] = sp2*cy2;
  q[3] = cp2*sy2;
}

void calc_NED_angles_deg_from_q(float* roll_deg, float* pitch_deg, float* yaw_deg, float* q)
{
  *roll_deg = atan2f( 2.0f*(q[0]*q[1] + q[2]*q[3]), 1.0f - 2.0f*(q[1]*q[1] + q[2]*q[2]) ) * FRADTODEG;

  float arg = 2.0f*(q[0]*q[2] - q[1]*q[3]);
  if (isnan(arg)) {
     *pitch_deg = 0.0f;
  } 
  else if (arg >= 1.0f) {
    *pitch_deg = 90.0f;
  } 
  else if (arg <= -1.0f) {
    *pitch_deg = -90.0f;
  } 
  else {
    *pitch_deg = asinf(arg) * FRADTODEG;
  }

  *yaw_deg = atan2f( 2.0f*(q[0]*q[3] + q[1]*q[2]), 1.0f - 2.0f*(q[2]*q[2] + q[3]*q[3]) ) * FRADTODEG;
}

void calc_G_angles_deg_from_q(float* roll_deg, float* pitch_deg, float* yaw_deg, float* q)
{
  *pitch_deg = atan2f( q[0]*q[2] - q[1]*q[3], 0.5f - q[1]*q[1] - q[2]*q[2] ) * FRADTODEG;

  float arg = 2.0f*(q[0]*q[1] + q[2]*q[3]);
  if (isnan(arg)) {
     *roll_deg = 0.0f;
  } 
  else if (arg >= 1.0f) {
    *roll_deg = 90.0f;
  } 
  else if (arg <= -1.0f) {
    *roll_deg = -90.0f;
  } 
  else {
    *roll_deg = asinf(arg) * FRADTODEG;
  }

  *yaw_deg = atan2f( q[0]*q[3] - q[1]*q[2], 0.5f - q[1]*q[1] - q[3]*q[3] ) * FRADTODEG;
}

// -- Generate MAVLink messages --
// these should never be called directly, should only be called by the task handler

void MavlinkTelem::generateCmdDoMountConfigure(uint8_t tsystem, uint8_t tcomponent, uint8_t mode)
{
  _generateCmdLong(tsystem, tcomponent,
      MAV_CMD_DO_MOUNT_CONFIGURE,
      mode, 0,0,0,0,0,0
      );
}

// ArduPilot: if a mount has no pan control, then this will also yaw the copter in guided mode overwriting _fixed_yaw !!
void MavlinkTelem::generateCmdDoMountControl(uint8_t tsystem, uint8_t tcomponent, float pitch_deg, float yaw_deg, uint8_t mode)
{
  _generateCmdLong(tsystem, tcomponent,
      MAV_CMD_DO_MOUNT_CONTROL,
      pitch_deg, 0.0f, yaw_deg, 0,0,0, mode
      );
}

// gimbal protocol v2: gimbal device

void MavlinkTelem::generateCmdRequestGimbalDeviceInformation(uint8_t tsystem, uint8_t tcomponent)
{
  _generateCmdLong(tsystem, tcomponent,
      MAV_CMD_REQUEST_MESSAGE,
      FASTMAVLINK_MSG_ID_GIMBAL_DEVICE_INFORMATION, 0,0,0,0,0,0
      );
}

void MavlinkTelem::generateGimbalDeviceSetAttitude(uint8_t tsystem, uint8_t tcomponent,
    float pitch_deg, float yaw_deg, uint16_t flags)
{
float q[4];

  if ((pitch_deg != NAN) && (yaw_deg != NAN)) {
    calc_q_from_pitchyaw_deg(q, pitch_deg, yaw_deg);
  } 
  else {
    q[0] = q[1] = q[2] = q[3] = NAN;
  }

  fmav_msg_gimbal_device_set_attitude_pack(
      &_msg_out, _my_sysid, _my_compid, //_sys_id and not _my_sysid !!! we mimic being part of the autopilot system
      tsystem, tcomponent,
      flags,
      q,
      NAN, NAN, NAN,
      &_status_out
      );
  _msg_out_available = true;
}

// gimbal protocol v2: gimbal manager

void MavlinkTelem::generateCmdRequestGimbalManagerInformation(uint8_t tsystem, uint8_t tcomponent)
{
  _generateCmdLong(tsystem, tcomponent,
      MAV_CMD_REQUEST_MESSAGE,
      FASTMAVLINK_MSG_ID_GIMBAL_MANAGER_INFORMATION, 0,0,0,0,0,0
      );
}

// we have 3 ways to control the gimbal
// - GIMBAL_MANAGER_SET_PITCHYAW: uses pitch & yaw Euler angles. This is the canonical method.
// - GIMBAL_MANAGER_SET_ATTITUDE: uses a quaternion
// - MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW: uses pitch & yaw Euler angles
// we map all three to the same interface

void MavlinkTelem::generateGimbalManagerSetPitchYaw(uint8_t tsystem, uint8_t tcomponent,
    uint8_t gimbal_id, float pitch_deg, float yaw_deg, uint16_t device_flags)
{
float pitch_rad, yaw_rad;

  if ((pitch_deg != NAN) && (yaw_deg != NAN)) {
      pitch_rad = pitch_deg * FDEGTORAD;
      yaw_rad = yaw_deg * FDEGTORAD;
  } 
  else {
      pitch_rad = yaw_rad = NAN;
  }

  fmav_msg_gimbal_manager_set_pitchyaw_pack(
      &_msg_out, _my_sysid, _my_compid,
      tsystem, tcomponent,
      device_flags,
      gimbal_id,
      pitch_rad, yaw_rad,
      NAN, NAN,
      &_status_out
      );
  _msg_out_available = true;
}

void MavlinkTelem::generateGimbalManagerSetAttitude(uint8_t tsystem, uint8_t tcomponent,
    uint8_t gimbal_id, float pitch_deg, float yaw_deg, uint16_t device_flags)
{
float q[4];

  if ((pitch_deg != NAN) && (yaw_deg != NAN)) {
    calc_q_from_pitchyaw_deg(q, pitch_deg, yaw_deg);
  }
  else {
    q[0] = q[1] = q[2] = q[3] = NAN;
  }

  fmav_msg_gimbal_manager_set_attitude_pack(
      &_msg_out, _my_sysid, _my_compid,
      tsystem, tcomponent,
      device_flags,
      gimbal_id,
      q,
      NAN, NAN, NAN,
      &_status_out
      );
  _msg_out_available = true;
}

void MavlinkTelem::generateCmdDoGimbalManagerPitchYaw(uint8_t tsystem, uint8_t tcomponent,
    uint8_t gimbal_id, float pitch_deg, float yaw_deg, uint16_t device_flags)
{
  if ((pitch_deg == NAN) || (yaw_deg == NAN)) {
    pitch_deg = yaw_deg = NAN;
  }

  _generateCmdLong(tsystem, tcomponent,
      MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW,
      pitch_deg, yaw_deg, NAN, NAN, device_flags, 0, gimbal_id
      );
}

void MavlinkTelem::generateCmdDoGimbalManagerConfigure(uint8_t tsystem, uint8_t tcomponent,
    uint8_t gimbal_id, uint8_t primary_sysid, uint8_t primary_compid, uint8_t secondary_sysid, uint8_t secondary_compid)
{
  _generateCmdLong(tsystem, tcomponent,
      MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE,
      primary_sysid, primary_compid, secondary_sysid, secondary_compid, 0,0, gimbal_id
      );
}

// -- Mavsdk Convenience Task Wrapper --
// to make it easy for api_mavsdk to call functions

void MavlinkTelem::sendGimbalTargetingMode(uint8_t mode)
{
  gimbalmanagerOut.device_flags &=~ (GIMBAL_DEVICE_FLAGS_RETRACT | GIMBAL_DEVICE_FLAGS_NEUTRAL);
  if (mode == MAV_MOUNT_MODE_RETRACT) {
    gimbalmanagerOut.device_flags |= GIMBAL_DEVICE_FLAGS_RETRACT;
  }
  if (mode == MAV_MOUNT_MODE_NEUTRAL) {
    gimbalmanagerOut.device_flags |= GIMBAL_DEVICE_FLAGS_NEUTRAL;
  }

  _t_gimbal_mode = mode;
  SETTASK(TASK_GIMBAL, TASK_SENDCMD_DO_MOUNT_CONFIGURE);
}

// -- gimbal protocol v1

void MavlinkTelem::sendGimbalPitchYawDeg(float pitch, float yaw)
{
  // it uses _t_gimbal_mode !!
  _t_gimbal_pitch_deg = pitch;
  _t_gimbal_yaw_deg = yaw;
  SETTASK(TASK_GIMBAL, TASK_SENDCMD_DO_MOUNT_CONTROL);
}

// -- gimbal protocol v2

void MavlinkTelem::setGimbalLock(bool roll_lock, bool pitch_lock, bool yaw_lock)
{
  gimbalmanagerOut.device_flags &=~ (GIMBAL_DEVICE_FLAGS_ROLL_LOCK | GIMBAL_DEVICE_FLAGS_PITCH_LOCK | GIMBAL_DEVICE_FLAGS_YAW_LOCK);
  if (roll_lock) {
    gimbalmanagerOut.device_flags |= GIMBAL_DEVICE_FLAGS_ROLL_LOCK;
  }
  if (pitch_lock) {
    gimbalmanagerOut.device_flags |= GIMBAL_DEVICE_FLAGS_PITCH_LOCK;
  }
  if (yaw_lock) {
    gimbalmanagerOut.device_flags |= GIMBAL_DEVICE_FLAGS_YAW_LOCK;
  }
}

void MavlinkTelem::setGimbalRcControl(uint8_t flag)
{
  gimbalmanagerOut.device_flags &=~ (GIMBAL_DEVICE_FLAGS_RC_EXCLUSIVE | GIMBAL_DEVICE_FLAGS_RC_MIXED);
  if (flag == 1) {
      gimbalmanagerOut.device_flags |= GIMBAL_DEVICE_FLAGS_RC_MIXED;
  }
  if (flag == 2) {
      gimbalmanagerOut.device_flags |= GIMBAL_DEVICE_FLAGS_RC_EXCLUSIVE;
  }
}

void MavlinkTelem::sendGimbalManagerPitchYawDeg(float pitch, float yaw)
{
  if (!gimbalmanager.compid) return; // no gimbal manager

  _t_GM_gdflags = gimbalmanagerOut.device_flags;
  _t_GM_gdflags |= GIMBAL_DEVICE_FLAGS_YAW_IN_VEHICLE_FRAME;
  _t_GM_gdflags &=~  GIMBAL_MANAGER_FLAGS_YAW_IN_EARTH_FRAME | GIMBAL_MANAGER_FLAGS_ACCEPTS_YAW_IN_EARTH_FRAME;
  _t_GM_pitch_deg = pitch;
  _t_GM_yaw_deg = yaw;
  SETTASK(TASK_GIMBAL, TASK_SENDMSG_GIMBAL_MANAGER_SET_PITCHYAW);
}

void MavlinkTelem::sendGimbalManagerSetAttitudePitchYawDeg(float pitch, float yaw)
{
  if (!gimbalmanager.compid) return; // no gimbal manager

  _t_GMatt_gdflags = gimbalmanagerOut.device_flags;
  _t_GMatt_gdflags |= GIMBAL_DEVICE_FLAGS_YAW_IN_VEHICLE_FRAME;
  _t_GMatt_gdflags &=~  GIMBAL_MANAGER_FLAGS_YAW_IN_EARTH_FRAME | GIMBAL_MANAGER_FLAGS_ACCEPTS_YAW_IN_EARTH_FRAME;
  _t_GMatt_pitch_deg = pitch;
  _t_GMatt_yaw_deg = yaw;
  SETTASK(TASK_GIMBAL, TASK_SENDMSG_GIMBAL_MANAGER_SET_ATTITUDE);
}

void MavlinkTelem::sendGimbalManagerCmdPitchYawDeg(float pitch, float yaw)
{
  if (!gimbalmanager.compid) return; // no gimbal manager

  _t_GMcmd_gdflags = gimbalmanagerOut.device_flags;
  _t_GMcmd_gdflags |= GIMBAL_DEVICE_FLAGS_YAW_IN_VEHICLE_FRAME;
  _t_GMcmd_gdflags &=~  GIMBAL_MANAGER_FLAGS_YAW_IN_EARTH_FRAME | GIMBAL_MANAGER_FLAGS_ACCEPTS_YAW_IN_EARTH_FRAME;
  _t_GMcmd_pitch_deg = pitch;
  _t_GMcmd_yaw_deg = yaw;
  SETTASK(TASK_GIMBAL, TASK_SENDCMD_DO_GIMBAL_MANAGER_PITCHYAW);
}

// -- Task handlers --

bool MavlinkTelem::doTaskGimbalAndGimbalClient(void)
{
  if (!_task[TASK_GIMBAL]) return false; // no task pending

  // if there is no gimbal, then there is also no gimbal manager which needs to be served
  // so first if()-checking for gimbal, and then for gimbalmanager is ok

  if (!gimbal.compid) { // no gimbal
    _task[TASK_GIMBAL] = 0; 
    return false; 
  }

  if (_task[TASK_GIMBAL] & TASK_SENDCMD_DO_MOUNT_CONFIGURE) {
    RESETTASK(TASK_GIMBAL, TASK_SENDCMD_DO_MOUNT_CONFIGURE);
    generateCmdDoMountConfigure(_sysid, autopilot.compid, _t_gimbal_mode);
    return true; //do only one per loop
  }
  if (_task[TASK_GIMBAL] & TASK_SENDCMD_DO_MOUNT_CONTROL) {
    RESETTASK(TASK_GIMBAL, TASK_SENDCMD_DO_MOUNT_CONTROL);
    generateCmdDoMountControl(_sysid, autopilot.compid, _t_gimbal_pitch_deg, _t_gimbal_yaw_deg, _t_gimbal_mode);
    return true; //do only one per loop
  }

  if (_task[TASK_GIMBAL] & TASK_SENDREQUEST_GIMBAL_DEVICE_INFORMATION) {
    RESETTASK(TASK_GIMBAL, TASK_SENDREQUEST_GIMBAL_DEVICE_INFORMATION);
    generateCmdRequestGimbalDeviceInformation(_sysid, gimbal.compid);
    return true; //do only one per loop
  }

  if (!gimbalmanager.compid) { // no gimbal manager
    RESETTASK(TASK_GIMBAL, TASK_SENDMSG_GIMBAL_MANAGER_SET_PITCHYAW | TASK_SENDMSG_GIMBAL_MANAGER_SET_ATTITUDE |
                           TASK_SENDCMD_DO_GIMBAL_MANAGER_PITCHYAW | TASK_SENDCMD_DO_GIMBAL_MANAGER_CONFIGURE |
                           TASK_SENDREQUEST_GIMBAL_MANAGER_INFORMATION);
	return false;
  }

  if (_task[TASK_GIMBAL] & TASK_SENDMSG_GIMBAL_MANAGER_SET_PITCHYAW) {
    RESETTASK(TASK_GIMBAL, TASK_SENDMSG_GIMBAL_MANAGER_SET_PITCHYAW);
    generateGimbalManagerSetPitchYaw(_sysid, gimbalmanager.compid, gimbal.compid,
                                     _t_GM_pitch_deg, _t_GM_yaw_deg, _t_GM_gdflags);
    return true; //do only one per loop
  }
  if (_task[TASK_GIMBAL] & TASK_SENDMSG_GIMBAL_MANAGER_SET_ATTITUDE) {
    RESETTASK(TASK_GIMBAL, TASK_SENDMSG_GIMBAL_MANAGER_SET_ATTITUDE);
    generateGimbalManagerSetAttitude(_sysid, gimbalmanager.compid, gimbal.compid,
                                     _t_GMatt_pitch_deg, _t_GMatt_yaw_deg, _t_GMatt_gdflags);
    return true; //do only one per loop
  }
  if (_task[TASK_GIMBAL] & TASK_SENDCMD_DO_GIMBAL_MANAGER_PITCHYAW) {
    RESETTASK(TASK_GIMBAL, TASK_SENDCMD_DO_GIMBAL_MANAGER_PITCHYAW);
    generateCmdDoGimbalManagerPitchYaw(_sysid, gimbalmanager.compid, gimbal.compid,
                                       _t_GMcmd_pitch_deg, _t_GMcmd_yaw_deg, _t_GMcmd_gdflags);
    return true; //do only one per loop
  }

  //TODO: handle TASK_SENDCMD_DO_GIMBAL_MANAGER_CONFIGURE

  if (_task[TASK_GIMBAL] & TASK_SENDREQUEST_GIMBAL_MANAGER_INFORMATION) {
    RESETTASK(TASK_GIMBAL, TASK_SENDREQUEST_GIMBAL_MANAGER_INFORMATION);
    generateCmdRequestGimbalManagerInformation(_sysid, gimbalmanager.compid);
    return true; //do only one per loop
  }

  return false;
}

// -- Handle incoming MAVLink messages, which are from the Gimbal Device to the Gimbal Client --

void MavlinkTelem::handleMessageGimbal(void)
{
  gimbal.is_receiving = MAVLINK_TELEM_RECEIVING_TIMEOUT; //we accept any msg from the gimbal to indicate it is alive

  switch (_msg.msgid) {
    case FASTMAVLINK_MSG_ID_HEARTBEAT: {
      fmav_heartbeat_t payload;
      fmav_msg_heartbeat_decode(&payload, &_msg);
      gimbal.system_status = payload.system_status;
      gimbal.custom_mode = payload.custom_mode;
      gimbal.is_armed = (payload.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) ? true : false;
      gimbal.is_standby = (payload.system_status <= MAV_STATE_STANDBY) ? true : false;
      gimbal.is_critical = (payload.system_status >= MAV_STATE_CRITICAL) ? true : false;
      gimbal.prearm_ok = (payload.custom_mode & 0x80000000) ? false : true;
      INCU8(gimbal.updated);
      //gimbal.is_receiving = MAVLINK_TELEM_RECEIVING_TIMEOUT;
      break;
    }
/*
    case FASTMAVLINK_MSG_ID_MOUNT_STATUS: {
      fmav_mount_status_t payload;
      fmav_msg_mount_status_decode(&payload, &_msg);
      gimbalAtt.roll_deg = ((float)payload.pointing_b * 0.01f);
      gimbalAtt.pitch_deg = ((float)payload.pointing_a * 0.01f);
      gimbalAtt.yaw_deg_vehicle_frame = ((float)payload.pointing_c * 0.01f);
      if (gimbalAtt.yaw_deg_earth_frame > 180.0f) gimbalAtt.yaw_deg_earth_frame -= 360.0f;
      gimbalAtt.yaw_deg_earth_frame = gimbalAtt.yaw_deg_earth_frame + att.yaw_rad * FRADTODEG;
      if (gimbalAtt.yaw_deg_earth_frame < -180.0f) gimbalAtt.yaw_deg_earth_frame += 360.0f;
      INCU8(gimbalAtt.updated);
      break;
    }
*/
    case FASTMAVLINK_MSG_ID_GIMBAL_DEVICE_INFORMATION: {
      fmav_gimbal_device_information_t payload;
      fmav_msg_gimbal_device_information_decode(&payload, &_msg);
      memset(gimbaldeviceInfo.vendor_name, 0, 32+1);
      memcpy(gimbaldeviceInfo.vendor_name, payload.vendor_name, 32);
      memset(gimbaldeviceInfo.model_name, 0, 32+1);
      memcpy(gimbaldeviceInfo.model_name, payload.model_name, 32);
      memset(gimbaldeviceInfo.custom_name, 0, 32+1);
      memcpy(gimbaldeviceInfo.custom_name, payload.custom_name, 32);
      gimbaldeviceInfo.firmware_version = payload.firmware_version;
      gimbaldeviceInfo.hardware_version = payload.hardware_version;
      gimbaldeviceInfo.cap_flags = payload.cap_flags;
      gimbaldeviceInfo.custom_cap_flags = payload.custom_cap_flags;
      INCU8(gimbaldeviceInfo.updated);
      clear_request(TASK_GIMBAL, TASK_SENDREQUEST_GIMBAL_DEVICE_INFORMATION);
      gimbal.requests_waiting_mask &=~ GIMBAL_REQUESTWAITING_GIMBAL_DEVICE_INFORMATION;
      gimbalmanager.requests_waiting_mask &=~ GIMBAL_REQUESTWAITING_GIMBAL_DEVICE_INFORMATION;
      // we do not copy to the gimbal manager capability flags, we rely on the gimbal manager to provide the correct ones
      break;
    }

    case FASTMAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS: {
      fmav_gimbal_device_attitude_status_t payload;
      fmav_msg_gimbal_device_attitude_status_decode(&payload, &_msg);
      // TODO: we assume yaw is in vehicle frame !!!!
      calc_G_angles_deg_from_q(&gimbalAtt.roll_deg, &gimbalAtt.pitch_deg, &gimbalAtt.yaw_deg_vehicle_frame, payload.q);
      gimbalAtt.yaw_deg_earth_frame = gimbalAtt.yaw_deg_vehicle_frame + att.yaw_rad * FRADTODEG;
      if (gimbalAtt.yaw_deg_earth_frame < -180.0f) gimbalAtt.yaw_deg_earth_frame += 360.0f;
      INCU8(gimbalAtt.updated);
      // update gimbal manager flags, do it always, even if no gimbal manager has been discovered
      gimbalmanagerStatus.device_flags = payload.flags;
      break;
    }
  }
}

// -- Handle incoming MAVLink messages, which are from the Gimbal Manager to the Gimbal Client --

void MavlinkTelem::handleMessageGimbalManager(void)
{
  //gimbalmanager.is_receiving = MAVLINK_TELEM_RECEIVING_TIMEOUT; //we accept any msg from the manager to indicate it is alive
  //no: we should not do this for the manager, since this function is called for any message from the hosting component

  switch (_msg.msgid) {
    case FASTMAVLINK_MSG_ID_GIMBAL_MANAGER_INFORMATION: {
      fmav_gimbal_manager_information_t payload;
      fmav_msg_gimbal_manager_information_decode(&payload, &_msg);
      if (payload.gimbal_device_id != gimbal.compid) break; // not for us
      gimbalmanagerInfo.device_cap_flags = (uint16_t)(payload.cap_flags);
      gimbalmanagerInfo.manager_cap_flags = (uint16_t)(payload.cap_flags > 16);
      INCU8(gimbalmanagerInfo.updated);
      clear_request(TASK_GIMBAL, TASK_SENDREQUEST_GIMBAL_MANAGER_INFORMATION);
      gimbalmanager.requests_waiting_mask &=~ GIMBAL_REQUESTWAITING_GIMBAL_MANAGER_INFORMATION;
      break;
    }

    case FASTMAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS: {
      fmav_gimbal_manager_status_t payload;
      fmav_msg_gimbal_manager_status_decode(&payload, &_msg);
      if (payload.gimbal_device_id != gimbal.compid) break; // not for us
      gimbalmanagerStatus.device_flags = (uint16_t)(payload.flags);
      gimbalmanagerStatus.primary_sysid = payload.primary_control_sysid;
      gimbalmanagerStatus.primary_compid = payload.primary_control_compid;
      gimbalmanagerStatus.secondary_sysid = payload.secondary_control_sysid;
      gimbalmanagerStatus.secondary_compid = payload.secondary_control_compid;
      INCU8(gimbalmanagerStatus.updated);
      gimbalmanager.is_receiving = MAVLINK_TELEM_GIMBALMANAGER_RECEIVING_TIMEOUT;
      break;
    }
  }
}

// -- Startup Requests --

void MavlinkTelem::setGimbalStartupRequests(void)
{
  set_request(TASK_GIMBAL, TASK_SENDREQUEST_GIMBAL_DEVICE_INFORMATION, 10, 200-12);
}

void MavlinkTelem::setGimbalManagerStartupRequests(void)
{
  set_request(TASK_GIMBAL, TASK_SENDREQUEST_GIMBAL_DEVICE_INFORMATION, 10, 200-12);
  set_request(TASK_GIMBAL, TASK_SENDREQUEST_GIMBAL_MANAGER_INFORMATION, 10, 200-18);
}

// -- Resets --

void MavlinkTelem::_resetGimbalAndGimbalClient(void)
{
  _task[TASK_GIMBAL] = 0;

  gimbal.compid = 0;
  gimbal.is_receiving = 0;

  gimbal.requests_triggered = 0;
  gimbal.requests_waiting_mask = GIMBAL_REQUESTWAITING_ALL;
  gimbal.is_initialized = false;

  gimbal.system_status = MAV_STATE_UNINIT;
  gimbal.custom_mode = 0;
  gimbal.is_armed = false;
  gimbal.is_standby = true;
  gimbal.is_critical = false;
  gimbal.prearm_ok = false;
  gimbal.updated = 0;

  gimbalAtt.roll_deg = 0.0f;
  gimbalAtt.pitch_deg = 0.0f;
  gimbalAtt.yaw_deg_vehicle_frame = 0.0f;
  gimbalAtt.yaw_deg_earth_frame = 0.0f;
  gimbalAtt.updated = 0;

  _resetGimbalClient();
}

void MavlinkTelem::_resetGimbalClient(void)
{
  //_task[TASK_GIMBAL] = 0; // only reset gimbal client tasks, but not very important

  gimbalmanager.compid = 0;
  gimbalmanager.is_receiving = 0;

  gimbalmanager.requests_triggered = 0;
  gimbalmanager.requests_waiting_mask = GIMBALCLIENT_REQUESTWAITING_ALL;
  gimbalmanager.is_initialized = false;

  gimbalmanager.updated = 0;

  gimbaldeviceInfo.vendor_name[0] = 0;
  gimbaldeviceInfo.model_name[0] = 0;
  gimbaldeviceInfo.custom_name[0] = 0;
  gimbaldeviceInfo.firmware_version = 0;
  gimbaldeviceInfo.hardware_version = 0;
  gimbaldeviceInfo.cap_flags = 0;
  gimbaldeviceInfo.custom_cap_flags = 0;
  gimbaldeviceInfo.updated = 0;

  gimbalmanagerInfo.device_cap_flags = 0;
  gimbalmanagerInfo.manager_cap_flags = 0;
  gimbalmanagerInfo.updated = 0;

  gimbalmanagerStatus.device_flags = 0;
  gimbalmanagerStatus.primary_sysid = 0;
  gimbalmanagerStatus.primary_compid = 0;
  gimbalmanagerStatus.secondary_sysid = 0;
  gimbalmanagerStatus.secondary_compid = 0;
  gimbalmanagerStatus.updated = 0;

  gimbalmanagerOut.device_flags = GIMBAL_DEVICE_FLAGS_ROLL_LOCK | GIMBAL_DEVICE_FLAGS_PITCH_LOCK;
}
