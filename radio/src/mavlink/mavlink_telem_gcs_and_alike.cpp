//*******************************************************
// MavTelem Library
// Copyright (c) OlliW, OlliW42, www.olliw.eu
// LGPL3
// https://www.gnu.org/licenses/lgpl-3.0.en.html
//*******************************************************

#include "opentx.h"

// -- Handle incoming MAVLink messages, from a GCs or alike --

void MavlinkTelem::handleMessageGcsAndAlike(void)
{
  switch (_msg.msgid) {
    // AUTOPILOT_VERSION_REQUEST is kind of required to make MissionPlanner happy
    case FASTMAVLINK_MSG_ID_AUTOPILOT_VERSION_REQUEST: {
      SETTASK(TASK_ME, TASK_ME_SENDMYAUTOPILOTVERSION);
      break;
    }
    // MissonPlanner wants us to have at least one parameter
    case FASTMAVLINK_MSG_ID_PARAM_REQUEST_READ: {
      fmav_param_request_read_t payload;
      fmav_msg_param_request_read_decode(&payload, &_msg);
      uint16_t index;
      if (fmav_param_do_param_request_read(&index, &payload)) {
        _mavlink_copy_g2p(); // sync parameter copies
        _pv_index = index;
        SETTASK(TASK_ME, TASK_ME_SENDMYPARAMVALUE);
      }
      break;
    }
    case FASTMAVLINK_MSG_ID_PARAM_REQUEST_LIST: {
      _mavlink_copy_g2p(); // sync parameter copies
      _prl_index = 0; // start with first parameter
      _prl_tlast = 0; // makes it to send first parameter ASAP
      SETTASK(TASK_ME, TASK_ME_SENDMYPARAMLIST);
      break;
    }
    case FASTMAVLINK_MSG_ID_PARAM_SET: {
      fmav_param_set_t payload;
      fmav_msg_param_set_decode(&payload, &_msg);
      uint16_t index;
      if (fmav_param_do_param_set(&index, &payload)) {
        fmav_param_set_value(index, payload.param_value);
        _mavlink_copy_p2g(); // sync global parameters & store
        storageDirty(EE_MODEL);
        _pv_index = index;
        SETTASK(TASK_ME, TASK_ME_SENDMYPARAMVALUE);
      }
      break;
    }
    case FASTMAVLINK_MSG_ID_COMMAND_LONG: {
      fmav_command_long_t payload;
      fmav_msg_command_long_decode(&payload, &_msg);
      if (payload.command == MAV_CMD_DO_SEND_BANNER) SETTASK(TASK_ME, TASK_ME_SENDMYBANNER);
      break;
    }
  }
}

// -- Startup Requests --
/*
void MavlinkTelem::setGcsAndAlikeStartupRequests(void)
{
}
*/

// -- Resets --
/*
void MavlinkTelem::_resetGcsAndAlike(void)
{
}
*/
