//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MLRS_H
#define FASTMAVLINK_MLRS_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef FASTMAVLINK_BUILD_DATE
#define FASTMAVLINK_BUILD_DATE  "Fri Dec 02 2022"
#endif

#ifndef FASTMAVLINK_DIALECT_VERSION
#define FASTMAVLINK_DIALECT_VERSION  0  // this is the version specified in the dialect xml file
#endif


//------------------------------
//-- Message credentials
//-- The values of msg_entry_t for all messages in the dialect.
//-- msgid, extra crc, max length, flag, target sysid offset, target compid offset
//------------------------------

#include "mlrs_msg_entries.h"

#ifndef FASTMAVLINK_MESSAGE_CRCS
#define FASTMAVLINK_MESSAGE_CRCS  FASTMAVLINK_MSG_ENTRIES
#endif


//------------------------------
//-- FastMavlink lib
//------------------------------

#include "../lib/fastmavlink.h"

#ifdef FASTMAVLINK_PYMAVLINK_ENABLED
#include "../lib/fastmavlink_pymavlink.h"
#endif


//------------------------------
//-- Enum definitons
//------------------------------

#ifndef FASTMAVLINK_TEST_EXCLUDE_ENUMS

#ifndef FASTMAVLINK_HAS_ENUM_RADIO_RC_CHANNELS_FLAGS
#define FASTMAVLINK_HAS_ENUM_RADIO_RC_CHANNELS_FLAGS
typedef enum RADIO_RC_CHANNELS_FLAGS {
    RADIO_RC_CHANNELS_FLAGS_FAILSAFE = 1,  // Failsafe is active. 
    RADIO_RC_CHANNELS_FLAGS_FRAME_MISSED = 2,  // Indicates that the current frame has not been received. Channel values are frozen. 
    RADIO_RC_CHANNELS_FLAGS_ENUM_END = 3,  // end marker
} RADIO_RC_CHANNELS_FLAGS;
#endif


#ifndef FASTMAVLINK_HAS_ENUM_RADIO_LINK_STATS_FLAGS
#define FASTMAVLINK_HAS_ENUM_RADIO_LINK_STATS_FLAGS
typedef enum RADIO_LINK_STATS_FLAGS {
    RADIO_LINK_STATS_FLAGS_RSSI_DBM = 1,  // Rssi are in negative dBm. Values 0..254 corresponds to 0..-254 dBm. 
    RADIO_LINK_STATS_FLAGS_ENUM_END = 2,  // end marker
} RADIO_LINK_STATS_FLAGS;
#endif


#ifndef FASTMAVLINK_HAS_ENUM_RADIO_TYPE
#define FASTMAVLINK_HAS_ENUM_RADIO_TYPE
typedef enum RADIO_TYPE {
    RADIO_TYPE_GENERIC = 0,  // Unknwon radio link type. 
    RADIO_TYPE_MLRS = 1,  // Radio link is mLRS. 
    RADIO_TYPE_ENUM_END = 2,  // end marker
} RADIO_TYPE;
#endif

#endif // FASTMAVLINK_DO_NOT_INCLUDE_ENUMS


//------------------------------
//-- Message definitions
//------------------------------

#ifdef FASTMAVLINK_IGNORE_WADDRESSOFPACKEDMEMBER
  #if defined __GNUC__ && __GNUC__ >= 9
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Waddress-of-packed-member"
  #endif
#endif

#include "./mavlink_msg_radio_rc_channels.h"
#include "./mavlink_msg_radio_link_stats.h"
#include "./mavlink_msg_radio_link_flow_control.h"
#include "./mavlink_msg_radio_link_information.h"
#include "./mavlink_msg_frsky_passthrough_array.h"
#include "./mavlink_msg_param_value_array.h"

#ifdef FASTMAVLINK_IGNORE_WADDRESSOFPACKEDMEMBER
  #if defined __GNUC__ && __GNUC__ >= 9
    #pragma GCC diagnostic pop
  #endif
#endif


//------------------------------
//-- Dialect includes
//------------------------------




#ifdef __cplusplus
}
#endif

#endif // FASTMAVLINK_MLRS_H
