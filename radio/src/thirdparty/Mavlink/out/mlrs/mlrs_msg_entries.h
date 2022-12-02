//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_ENTRIES_H
#define FASTMAVLINK_MSG_ENTRIES_H


//------------------------------
//-- Message credentials
//-- The values of msg_entry_t for all messages in the dialect.
//-- msgid, extra crc, max length, flag, target sysid offset, target compid offset
//------------------------------

#define FASTMAVLINK_MSG_ENTRY_RADIO_RC_CHANNELS  {95, 89, 50, 0, 0, 0}
#define FASTMAVLINK_MSG_ENTRY_RADIO_LINK_STATS  {96, 238, 15, 0, 0, 0}
#define FASTMAVLINK_MSG_ENTRY_RADIO_LINK_FLOW_CONTROL  {97, 82, 7, 0, 0, 0}
#define FASTMAVLINK_MSG_ENTRY_RADIO_LINK_INFORMATION  {98, 188, 6, 0, 0, 0}
#define FASTMAVLINK_MSG_ENTRY_FRSKY_PASSTHROUGH_ARRAY  {60040, 156, 245, 0, 0, 0}
#define FASTMAVLINK_MSG_ENTRY_PARAM_VALUE_ARRAY  {60041, 76, 255, 0, 0, 0}


/*------------------------------
 * If only relatively few MAVLink messages are used, efficiency can
 * be much improved, both memory and computational time wise, by
 * limiting the known message entries to only those which are used.
 *
 * This can be achieved by commenting out in the below define of
 * FASTMAVLINK_MSG_ENTRIES all those message entries which are not used.
 *
 * Alternatively, one can define one's own FASTMAVLINK_MESSAGE_CRCS
 * using the above defines for each message entry. It is then MOST
 * important to keep the sequence in order since otherwise the default
 * binary search will fail. For instance:
 *
 * #include "pathtofastmavlink/thedialect/fmav_msg_entries.h"
 * #define FASTMAVLINK_MESSAGE_CRCS {\
 *     FASTMAVLINK_MSG_ENTRY_PARAM_REQUEST_READ,\
 *     FASTMAVLINK_MSG_ENTRY_PARAM_REQUEST_LIST,\
 *     FASTMAVLINK_MSG_ENTRY_PARAM_SET,\
 *     FASTMAVLINK_MSG_ENTRY_COMMAND_LONG,\
 *     FASTMAVLINK_MSG_ENTRY_AUTOPILOT_VERSION_REQUEST }
 ------------------------------*/

#define FASTMAVLINK_MSG_ENTRIES {\
  FASTMAVLINK_MSG_ENTRY_RADIO_RC_CHANNELS,\
  FASTMAVLINK_MSG_ENTRY_RADIO_LINK_STATS,\
  FASTMAVLINK_MSG_ENTRY_RADIO_LINK_FLOW_CONTROL,\
  FASTMAVLINK_MSG_ENTRY_RADIO_LINK_INFORMATION,\
  FASTMAVLINK_MSG_ENTRY_FRSKY_PASSTHROUGH_ARRAY,\
  FASTMAVLINK_MSG_ENTRY_PARAM_VALUE_ARRAY\
}


#endif // FASTMAVLINK_MSG_ENTRIES_H
