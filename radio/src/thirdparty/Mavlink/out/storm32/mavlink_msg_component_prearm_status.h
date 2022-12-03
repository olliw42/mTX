//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_COMPONENT_PREARM_STATUS_H
#define FASTMAVLINK_MSG_COMPONENT_PREARM_STATUS_H


//----------------------------------------
//-- Message COMPONENT_PREARM_STATUS
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_component_prearm_status_t {
    uint32_t enabled_flags;
    uint32_t fail_flags;
    uint8_t target_system;
    uint8_t target_component;
}) fmav_component_prearm_status_t;


#define FASTMAVLINK_MSG_ID_COMPONENT_PREARM_STATUS  60025

#define FASTMAVLINK_MSG_COMPONENT_PREARM_STATUS_PAYLOAD_LEN_MAX  10
#define FASTMAVLINK_MSG_COMPONENT_PREARM_STATUS_CRCEXTRA  20

#define FASTMAVLINK_MSG_COMPONENT_PREARM_STATUS_FLAGS  3
#define FASTMAVLINK_MSG_COMPONENT_PREARM_STATUS_TARGET_SYSTEM_OFS  8
#define FASTMAVLINK_MSG_COMPONENT_PREARM_STATUS_TARGET_COMPONENT_OFS  9

#define FASTMAVLINK_MSG_COMPONENT_PREARM_STATUS_FRAME_LEN_MAX  35



#define FASTMAVLINK_MSG_COMPONENT_PREARM_STATUS_FIELD_ENABLED_FLAGS_OFS  0
#define FASTMAVLINK_MSG_COMPONENT_PREARM_STATUS_FIELD_FAIL_FLAGS_OFS  4
#define FASTMAVLINK_MSG_COMPONENT_PREARM_STATUS_FIELD_TARGET_SYSTEM_OFS  8
#define FASTMAVLINK_MSG_COMPONENT_PREARM_STATUS_FIELD_TARGET_COMPONENT_OFS  9


//----------------------------------------
//-- Message COMPONENT_PREARM_STATUS pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_component_prearm_status_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint32_t enabled_flags, uint32_t fail_flags,
    fmav_status_t* _status)
{
    fmav_component_prearm_status_t* _payload = (fmav_component_prearm_status_t*)_msg->payload;

    _payload->enabled_flags = enabled_flags;
    _payload->fail_flags = fail_flags;
    _payload->target_system = target_system;
    _payload->target_component = target_component;


    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_COMPONENT_PREARM_STATUS;
    _msg->target_sysid = target_system;
    _msg->target_compid = target_component;
    _msg->crc_extra = FASTMAVLINK_MSG_COMPONENT_PREARM_STATUS_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_COMPONENT_PREARM_STATUS_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_component_prearm_status_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_component_prearm_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_component_prearm_status_pack(
        _msg, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->enabled_flags, _payload->fail_flags,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_component_prearm_status_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint32_t enabled_flags, uint32_t fail_flags,
    fmav_status_t* _status)
{
    fmav_component_prearm_status_t* _payload = (fmav_component_prearm_status_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->enabled_flags = enabled_flags;
    _payload->fail_flags = fail_flags;
    _payload->target_system = target_system;
    _payload->target_component = target_component;


    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_COMPONENT_PREARM_STATUS;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_COMPONENT_PREARM_STATUS >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_COMPONENT_PREARM_STATUS >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_COMPONENT_PREARM_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_COMPONENT_PREARM_STATUS_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_component_prearm_status_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_component_prearm_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_component_prearm_status_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->enabled_flags, _payload->fail_flags,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_component_prearm_status_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint32_t enabled_flags, uint32_t fail_flags,
    fmav_status_t* _status)
{
    fmav_component_prearm_status_t _payload;

    _payload.enabled_flags = enabled_flags;
    _payload.fail_flags = fail_flags;
    _payload.target_system = target_system;
    _payload.target_component = target_component;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_COMPONENT_PREARM_STATUS,
        FASTMAVLINK_MSG_COMPONENT_PREARM_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_COMPONENT_PREARM_STATUS_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_component_prearm_status_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_component_prearm_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_COMPONENT_PREARM_STATUS,
        FASTMAVLINK_MSG_COMPONENT_PREARM_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_COMPONENT_PREARM_STATUS_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message COMPONENT_PREARM_STATUS decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zerofill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_component_prearm_status_decode(fmav_component_prearm_status_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_COMPONENT_PREARM_STATUS_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_COMPONENT_PREARM_STATUS_PAYLOAD_LEN_MAX - msg->len);
    } else {
		// note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_COMPONENT_PREARM_STATUS_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_COMPONENT_PREARM_STATUS_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_component_prearm_status_get_field_enabled_flags(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_component_prearm_status_get_field_fail_flags(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[4]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_component_prearm_status_get_field_target_system(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[8]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_component_prearm_status_get_field_target_component(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[9]), sizeof(uint8_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_COMPONENT_PREARM_STATUS  60025

#define mavlink_component_prearm_status_t  fmav_component_prearm_status_t

#define MAVLINK_MSG_ID_COMPONENT_PREARM_STATUS_LEN  10
#define MAVLINK_MSG_ID_COMPONENT_PREARM_STATUS_MIN_LEN  10
#define MAVLINK_MSG_ID_60025_LEN  10
#define MAVLINK_MSG_ID_60025_MIN_LEN  10

#define MAVLINK_MSG_ID_COMPONENT_PREARM_STATUS_CRC  20
#define MAVLINK_MSG_ID_60025_CRC  20




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_component_prearm_status_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint8_t target_system, uint8_t target_component, uint32_t enabled_flags, uint32_t fail_flags)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_component_prearm_status_pack(
        _msg, sysid, compid,
        target_system, target_component, enabled_flags, fail_flags,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_component_prearm_status_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint32_t enabled_flags, uint32_t fail_flags)
{
    return fmav_msg_component_prearm_status_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        target_system, target_component, enabled_flags, fail_flags,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_component_prearm_status_decode(const mavlink_message_t* msg, mavlink_component_prearm_status_t* payload)
{
    fmav_msg_component_prearm_status_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_COMPONENT_PREARM_STATUS_H
