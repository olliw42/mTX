//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_RADIO_RC_CHANNELS_DEV_H
#define FASTMAVLINK_MSG_RADIO_RC_CHANNELS_DEV_H


//----------------------------------------
//-- Message RADIO_RC_CHANNELS_DEV
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_radio_rc_channels_dev_t {
    uint16_t flags;
    uint8_t target_system;
    uint8_t target_component;
    uint8_t count;
    int16_t channels[32];
}) fmav_radio_rc_channels_dev_t;


#define FASTMAVLINK_MSG_ID_RADIO_RC_CHANNELS_DEV  420

#define FASTMAVLINK_MSG_RADIO_RC_CHANNELS_DEV_PAYLOAD_LEN_MAX  69
#define FASTMAVLINK_MSG_RADIO_RC_CHANNELS_DEV_CRCEXTRA  174

#define FASTMAVLINK_MSG_RADIO_RC_CHANNELS_DEV_FLAGS  3
#define FASTMAVLINK_MSG_RADIO_RC_CHANNELS_DEV_TARGET_SYSTEM_OFS  2
#define FASTMAVLINK_MSG_RADIO_RC_CHANNELS_DEV_TARGET_COMPONENT_OFS  3

#define FASTMAVLINK_MSG_RADIO_RC_CHANNELS_DEV_FRAME_LEN_MAX  94

#define FASTMAVLINK_MSG_RADIO_RC_CHANNELS_DEV_FIELD_CHANNELS_NUM  32 // number of elements in array
#define FASTMAVLINK_MSG_RADIO_RC_CHANNELS_DEV_FIELD_CHANNELS_LEN  64 // length of array = number of bytes

#define FASTMAVLINK_MSG_RADIO_RC_CHANNELS_DEV_FIELD_FLAGS_OFS  0
#define FASTMAVLINK_MSG_RADIO_RC_CHANNELS_DEV_FIELD_TARGET_SYSTEM_OFS  2
#define FASTMAVLINK_MSG_RADIO_RC_CHANNELS_DEV_FIELD_TARGET_COMPONENT_OFS  3
#define FASTMAVLINK_MSG_RADIO_RC_CHANNELS_DEV_FIELD_COUNT_OFS  4
#define FASTMAVLINK_MSG_RADIO_RC_CHANNELS_DEV_FIELD_CHANNELS_OFS  5


//----------------------------------------
//-- Message RADIO_RC_CHANNELS_DEV pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_radio_rc_channels_dev_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint8_t count, uint16_t flags, const int16_t* channels,
    fmav_status_t* _status)
{
    fmav_radio_rc_channels_dev_t* _payload = (fmav_radio_rc_channels_dev_t*)_msg->payload;

    _payload->flags = flags;
    _payload->target_system = target_system;
    _payload->target_component = target_component;
    _payload->count = count;
    memcpy(&(_payload->channels), channels, sizeof(int16_t)*32);

    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_RADIO_RC_CHANNELS_DEV;
    _msg->target_sysid = target_system;
    _msg->target_compid = target_component;
    _msg->crc_extra = FASTMAVLINK_MSG_RADIO_RC_CHANNELS_DEV_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_RADIO_RC_CHANNELS_DEV_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_radio_rc_channels_dev_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_radio_rc_channels_dev_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_radio_rc_channels_dev_pack(
        _msg, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->count, _payload->flags, _payload->channels,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_radio_rc_channels_dev_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint8_t count, uint16_t flags, const int16_t* channels,
    fmav_status_t* _status)
{
    fmav_radio_rc_channels_dev_t* _payload = (fmav_radio_rc_channels_dev_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->flags = flags;
    _payload->target_system = target_system;
    _payload->target_component = target_component;
    _payload->count = count;
    memcpy(&(_payload->channels), channels, sizeof(int16_t)*32);

    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_RADIO_RC_CHANNELS_DEV;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_RADIO_RC_CHANNELS_DEV >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_RADIO_RC_CHANNELS_DEV >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_RADIO_RC_CHANNELS_DEV_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_RADIO_RC_CHANNELS_DEV_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_radio_rc_channels_dev_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_radio_rc_channels_dev_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_radio_rc_channels_dev_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->count, _payload->flags, _payload->channels,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_radio_rc_channels_dev_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint8_t count, uint16_t flags, const int16_t* channels,
    fmav_status_t* _status)
{
    fmav_radio_rc_channels_dev_t _payload;

    _payload.flags = flags;
    _payload.target_system = target_system;
    _payload.target_component = target_component;
    _payload.count = count;
    memcpy(&(_payload.channels), channels, sizeof(int16_t)*32);

    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_RADIO_RC_CHANNELS_DEV,
        FASTMAVLINK_MSG_RADIO_RC_CHANNELS_DEV_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_RADIO_RC_CHANNELS_DEV_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_radio_rc_channels_dev_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_radio_rc_channels_dev_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_RADIO_RC_CHANNELS_DEV,
        FASTMAVLINK_MSG_RADIO_RC_CHANNELS_DEV_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_RADIO_RC_CHANNELS_DEV_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message RADIO_RC_CHANNELS_DEV decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_radio_rc_channels_dev_decode(fmav_radio_rc_channels_dev_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_RADIO_RC_CHANNELS_DEV_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_RADIO_RC_CHANNELS_DEV_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_RADIO_RC_CHANNELS_DEV_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_RADIO_RC_CHANNELS_DEV_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_radio_rc_channels_dev_get_field_flags(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_radio_rc_channels_dev_get_field_target_system(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[2]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_radio_rc_channels_dev_get_field_target_component(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[3]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_radio_rc_channels_dev_get_field_count(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[4]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t* fmav_msg_radio_rc_channels_dev_get_field_channels_ptr(const fmav_message_t* msg)
{
    return (int16_t*)&(msg->payload[5]);
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_radio_rc_channels_dev_get_field_channels(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_RADIO_RC_CHANNELS_DEV_FIELD_CHANNELS_NUM) return 0;
    return ((int16_t*)&(msg->payload[5]))[index];
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_RADIO_RC_CHANNELS_DEV  420

#define mavlink_radio_rc_channels_dev_t  fmav_radio_rc_channels_dev_t

#define MAVLINK_MSG_ID_RADIO_RC_CHANNELS_DEV_LEN  69
#define MAVLINK_MSG_ID_RADIO_RC_CHANNELS_DEV_MIN_LEN  5
#define MAVLINK_MSG_ID_420_LEN  69
#define MAVLINK_MSG_ID_420_MIN_LEN  5

#define MAVLINK_MSG_ID_RADIO_RC_CHANNELS_DEV_CRC  174
#define MAVLINK_MSG_ID_420_CRC  174

#define MAVLINK_MSG_RADIO_RC_CHANNELS_DEV_FIELD_CHANNELS_LEN 32


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_radio_rc_channels_dev_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint8_t target_system, uint8_t target_component, uint8_t count, uint16_t flags, const int16_t* channels)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_radio_rc_channels_dev_pack(
        _msg, sysid, compid,
        target_system, target_component, count, flags, channels,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_radio_rc_channels_dev_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_radio_rc_channels_dev_t* _payload)
{
    return mavlink_msg_radio_rc_channels_dev_pack(
        sysid,
        compid,
        _msg,
        _payload->target_system, _payload->target_component, _payload->count, _payload->flags, _payload->channels);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_radio_rc_channels_dev_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint8_t count, uint16_t flags, const int16_t* channels)
{
    return fmav_msg_radio_rc_channels_dev_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        target_system, target_component, count, flags, channels,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_radio_rc_channels_dev_decode(const mavlink_message_t* msg, mavlink_radio_rc_channels_dev_t* payload)
{
    fmav_msg_radio_rc_channels_dev_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_RADIO_RC_CHANNELS_DEV_H
