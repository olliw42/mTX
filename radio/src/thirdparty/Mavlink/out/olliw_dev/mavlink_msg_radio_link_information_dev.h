//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_RADIO_LINK_INFORMATION_DEV_H
#define FASTMAVLINK_MSG_RADIO_LINK_INFORMATION_DEV_H


//----------------------------------------
//-- Message RADIO_LINK_INFORMATION_DEV
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_radio_link_information_dev_t {
    uint16_t tx_rate;
    uint16_t rx_rate;
    uint8_t target_system;
    uint8_t target_component;
    uint8_t type;
    uint8_t mode;
    uint8_t tx_receive_sensitivity;
    uint8_t rx_receive_sensitivity;
}) fmav_radio_link_information_dev_t;


#define FASTMAVLINK_MSG_ID_RADIO_LINK_INFORMATION_DEV  422

#define FASTMAVLINK_MSG_RADIO_LINK_INFORMATION_DEV_PAYLOAD_LEN_MAX  10
#define FASTMAVLINK_MSG_RADIO_LINK_INFORMATION_DEV_CRCEXTRA  9

#define FASTMAVLINK_MSG_RADIO_LINK_INFORMATION_DEV_FLAGS  3
#define FASTMAVLINK_MSG_RADIO_LINK_INFORMATION_DEV_TARGET_SYSTEM_OFS  4
#define FASTMAVLINK_MSG_RADIO_LINK_INFORMATION_DEV_TARGET_COMPONENT_OFS  5

#define FASTMAVLINK_MSG_RADIO_LINK_INFORMATION_DEV_FRAME_LEN_MAX  35



#define FASTMAVLINK_MSG_RADIO_LINK_INFORMATION_DEV_FIELD_TX_RATE_OFS  0
#define FASTMAVLINK_MSG_RADIO_LINK_INFORMATION_DEV_FIELD_RX_RATE_OFS  2
#define FASTMAVLINK_MSG_RADIO_LINK_INFORMATION_DEV_FIELD_TARGET_SYSTEM_OFS  4
#define FASTMAVLINK_MSG_RADIO_LINK_INFORMATION_DEV_FIELD_TARGET_COMPONENT_OFS  5
#define FASTMAVLINK_MSG_RADIO_LINK_INFORMATION_DEV_FIELD_TYPE_OFS  6
#define FASTMAVLINK_MSG_RADIO_LINK_INFORMATION_DEV_FIELD_MODE_OFS  7
#define FASTMAVLINK_MSG_RADIO_LINK_INFORMATION_DEV_FIELD_TX_RECEIVE_SENSITIVITY_OFS  8
#define FASTMAVLINK_MSG_RADIO_LINK_INFORMATION_DEV_FIELD_RX_RECEIVE_SENSITIVITY_OFS  9


//----------------------------------------
//-- Message RADIO_LINK_INFORMATION_DEV pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_radio_link_information_dev_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint8_t type, uint8_t mode, uint16_t tx_rate, uint16_t rx_rate, uint8_t tx_receive_sensitivity, uint8_t rx_receive_sensitivity,
    fmav_status_t* _status)
{
    fmav_radio_link_information_dev_t* _payload = (fmav_radio_link_information_dev_t*)_msg->payload;

    _payload->tx_rate = tx_rate;
    _payload->rx_rate = rx_rate;
    _payload->target_system = target_system;
    _payload->target_component = target_component;
    _payload->type = type;
    _payload->mode = mode;
    _payload->tx_receive_sensitivity = tx_receive_sensitivity;
    _payload->rx_receive_sensitivity = rx_receive_sensitivity;


    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_RADIO_LINK_INFORMATION_DEV;
    _msg->target_sysid = target_system;
    _msg->target_compid = target_component;
    _msg->crc_extra = FASTMAVLINK_MSG_RADIO_LINK_INFORMATION_DEV_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_RADIO_LINK_INFORMATION_DEV_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_radio_link_information_dev_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_radio_link_information_dev_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_radio_link_information_dev_pack(
        _msg, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->type, _payload->mode, _payload->tx_rate, _payload->rx_rate, _payload->tx_receive_sensitivity, _payload->rx_receive_sensitivity,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_radio_link_information_dev_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint8_t type, uint8_t mode, uint16_t tx_rate, uint16_t rx_rate, uint8_t tx_receive_sensitivity, uint8_t rx_receive_sensitivity,
    fmav_status_t* _status)
{
    fmav_radio_link_information_dev_t* _payload = (fmav_radio_link_information_dev_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->tx_rate = tx_rate;
    _payload->rx_rate = rx_rate;
    _payload->target_system = target_system;
    _payload->target_component = target_component;
    _payload->type = type;
    _payload->mode = mode;
    _payload->tx_receive_sensitivity = tx_receive_sensitivity;
    _payload->rx_receive_sensitivity = rx_receive_sensitivity;


    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_RADIO_LINK_INFORMATION_DEV;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_RADIO_LINK_INFORMATION_DEV >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_RADIO_LINK_INFORMATION_DEV >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_RADIO_LINK_INFORMATION_DEV_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_RADIO_LINK_INFORMATION_DEV_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_radio_link_information_dev_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_radio_link_information_dev_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_radio_link_information_dev_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->type, _payload->mode, _payload->tx_rate, _payload->rx_rate, _payload->tx_receive_sensitivity, _payload->rx_receive_sensitivity,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_radio_link_information_dev_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint8_t type, uint8_t mode, uint16_t tx_rate, uint16_t rx_rate, uint8_t tx_receive_sensitivity, uint8_t rx_receive_sensitivity,
    fmav_status_t* _status)
{
    fmav_radio_link_information_dev_t _payload;

    _payload.tx_rate = tx_rate;
    _payload.rx_rate = rx_rate;
    _payload.target_system = target_system;
    _payload.target_component = target_component;
    _payload.type = type;
    _payload.mode = mode;
    _payload.tx_receive_sensitivity = tx_receive_sensitivity;
    _payload.rx_receive_sensitivity = rx_receive_sensitivity;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_RADIO_LINK_INFORMATION_DEV,
        FASTMAVLINK_MSG_RADIO_LINK_INFORMATION_DEV_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_RADIO_LINK_INFORMATION_DEV_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_radio_link_information_dev_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_radio_link_information_dev_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_RADIO_LINK_INFORMATION_DEV,
        FASTMAVLINK_MSG_RADIO_LINK_INFORMATION_DEV_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_RADIO_LINK_INFORMATION_DEV_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message RADIO_LINK_INFORMATION_DEV decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_radio_link_information_dev_decode(fmav_radio_link_information_dev_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_RADIO_LINK_INFORMATION_DEV_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_RADIO_LINK_INFORMATION_DEV_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_RADIO_LINK_INFORMATION_DEV_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_RADIO_LINK_INFORMATION_DEV_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_radio_link_information_dev_get_field_tx_rate(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_radio_link_information_dev_get_field_rx_rate(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[2]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_radio_link_information_dev_get_field_target_system(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[4]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_radio_link_information_dev_get_field_target_component(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[5]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_radio_link_information_dev_get_field_type(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[6]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_radio_link_information_dev_get_field_mode(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[7]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_radio_link_information_dev_get_field_tx_receive_sensitivity(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[8]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_radio_link_information_dev_get_field_rx_receive_sensitivity(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[9]), sizeof(uint8_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_RADIO_LINK_INFORMATION_DEV  422

#define mavlink_radio_link_information_dev_t  fmav_radio_link_information_dev_t

#define MAVLINK_MSG_ID_RADIO_LINK_INFORMATION_DEV_LEN  10
#define MAVLINK_MSG_ID_RADIO_LINK_INFORMATION_DEV_MIN_LEN  10
#define MAVLINK_MSG_ID_422_LEN  10
#define MAVLINK_MSG_ID_422_MIN_LEN  10

#define MAVLINK_MSG_ID_RADIO_LINK_INFORMATION_DEV_CRC  9
#define MAVLINK_MSG_ID_422_CRC  9




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_radio_link_information_dev_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint8_t target_system, uint8_t target_component, uint8_t type, uint8_t mode, uint16_t tx_rate, uint16_t rx_rate, uint8_t tx_receive_sensitivity, uint8_t rx_receive_sensitivity)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_radio_link_information_dev_pack(
        _msg, sysid, compid,
        target_system, target_component, type, mode, tx_rate, rx_rate, tx_receive_sensitivity, rx_receive_sensitivity,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_radio_link_information_dev_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_radio_link_information_dev_t* _payload)
{
    return mavlink_msg_radio_link_information_dev_pack(
        sysid,
        compid,
        _msg,
        _payload->target_system, _payload->target_component, _payload->type, _payload->mode, _payload->tx_rate, _payload->rx_rate, _payload->tx_receive_sensitivity, _payload->rx_receive_sensitivity);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_radio_link_information_dev_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint8_t type, uint8_t mode, uint16_t tx_rate, uint16_t rx_rate, uint8_t tx_receive_sensitivity, uint8_t rx_receive_sensitivity)
{
    return fmav_msg_radio_link_information_dev_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        target_system, target_component, type, mode, tx_rate, rx_rate, tx_receive_sensitivity, rx_receive_sensitivity,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_radio_link_information_dev_decode(const mavlink_message_t* msg, mavlink_radio_link_information_dev_t* payload)
{
    fmav_msg_radio_link_information_dev_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_RADIO_LINK_INFORMATION_DEV_H
