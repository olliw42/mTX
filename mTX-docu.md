# mTX Documentation

Very rudimentary documentation of few things.

## RSSI

The topic of RSSI is complicated, as there are many different sources and different units/scalings in the game.

#### Units/scaling
- **dBm**: real units so to say, rssi is negative (except in super close range)
- **negated dBm**: like dBm but with opposite sign, rssi is positive (except in super close range)
- **mavlink units**: device-dependent units/scale, values 0 .. 254, 255 = invalid/unknown
- **ArduPilot scaling**: values 0 .. 255
- **percentage**: 0 .. 100%, the scaling or reference points can vary
- **ArduPilot CRSF scaling**: converts dBm to mavlink units using the formula (1 - (|rssi| - 50)/70) * 255, 50/-50 dBm corresponds to 255, 120/-120 dBm corresponds to 0

#### Sources
- **MAVLink**
  - MAVLink message RADIO_STATUS (#109) 
    - uint8_t rssi: mavlink units, rssi for reception on Tx module
    - uint8_t remrssi: mavlink units, rssi for reception on Rx module
    - uint8_t noise: noise on Tx module side
    - uint8_t remnoise: noise on Rx module side
  - MAVLink message RC_CHANNELS (#65)
    - uint8_t rssi: mavlink units
  - MAVLink message RC_CHANNELS_RAW (#35)
    - uint8_t rssi: mavlink units
- **mBridge**
  - rssi in negative dBm, is converted to mavlink units using AP CRSF formula
- **CRSF**
  - none yet

#### mTx MavSdk Lua Functions
- **getRadioRssiRaw()**: rssi in mavlink units. Selects from the various sources according in this priority: mBridge, RADIO_STATUS, RC_CHANNELS, RC_CHANNELS_RAW
- **getRadioRssiScaled()**: same as getRadioRssiRaw() but if the parameter mavlinkRssiScale is non zero it is scaled into percentage (0% .. 100%) using that scale factor
- **getRadioRssi()**: if the parameter mavlinkRssiScale is non zero return the value of getRadioRssiScaled() else that of getRadioRssiRaw(). Should normally be used 

## LQ

#### Sources
- **MAVLink**: none
- **mBridge**:
  - rssi in negative dBm, is converted to mavlink units using AP CRSF formula
- **CRSF**: none

#### mTx MavSdk Lua Functions
- **getRadioLQ()**:

