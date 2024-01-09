## mTX: MAVLink for OpenTX

The mTX project adds bi-directional, native MAVLink support to the OpenTx firmware.

Project web page: http://www.olliw.eu/2020/olliwtelem/

Discussion thread: https://www.rcgroups.com/forums/showthread.php?3532969-MAVLink-for-OpenTx-and-Telemetry-Script

Youtube playlist: https://youtube.com/playlist?list=PLCgau2zkFD_nKGHDyU4nltgT2IfCj1CN2

### OpenTX

mTX is a fork of OpenTx 2.3.15.

Refer to the [OpenTX wiki](https://github.com/opentx/opentx/wiki) for information about setting up a tool chain for building OpenTX and other development/developer related topics.

### Installation

Installation largely follows the installation of OpenTx 2.3.15. It involves three steps:

1. Flash the mTX firmware binary appropriate for your radio to the radio. You can follow the standard OpenTx descriptions, except that you choose a different binary file. A convenient method for flashing can be using the OpenTx Companion.
2. Copy the SD content required for OpenTx 2.3.15 to the radio's SD card. Follow the OpenTx descriptions for this step.
3. Copy the SD card content provided in the SDCARD folder to the radio's SD card. 

Useful links:
 * OpenTX Main Site https://www.open-tx.org/
 * OpenTX User Manual https://www.gitbook.com/book/opentx/opentx-taranis-manual/details
 * OpenTX Lua Reference Guide https://www.gitbook.com/book/opentx/opentx-lua-reference-guide/details
