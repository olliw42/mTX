set(LUA "NO" CACHE STRING "Lua scripts (YES/NO/NO_MODEL_SCRIPTS)")
set_property(CACHE LUA PROPERTY STRINGS YES NO NO_MODEL_SCRIPTS)
set(LUA_SCRIPT_LOAD_MODE "" CACHE STRING "Script loading mode and compilation flags [btTxcd] (see loadScript() API docs). Blank for default ('bt' on radio, 'T' on SIMU/DEBUG builds)")
option(LUA_COMPILER "Pre-compile and save Lua scripts" ON)
option(LUA_ALLOCATOR_TRACER "Trace Lua memory (de)allocations to debug port (also needs DEBUG=YES NANO=NO)" OFF)

option(USB_SERIAL "Enable USB serial (CDC)" OFF)

set(ARCH ARM)
set(STM32USB_DIR ${THIRDPARTY_DIR}/STM32_USB-Host-Device_Lib_V2.2.0/Libraries)
add_definitions(-DSTM32 -DLUA_INPUTS -DVARIO)

include_directories(${RADIO_SRC_DIRECTORY}/targets/common/arm/stm32)
include_directories(${STM32USB_DIR}/STM32_USB_OTG_Driver/inc)
include_directories(${STM32USB_DIR}/STM32_USB_Device_Library/Core/inc)
include_directories(${STM32USB_DIR}/STM32_USB_Device_Library/Class/msc/inc)
include_directories(${STM32USB_DIR}/STM32_USB_Device_Library/Class/hid/inc)
include_directories(${STM32USB_DIR}/STM32_USB_Device_Library/Class/cdc/inc)
include_directories(${STM32USB_DIR}/STM32_USB_Device_Library/Core/inc/)
include_directories(${STM32USB_DIR}/STM32_USB_OTG_Driver/inc)

set(STM32USB_SRC
  STM32_USB_OTG_Driver/src/usb_core.c
  STM32_USB_OTG_Driver/src/usb_dcd.c
  STM32_USB_OTG_Driver/src/usb_dcd_int.c
  STM32_USB_Device_Library/Core/src/usbd_core.c
  STM32_USB_Device_Library/Core/src/usbd_ioreq.c
  STM32_USB_Device_Library/Core/src/usbd_req.c
  )

set(FIRMWARE_TARGET_SRC
  ${FIRMWARE_TARGET_SRC}
  ../common/arm/stm32/rtc_driver.cpp
  ${AUX_SERIAL_DRIVER}
  ../common/arm/stm32/delays.cpp
  ../common/arm/stm32/usb_bsp.c
  ../common/arm/stm32/usbd_desc.c
  ../common/arm/stm32/usbd_usr.cpp
  ../common/arm/stm32/usb_driver.cpp
  ../common/arm/stm32/flash_driver.cpp
  ../common/arm/stm32/usbd_storage_msd.cpp
  ../common/arm/stm32/usbd_hid_joystick.c
  )

# //OW
# //this cmake runs before target/horus/CMakeList.txt
if (TELEMETRY_MAVLINK)
  add_definitions(-DTELEMETRY_MAVLINK)
  # determine if usb serial should be added
  set(TELEMETRY_MAVLINK_USB_SERIAL ON)
  if (DEBUG OR CLI)
    #set(TELEMETRY_MAVLINK_USB_SERIAL OFF)
    #message(FATAL_ERROR "don't compile with DEBUG or CLI")
  endif()
  # generate required header files from fastmavlink submodule
  execute_process(
    COMMAND ${PYTHON_EXECUTABLE} fmav_generate_c_library.py 
    WORKING_DIRECTORY ${RADIO_DIRECTORY}/src/${THIRDPARTY_DIR}/Mavlink
    RESULT_VARIABLE FMAV_GENERATE_CLIB_RESULT
    )
  if (NOT FMAV_GENERATE_CLIB_RESULT EQUAL "0")
    message(FATAL_ERROR "fmav_generate_c_library.py could not be run: ${FMAV_GENERATE_CLIB_RESULT}")
  endif()
  execute_process(
    COMMAND ${PYTHON_EXECUTABLE} fmav_generate_lua_lib.py -nw
    WORKING_DIRECTORY ${RADIO_DIRECTORY}/src/${THIRDPARTY_DIR}/Mavlink
    RESULT_VARIABLE FMAV_GENERATE_LUALIB_RESULT
    )
  if (NOT FMAV_GENERATE_LUALIB_RESULT EQUAL "0")
    message(FATAL_ERROR "fmav_generate_lua_lib.py could not be run: ${FMAV_GENERATE_LUALIB_RESULT}")
  endif()
endif()
# //OWEND

# //OW
# //if(DEBUG OR CLI OR USB_SERIAL)
if (DEBUG OR CLI OR USB_SERIAL OR TELEMETRY_MAVLINK_USB_SERIAL)
# //OWEND
  set(STM32USB_SRC
    ${STM32USB_SRC}
    STM32_USB_Device_Library/Class/cdc/src/usbd_cdc_core.c
  )
  set(FIRMWARE_TARGET_SRC
    ${FIRMWARE_TARGET_SRC}
    ../common/arm/stm32/usbd_cdc.cpp
  )
# //OW
# //  add_definitions(-DUSB_SERIAL)
  if (DEBUG OR CLI OR USB_SERIAL)
    add_definitions(-DUSB_SERIAL)
  endif()
  if (TELEMETRY_MAVLINK_USB_SERIAL)
    add_definitions(-DTELEMETRY_MAVLINK_USB_SERIAL)
  endif()
# //OWEND
endif()

set(STM32USB_SRC
  ${STM32USB_SRC}
  STM32_USB_Device_Library/Class/msc/src/usbd_msc_data.c
  STM32_USB_Device_Library/Class/msc/src/usbd_msc_scsi.c
  STM32_USB_Device_Library/Class/msc/src/usbd_msc_bot.c
  STM32_USB_Device_Library/Class/msc/src/usbd_msc_core.c
  )

if(GVARS)
  set(GUI_SRC
    ${GUI_SRC}
    ${GVAR_SCREEN}
    )
endif()

set(FIRMWARE_SRC
  ${FIRMWARE_SRC}
  serial.cpp
  syscalls.c
  )

foreach(FILE ${STM32LIB_SRC})
  set(FIRMWARE_SRC
    ${FIRMWARE_SRC}
    ${STM32LIB_DIR}/${FILE}
    )
endforeach()

foreach(FILE ${STM32USB_SRC})
  set(FIRMWARE_SRC
    ${FIRMWARE_SRC}
    ${STM32USB_DIR}/${FILE}
    )
endforeach()

set(TARGET_SRC
  ${TARGET_SRC}
  ../common/arm/stm32/cpu_id.cpp
  )

set(SRC
  ${SRC}
  io/bootloader_flash.cpp
  )
