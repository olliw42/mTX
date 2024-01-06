ECHO OFF
SETLOCAL

SET cpydir=SDCARD\WIDGETS\OlliWTel\

copy lua_telem_script\tplay.lua %cpydir%
copy lua_telem_script\tutils.lua %cpydir%
copy lua_telem_script\tvehicle.lua %cpydir%
copy lua_telem_script\tobject.lua %cpydir%

copy lua_telem_script\tautopilot.lua %cpydir%
copy lua_telem_script\tcamera.lua %cpydir%
copy lua_telem_script\tgimbal.lua %cpydir%
copy lua_telem_script\taction.lua %cpydir%
copy lua_telem_script\tdebug.lua %cpydir%

copy lua_telem_script\tconfig.lua %cpydir%

copy lua_telem_script\main.lua %cpydir%


SET cpydir=SDCARD\SCRIPTS\TOOLS\
copy lua_telem_script\OlliWTelCfg.lua %cpydir%

pause

