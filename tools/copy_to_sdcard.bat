ECHO OFF
SETLOCAL

SET cpysrcr=..\lua_telem_script

SET cpydir=..\SDCARD\WIDGETS\OlliWTel\

copy %cpysrcr%\tplay.lua %cpydir%
copy %cpysrcr%\tutils.lua %cpydir%
copy %cpysrcr%\tvehicle.lua %cpydir%
copy %cpysrcr%\tobject.lua %cpydir%

copy %cpysrcr%\tautopilot.lua %cpydir%
copy %cpysrcr%\tcamera.lua %cpydir%
copy %cpysrcr%\tgimbal.lua %cpydir%
copy %cpysrcr%\taction.lua %cpydir%
copy %cpysrcr%\tdebug.lua %cpydir%

copy %cpysrcr%\tconfig.lua %cpydir%

copy ..\lua_telem_script\main.lua %cpydir%


SET cpydir=..\SDCARD\SCRIPTS\TOOLS\
copy %cpysrcr%\OlliWTelCfg.lua %cpydir%

pause

