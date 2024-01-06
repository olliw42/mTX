----------------------------------------------------------------------
-- OlliW Telemetry Widget Script
-- (c) www.olliw.eu, OlliW, OlliW42
-- licence: GPL 3.0
-- https://www.gnu.org/licenses/gpl-3.0.de.html
----------------------------------------------------------------------
-- Configuration
----------------------------------------------------------------------


----------------------------------------------------------------------
-- Widget Configuration
----------------------------------------------------------------------
-- Please feel free to set these configuration options as you desire

local config_g = {
    -- Set to true if you want to see the Action page, else set to false
    showActionPage = false, --true,
    
    -- Set to true if you want to see the Camera page, else set to false
    showCameraPage = true,
    
    -- Set to true if you want to see the Gimbal page, else set to false
    showGimbalPage = true,
    
    -- Set to a (toggle) source if you want control videoon/of & take photo with a switch,
    -- else set to ""
    cameraShootSwitch = "sh",
    
    -- Set to true if camera should be included in overall prearm check, else set to false
    cameraPrearmCheck = false,
    
    -- Set to a source if you want control the gimbal pitch, else set to ""
    gimbalPitchSlider = "rs",
    
    -- Set to the appropriate value if you want to start teh gimbal in a given targeting mode, 
    -- else set to nil
    -- 2: MAVLink Targeting, 3: RC Targeting, 4: GPS Point Targeting, 5: SysId Targeting
    gimbalDefaultTargetingMode = 3,
    
    -- Set to true if gimbal should be included in overall prearm check, else set to false
    gimbalPrearmCheck = false,
    
    -- Set to true for gimbal manager protocol v2, else false for old gimbal protocol v1
    gimbalUseGimbalManager = true,

    -- Set to a source if you want control the gimbal yaw,
    -- else set to "", which also disables gimbal yaw control
    -- only relevant if gimbalUseGimbalManager = true
    gimbalYawSlider = "", --"ls",
    
    -- Set to true if you do not want to hear any voice, else set to false
    disableSound = false,
    
    -- not for you ;)
    disableEvents = false, -- not needed, just to have it safe
    
    -- Set to true if you want to see the Debug page, else set to false
    showDebugPage = false, --true,
}


----------------------------------------------------------------------
-- Color Map
----------------------------------------------------------------------

local p = {
    WHITE =       lcd.RGB(0xFF,0xFF,0xFF),
    BLACK =       lcd.RGB(0,0,0),
    RED =         RED,        --RGB(229, 32, 30)
    DARKRED =     DARKRED,    --RGB(160, 0, 6)
    GREEN =       lcd.RGB(25,150,50),
    BLUE =        BLUE,       --RGB(0x30, 0xA0, 0xE0)
    YELLOW =      YELLOW,     --RGB(0xF0, 0xD0, 0x10)
    GREY =        GREY,       --RGB(96, 96, 96)
    DARKGREY =    DARKGREY,   --RGB(64, 64, 64)
    LIGHTGREY =   LIGHTGREY,  --RGB(180, 180, 180)
    SKYBLUE =     lcd.RGB(135,206,235),
    OLIVEDRAB =   lcd.RGB(107,142,35),
    YAAPUBROWN =  lcd.RGB(0x63,0x30,0x00),
    YAAPUBLUE =   lcd.RGB(0x08,0x54,0x88),
    BRIGHTGREEN = lcd.RGB(0,255,0),
}    
p.HUD_SKY = p.SKYBLUE
p.HUD_EARTH = p.OLIVEDRAB
p.BACKGROUND = p.YAAPUBLUE
p.CAMERA_BACKGROUND = p.YAAPUBLUE
p.GIMBAL_BACKGROUND = p.YAAPUBLUE


----------------------------------------------------------------------
-- Conig functions
----------------------------------------------------------------------

local config_options = {
  { "showActionPage", "bool", "show Action page" },
  { "showCameraPage", "bool", "show Camera page" },
  { "showGimbalPage", "bool", "show Gimbal page" },
  { "showDebugPage", "bool", "show Debug page" },

  { "cameraShootSwitch", "sw", "Camera Shoot Switch" },
  { "cameraPrearmCheck", "bool", "Camera Prearm Check" },

  { "gimbalPitchSlider", "sw", "Gimbal Pitch Slider" },
  { "gimbalYawSlider", "sw", "Gimbal Yaw Slider" },
  { "gimbalPrearmCheck", "bool", "Gimbal Prearm Check" },
  { "gimbalUseGimbalManager", "bool", "Gimbal Use Gimbal Manager" },
  
  --{ "gimbalDefaultTargetingMode", "list", 
  
  { "disableSound", "bool", "disable Sound" }
}


local config = {}

local configFile = "/WIDGETS/OlliwTel/cfg.txt"


function config:Options()
    return config_options
end    


function config:Write()
    local file = assert(io.open(configFile, "w"))
    if file ~= nil then
        for i = 1,#config_options do
            io.write(file, config_options[i][1])
            io.write(file, ":")
            if config_options[i][2] == "bool" then
                if config_g[config_options[i][1]] then 
                    io.write(file, "true")
                else    
                    io.write(file, "false")
                end  
            elseif config_options[i][2] == "sw" then  
                io.write(file, config_g[config_options[i][1]])
            end
            io.write(file, ",")
        end    
        io.close(file)
    end
end  


function config:Load()
    local found = false
    local config_str

    local file = io.open(configFile, "r")
    if file ~= nil then
        config_str = io.read(file, 1024)
        io.close(file)
        if string.len(config_str) > 0 then found = true end
    end  

    if not found then
        config.Write()
        return config_g, p -- use config_g as is
    end

    for i = 1,#config_options do
        local value = string.match(config_str, config_options[i][1]..":([%a]+),")
        if value ~= nil then
            if config_options[i][2] == "bool" then
                if value == "true" then
                    config_g[config_options[i][1]] = true
                else    
                    config_g[config_options[i][1]] = false
                end    
            elseif config_options[i][2] == "sw" then  
                config_g[config_options[i][1]] = value
            end
        end  
    end
    
    return config_g, p
end  


return 
  config

  
  
