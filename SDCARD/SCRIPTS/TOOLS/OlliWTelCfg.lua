--local toolName = "TNS|mTX OlliW Telem Config|TNE"
----------------------------------------------------------------------
-- OlliW Telemetry Widget Script for mTX
-- (c) www.olliw.eu, OlliW, OlliW42
-- licence: GPL 3.0
-- https://www.gnu.org/licenses/gpl-3.0.de.html
----------------------------------------------------------------------
-- Lua Configuration Script
----------------------------------------------------------------------
-- copy script to SCRIPTS\TOOLS folder on OpenTx SD card

local versionStr = "0.37 2024-01-06"


----------------------------------------------------------------------
-- Library Loader
----------------------------------------------------------------------
local sourcePath = "/WIDGETS/OlliwTel/"

local function loadLib(fname,...)
  local f = assert(loadScript(sourcePath..fname))
  collectgarbage()
  collectgarbage()
  return f(...)
end


----------------------------------------------------------------------
-- Widget Configuration
-- Color Map
----------------------------------------------------------------------

local config = loadLib("tconfig.lua")
local config_g, p = config.Load()
local config_options = config.Options()


----------------------------------------------------------------------
-- Script
----------------------------------------------------------------------

local cursor_idx = 1
local edit = false
local curr_option_value = nil


local sw_values = {
    "",
    "sa", "sb", "sc", "sd", "se", "sf", "sg", "sh",
    "6pos",
    "s1", "s2",
    "ls", "rs"
}


function indexOf(array, value)
    for i, v in ipairs(array) do
        if v == value then return i end
    end
    return nil
end


local function cur_attr(idx) -- used in menu
    local attr = TEXT_COLOR
    if cursor_idx == idx then
        attr = attr + INVERS
        if edit then attr = attr + BLINK end
    end
    return attr
end


local function edit_option_next(idx)
    if config_options[idx][2] == "bool" then
        if not config_g[config_options[idx][1]] then 
            config_g[config_options[idx][1]] = true
        end  
    elseif config_options[idx][2] == "sw" then
        local sw_i = indexOf(sw_values, config_g[config_options[idx][1]])
        if sw_i ~= nil then 
            sw_i = sw_i + 1
            if sw_i <= #sw_values then config_g[config_options[idx][1]] = sw_values[sw_i] end
        else
            config_g[config_options[idx][1]] = sw_values[1]
        end    
    end
end  


local function edit_option_prev(idx)
    if config_options[idx][2] == "bool" then
        if config_g[config_options[idx][1]] then 
            config_g[config_options[idx][1]] = false
        end  
    elseif config_options[idx][2] == "sw" then  
        local sw_i = indexOf(sw_values, config_g[config_options[idx][1]]) 
        if sw_i ~= nil then
            sw_i = sw_i - 1
            if sw_i >= 1 then config_g[config_options[idx][1]] = sw_values[sw_i] end
        else
            config_g[config_options[idx][1]] = sw_values[#sw_values]
        end    
    end
end  


local function drawPage()
    local y = 35

    for idx = 1,#config_options do
        lcd.drawText(5, y, config_options[idx][3], TEXT_COLOR)
      
        lcd.drawText(5+240, y, ":", TEXT_COLOR)
        
        local option_str = ""
        if config_options[idx][2] == "bool" then
            if config_g[config_options[idx][1]] then 
                option_str = "true"
            else    
                option_str = "false"
            end  
        elseif config_options[idx][2] == "sw" then  
            option_str = config_g[config_options[idx][1]]
        end
        if string.len(option_str) == 0 then option_str = "-" end 
        
        lcd.drawText(5+260, y, option_str, cur_attr(idx))
      
        y = y + 20
    end
end  


local function Do(event)
    lcd.clear()
    lcd.drawFilledRectangle(0, 0, LCD_W, 30, TITLE_BGCOLOR)
    lcd.drawText(5, 5, "OlliW Telemetry Configurator", MENU_TITLE_COLOR)
    lcd.drawText(LCD_W-1, 0, versionStr, MENU_TITLE_COLOR+TINSIZE+RIGHT)

    lcd.setColor(CUSTOM_COLOR, RED)

    if config == nil then
        lcd.drawText(5, 35, "ERROR: tconfig could not be loaded", CUSTOM_COLOR)
        return
    end  
    if config_g == nil then
        lcd.drawText(5, 35, "ERROR: config_g could not be loaded", CUSTOM_COLOR)
        return
    end
    if config_options == nil then
        lcd.drawText(5, 35, "ERROR: config_options could not be loaded", CUSTOM_COLOR)
        return
    end
    
    if not edit then
        if event == EVT_VIRTUAL_EXIT then
            -- nothing to do
        elseif event == EVT_VIRTUAL_ENTER then
            curr_option_value = config_g[config_options[cursor_idx][1]]
            edit = true
        elseif event == EVT_VIRTUAL_NEXT then
            cursor_idx = cursor_idx + 1
            if cursor_idx > #config_options then cursor_idx = #config_options end
        elseif event == EVT_VIRTUAL_PREV then
            cursor_idx = cursor_idx - 1
            if cursor_idx < 1 then cursor_idx = 1 end
        end
    else    
        if event == EVT_VIRTUAL_EXIT then
            config_g[config_options[cursor_idx][1]] = curr_option_value
            edit = false
        elseif event == EVT_VIRTUAL_ENTER then
            config.Write()
            edit = false
        elseif event == EVT_VIRTUAL_NEXT then
            edit_option_next(cursor_idx)
        elseif event == EVT_VIRTUAL_PREV then
            edit_option_prev(cursor_idx)
        end
    end    
    
    drawPage()
end


----------------------------------------------------------------------
-- Script OTX Interface
----------------------------------------------------------------------

local function scriptInit()
end


local function scriptRun(event)
    if event == nil then
        error("Cannot be run as a model script!")
        return 2
    end

    Do(event)

    return 0
end


return { init=scriptInit, run=scriptRun }
