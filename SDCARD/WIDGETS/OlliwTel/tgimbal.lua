----------------------------------------------------------------------
-- OlliW Telemetry Widget Script
-- (c) www.olliw.eu, OlliW, OlliW42
-- licence: GPL 3.0
-- https://www.gnu.org/licenses/gpl-3.0.de.html
----------------------------------------------------------------------
-- Page Gimbal
----------------------------------------------------------------------
local config_g, status_g, p, draw, play, tmenu, tbutton = ...


local LCD_XMID = draw.xmid


local function drawNoGimbal()
    if mavsdk.isReceiving() and not mavsdk.gimbalIsReceiving() then
        draw:WarningBox("no gimbal")
        return true
    end
    return false
end


local function getGimbalIdStr(compid)
    if compid == mavlink.COMP_ID_GIMBAL then
        return "Gimbal1"
    elseif compid >= mavlink.COMP_ID_GIMBAL2 and compid <= mavlink.COMP_ID_GIMBAL6 then
        return "Gimbal"..tostring(compid - mavlink.COMP_ID_GIMBAL2 + 2)
    end
    return "Gimbal"
end    


local gimbal = {
    initialized = false,
  
    pitch_cntrl_deg = nil,
    yaw_cntrl_deg = 0,
    mount_mode = nil,
    set_mode_cnt = 0,
    set_roisysid_cnt = 0,
    is_in_control = false,
    debug = true, --false,
}    


local function gimbalSetMode_idx(idx, sound_flag)
    if idx == 1 then
        mavsdk.gimbalClientSetRcControl(0)
        mavsdk.gimbalSendMavlinkTargetingMode()
        gimbal.mount_mode = mavlink.MOUNT_MODE_MAVLINK_TARGETING
        if sound_flag then play:QShotDefault() end
    elseif idx == 2 then
        mavsdk.gimbalClientSetRcControl(0)
        mavsdk.gimbalSendNeutralMode()
        gimbal.mount_mode = mavlink.MOUNT_MODE_NEUTRAL
        if sound_flag then play:QShotNeutral() end
    elseif idx == 3 then
        mavsdk.gimbalClientSetRcControl(0)
        mavsdk.gimbalSendRcTargetingMode()
        gimbal.mount_mode = mavlink.MOUNT_MODE_RC_TARGETING
        if sound_flag then play:QShotRcControl() end
    elseif idx == 4 then
        mavsdk.gimbalClientSetRcControl(0)
        mavsdk.gimbalSendSysIdTargetingMode()
        gimbal.mount_mode = mavlink.MOUNT_MODE_SYSID_TARGET
        if sound_flag then play:QShotTargetMe() end
    elseif idx == 5 then
        mavsdk.gimbalClientSetRcControl(1)
        mavsdk.gimbalSendSysIdTargetingMode()
        gimbal.mount_mode = mavlink.MOUNT_MODE_SYSID_TARGET
        if sound_flag then play:QShotTargetMe() end
--[[        
    elseif idx == 4 then
        mavsdk.gimbalSendGpsPointMode()
        if sound_flag then play:QShotPOI() end
]]        
    end
end  


local function gimbalSetRoiSysId()
    local ap_sysid, ap_compid = mavlink.getAutopilotIds()
    local my_sysid, my_compid = mavlink.getMyIds()
    mavlink.sendMessage({
        msgid = mavlink.M_COMMAND_LONG,
        target_sysid = ap_sysid,
        target_compid = ap_compid,
        command = mavlink.CMD_DO_SET_ROI_SYSID,
        param1 = my_sysid,
        param2 = 0, --gimbal device id, ignored by ArduPilot
    })
end


-- this is a wrapper
local function gimbalSetPitchYawDeg(pitch, yaw)
    if not config_g.gimbalUseGimbalManager then
        yaw = 0.0 -- clear yaw since it would turn the copter!
        mavsdk.gimbalSendPitchYawDeg(pitch, yaw) -- v1
    else
        if gimbal.mount_mode ~= mavlink.MOUNT_MODE_MAVLINK_TARGETING then
            mavsdk.gimbalClientSendFlags() -- v2
            return
        end  
        mavsdk.gimbalClientSendPitchYawDeg(pitch, yaw) -- v2
--        mavsdk.gimbalClientSendSetAttitudePitchYawDeg(pitch, yaw) -- v2
--        mavsdk.gimbalClientSendCmdPitchYawDeg(pitch, yaw) -- v2
    end    
end


local function gimbal_menu_click(menu, idx)
    gimbal.set_mode_cnt = 0
    gimbal.set_roisysid_cnt = 0
    gimbalSetMode_idx(idx, true)
    if gimbal.mount_mode == mavlink.MOUNT_MODE_MAVLINK_TARGETING then
        -- sending once is sufficient, since AP sets the mode
        -- but BP does not, so repeat
        gimbal.set_mode_cnt = 2  -- send 2 more times, to be sure it is received
    elseif gimbal.mount_mode == mavlink.MOUNT_MODE_NEUTRAL then
        -- we send FLags, so not needed
    elseif gimbal.mount_mode == mavlink.MOUNT_MODE_RC_TARGETING then
        gimbal.set_mode_cnt = 2  -- send mode 2 more times, to be sure it is received
    elseif gimbal.mount_mode == mavlink.MOUNT_MODE_SYSID_TARGET then
        -- sending once is sufficient, since AP sets the mode upon gimbalSetRoiSysId
        gimbal.set_roisysid_cnt = 3 
    end  
end


local gimbal_menu = {
    rect = { x = LCD_XMID - 240/2, y = 235, w = 240, h = 34 },
    click_func = gimbal_menu_click,
    min = 1, max = 5, default = 1, 
--    option = { "Default", "Neutral", "RC Control", "POI Targeting", "Cable Cam" },
    option = { "Default", "Neutral", "RC Control", "Target Me", "Target Me w/nudge" },
}


local function gimbal_claim_control_button_click()
    local gm_sysid, gm_compid = mavlink.getGimbalManagerIds()
    local gd_sysid, gd_compid = mavlink.getGimbalIds()
    local my_sysid, my_compid = mavlink.getMyIds()
    mavlink.sendMessage({
        msgid = mavlink.M_COMMAND_LONG,
        target_sysid = gm_sysid,
        target_compid = gm_compid,
        command = mavlink.CMD_DO_GIM_MAN_CONFIGURE,
        param1 = my_sysid, -- sysid primary control
        param2 = my_compid, -- compid primary control
        param3 = -1, -- sysid secondary control
        param4 = -1, -- compid  secondary control
        param7 = gd_compid, -- gimbal device id
    })
end


local gimbal_claim_control_button = {
    rect = { x = LCD_XMID - 240/2, y = 235, w = 240, h = 34 },
    txt = "Claim Control",
    click_func = gimbal_claim_control_button_click,
}


local function drawNoControl()
    draw:WarningBox("gimbal not in control")
end



----------------------------------------------------------------------
-- Interface
----------------------------------------------------------------------

local function gimbalDoAlways()
    if not mavsdk.gimbalIsReceiving() then return end
  
    if not gimbal.initialized then
        gimbal.initialized = true
        tmenu.init(gimbal_menu)
        tbutton.init(gimbal_claim_control_button)
    end    
  
    -- set gimbal into default MAVLink targeting mode upon connection
    if status_g.gimbal_changed_to_receiving then
        gimbal_menu.idx = gimbal_menu.default
        gimbal.set_mode_cnt = 4  -- send mode 4 times, to be sure it is received
        gimbal.set_roisysid_cnt = 0
    end
    
    local gm_status = mavsdk.gimbalManagerGetStatus()
    local my_sysid, my_compid = mavlink.getMyIds()
    if (gm_status.primary_sysid == 0 and gm_status.primary_compid == 0) or 
       (gm_status.primary_sysid == my_sysid and gm_status.primary_compid == my_compid) then
        gimbal.is_in_control = true
    else     
        gimbal.is_in_control = false
        return
    end
    
    if gimbal.set_mode_cnt > 0 then
        gimbal.set_mode_cnt = gimbal.set_mode_cnt - 1
        gimbalSetMode_idx(gimbal_menu.idx, false)
        return -- give it some processing time
    end
    
    -- roi target, do only if this mode is enabled, as ArduPilot will set mode
    if gimbal.set_roisysid_cnt > 0 then
        gimbal.set_roisysid_cnt = gimbal.set_roisysid_cnt - 1
        -- gimbalSetRoiSysId()
        return -- give it some processing time
    end    
    
    -- pitch control slider
    local pitch_cntrl = getValue(config_g.gimbalPitchSlider)
    if pitch_cntrl ~= nil then 
        gimbal.pitch_cntrl_deg = -(pitch_cntrl+1008)/1008*45
        if gimbal.pitch_cntrl_deg > 0 then gimbal.pitch_cntrl_deg = 0 end
        if gimbal.pitch_cntrl_deg < -90 then gimbal.pitch_cntrl_deg = -90 end
    else    
        gimbal.pitch_cntrl_deg = 0
    end
    -- yaw control slider
    local yaw_cntrl = getValue(config_g.gimbalYawSlider)
    if yaw_cntrl ~= nil then 
        gimbal.yaw_cntrl_deg = yaw_cntrl/1008*75
        if gimbal.yaw_cntrl_deg > 75 then gimbal.yaw_cntrl_deg = 75 end
        if gimbal.yaw_cntrl_deg < -75 then gimbal.yaw_cntrl_deg = -75 end
    else    
        gimbal.yaw_cntrl_deg = 0
    end
    
    --we correct pitch to be in +-20° range
    if pitch_cntrl ~= nil then 
        gimbal.pitch_cntrl_deg = -pitch_cntrl/1008*20
    end
    
    -- do this always, since otherwise we wouldn't get the flags to the gimbal manger & gimbal
    -- ArduPilot is weired
    -- modified in BetaPilot to not set the angles/rate if not mavlink targeting
    gimbalSetPitchYawDeg(gimbal.pitch_cntrl_deg, gimbal.yaw_cntrl_deg)
end


local function doPageGimbal()
    if drawNoGimbal() then return end
    local x = 0
    local y = 0
    
    -- MENU HANDLING
    if gimbal.is_in_control then
        tmenu.handle(gimbal_menu)
    else    
        tbutton.handle(gimbal_claim_control_button)
    end     
    
    -- DISPLAY
    local info = mavsdk.gimbalGetInfo()
    local compid = info.compid
    local gimbalStr = string.format("%s %d", string.upper(getGimbalIdStr(compid)), compid)
    lcd.setColor(CUSTOM_COLOR, p.WHITE)
    lcd.drawText(1, 20, gimbalStr, CUSTOM_COLOR)
if not gimbal.debug then
    local modelStr = info.model_name
    lcd.drawText(1, 35, modelStr, CUSTOM_COLOR)
    local versionStr = info.firmware_version
    lcd.drawText(1, 50, versionStr, CUSTOM_COLOR)
end    
    
if gimbal.debug then
    local modelStr = info.model_name
    lcd.drawText(LCD_W-1, 20, modelStr, CUSTOM_COLOR+RIGHT)
    local vendorStr = info.vendor_name
    lcd.drawText(LCD_W-1, 35, vendorStr, CUSTOM_COLOR+RIGHT)
    local versionStr = info.firmware_version
    lcd.drawText(LCD_W-1, 50, versionStr, CUSTOM_COLOR+RIGHT)
--    local customStr = info.custom_name
--    lcd.drawText(LCD_W-1, 65, customStr, CUSTOM_COLOR+RIGHT)
if config_g.gimbalUseGimbalManager then
    if mavsdk.gimbalClientIsInitialized() then
        lcd.drawText(1, 35, "client initialized", CUSTOM_COLOR)
    elseif mavsdk.gimbalClientIsReceiving() then
        lcd.drawText(1, 35, "client is receiving", CUSTOM_COLOR)
    else
        lcd.drawText(1, 35, "client is waiting...", CUSTOM_COLOR)
    end    
    local clientInfo = mavsdk.gimbalClientGetInfo()
    if clientInfo.gimbal_manager_id > 0 then
        local s = string.format("manager %d", clientInfo.gimbal_manager_id)
        lcd.drawText(1, 50, s, CUSTOM_COLOR)
    end
    --local my_sysid, my_compid = mavlink.getMyIds()
    --local s = string.format("me %d %d", my_sysid, my_compid)
    --lcd.drawText(1, 65, s, CUSTOM_COLOR)
    
    y = 208
    local gm_status = mavsdk.gimbalManagerGetStatus()
    lcd.setColor(CUSTOM_COLOR, p.WHITE)
    lcd.drawText(1, y, "1st:", CUSTOM_COLOR)
      lcd.drawNumber(45, y, gm_status.primary_sysid, CUSTOM_COLOR)
      lcd.drawNumber(82, y, gm_status.primary_compid, CUSTOM_COLOR)
    y = y + 15
    lcd.drawText(1, y, "2nd:", CUSTOM_COLOR)
      lcd.drawNumber(45, y, gm_status.secondary_sysid, CUSTOM_COLOR)
      lcd.drawNumber(82, y, gm_status.secondary_compid, CUSTOM_COLOR)
    y = y + 15
    lcd.drawText(1, y, "RC:", CUSTOM_COLOR)
    if bit32.btest(gm_status.device_flags, mavsdk.GDFLAGS_RC_MIXED) then
        lcd.drawText(45, y, "mix", CUSTOM_COLOR)
    elseif bit32.btest(gm_status.device_flags, mavsdk.GDFLAGS_RC_EXCLUSIVE) then
        lcd.drawText(45, y, "excl", CUSTOM_COLOR)
    else    
        lcd.drawText(45, y, "-", CUSTOM_COLOR)
    end
    
    mount_status = mavlink.getMessage(mavlink.M_MOUNT_STATUS, mavlink.getAutopilotIds());
    if mount_status ~= nil then
        y = y + 15
        lcd.drawText(1, y, "M:", CUSTOM_COLOR)
        lcd.drawNumber(45, y, mount_status.mount_mode, CUSTOM_COLOR)
    end  
end    
end
    
    local is_armed = mavsdk.gimbalGetStatus().is_armed
    local prearm_ok = mavsdk.gimbalGetStatus().prearm_ok
    if is_armed then 
        lcd.setColor(CUSTOM_COLOR, p.GREEN)
        lcd.drawText(LCD_XMID, 20-4, "ARMED", CUSTOM_COLOR+DBLSIZE+CENTER)
    elseif prearm_ok then     
        lcd.setColor(CUSTOM_COLOR, p.YELLOW)
        lcd.drawText(LCD_XMID, 20, "Prearm Checks Ok", CUSTOM_COLOR+MIDSIZE+CENTER)
    else  
        lcd.setColor(CUSTOM_COLOR, p.YELLOW)
        lcd.drawText(LCD_XMID, 20, "Initializing", CUSTOM_COLOR+MIDSIZE+CENTER)
    end
    
    x = 10
    y = 95
    local pitch = mavsdk.gimbalGetAttPitchDeg()
    local roll = mavsdk.gimbalGetAttRollDeg()
    local yaw = mavsdk.gimbalGetAttYawDeg()
    lcd.setColor(CUSTOM_COLOR, p.WHITE)
    lcd.drawText(x, y, "Pitch:", CUSTOM_COLOR+MIDSIZE)
    lcd.drawNumber(x+80, y, pitch*100, CUSTOM_COLOR+MIDSIZE+PREC2)
    lcd.drawText(x, y+35, "Roll:", CUSTOM_COLOR+MIDSIZE)
    lcd.drawNumber(x+80, y+35, roll*100, CUSTOM_COLOR+MIDSIZE+PREC2)
    lcd.drawText(x, y+70, "Yaw:", CUSTOM_COLOR+MIDSIZE)
    lcd.drawNumber(x+80, y + 70, yaw*100, CUSTOM_COLOR+MIDSIZE+PREC2)

    x = 220
    y = y + 15
    local r = 80
    local sqrtr = 56
    lcd.setColor(CUSTOM_COLOR, p.YELLOW)
    lcd.drawCircleQuarter(x, y, r, 4, CUSTOM_COLOR)    
    lcd.drawLine(x+r-10, y, x+r, y, SOLID, CUSTOM_COLOR)
    lcd.drawLine(x+sqrtr-6, y+sqrtr-6, x+sqrtr, y+sqrtr, SOLID, CUSTOM_COLOR)
    lcd.drawLine(x, y+r-10, x, y+r, SOLID, CUSTOM_COLOR)
    
    lcd.setColor(CUSTOM_COLOR, p.WHITE)
    if gimbal.pitch_cntrl_deg ~= nil then
        local cangle = gimbal.pitch_cntrl_deg
        lcd.drawCircle(x + (r-10)*math.cos(math.rad(cangle)), y - (r-10)*math.sin(math.rad(cangle)), 7, CUSTOM_COLOR)
    end    
    if gimbal.pitch_cntrl_deg ~= nil then
        lcd.drawNumber(400, y, gimbal.pitch_cntrl_deg, CUSTOM_COLOR+XXLSIZE+CENTER)
    end    
    if gimbal.yaw_cntrl_deg ~= nil and config_g.gimbalYawSlider ~= "" then
        lcd.drawNumber(400, y+60, gimbal.yaw_cntrl_deg, CUSTOM_COLOR+CENTER)
    end    
    
    lcd.setColor(CUSTOM_COLOR, p.RED)
    local gangle = pitch
    if gangle > 20 then gangle = 20 end
    if gangle < -100 then gangle = -100 end
    lcd.drawFilledCircle(x + (r-10)*math.cos(math.rad(gangle)), y - (r-10)*math.sin(math.rad(gangle)), 5, CUSTOM_COLOR)

    -- MENU DISPLAY
    if gimbal.is_in_control then
        tmenu.draw(gimbal_menu)
    else    
        drawNoControl()
        tbutton.draw(gimbal_claim_control_button)
    end    
end  


return 
    gimbalDoAlways,
    doPageGimbal

