----------------------------------------------------------------------
-- OlliW Telemetry Widget Script
-- (c) www.olliw.eu, OlliW, OlliW42
-- licence: GPL 3.0
-- https://www.gnu.org/licenses/gpl-3.0.de.html
----------------------------------------------------------------------
-- General Utility Functions
----------------------------------------------------------------------


local utils = {}


----------------------------------------------------------------------
-- General 
----------------------------------------------------------------------

function utils:TimeToStr(time_s)
    local hours = math.floor(time_s/3600)
    local mins = math.floor(time_s/60 - hours*60)
    local secs = math.floor(time_s - hours*3600 - mins *60)
    --return string.format("%02d:%02d:%02d", hours, mins, secs)
    return string.format("%01d:%02d:%02d", hours, mins, secs)
end


function utils:LatLonToDms(dec,lat)
    -- this code part is from Yaapu FrSky Telemetry Script, thx!
    local D = math.floor(math.abs(dec))
    local M = math.floor((math.abs(dec) - D)*60)
    local S = (math.abs((math.abs(dec) - D)*60) - M)*60
	  return D .. string.format("\64%d'%04.1f", M, S) .. (lat and (dec >= 0 and "E" or "W") or (dec >= 0 and "N" or "S"))
end


local function dpos_to_m(dposint)
    -- flat earth math
    -- y = rad[(lon-lon0) * 1e-7] * R = pi/180 * 1e-7 * 6.371e6 * (lon-lon0) 
    return math.rad(0.6371) * dposint
end    

-- pos1 in respect to pos0
function utils:posDistance(lat1,lon1,lat0,lon0)
    -- flat earth
    local xScale = math.cos(math.rad((lat1+lat0) * 1.0e-7) * 0.5)
    local x = dpos_to_m(lon1 - lon0) * xScale
    local y = dpos_to_m(lat1 - lat0)
    return math.sqrt(x*x + y*y) 
end  

function utils:posAngle(lat1,lon1,lat0,lon0)
    -- flat earth
    local xScale = math.cos(math.rad((lat1+lat0) * 1.0e-7) * 0.5)
    local x = dpos_to_m(lon1 - lon0) * xScale
    local y = dpos_to_m(lat1 - lat0)
    local phi = math.deg(math.atan2(x,y))
    if phi < 0.0 then phi = phi + 360.0 end
    return phi
end  

function utils:posDistanceAngle(lat1,lon1,lat0,lon0)
    -- flat earth
    local xScale = math.cos(math.rad((lat1+lat0) * 1.0e-7) * 0.5)
    local x = dpos_to_m(lon1 - lon0) * xScale
    local y = dpos_to_m(lat1 - lat0)
    local phi = math.deg(math.atan2(x,y))
    if phi < 0.0 then phi = phi + 360.0 end
    return math.sqrt(x*x + y*y), phi
end  

function utils:posXY(lat1,lon1,lat0,lon0)
    -- flat earth
    local xScale = math.cos(math.rad((lat1+lat0) * 1.0e-7) * 0.5)
    local x = dpos_to_m(lon1 - lon0) * xScale
    local y = dpos_to_m(lat1 - lat0)
    return x,y
end  


----------------------------------------------------------------------
-- MavSDK Extensions
----------------------------------------------------------------------

function utils:getVehicleClassStr()
    local vc = mavsdk.getVehicleClass();
    if vc == mavsdk.VEHICLECLASS_COPTER then
        return "COPTER"
    elseif vc == mavsdk.VEHICLECLASS_PLANE then    
        return "PLANE"
    end    
    return nil
end    


function utils:isCopter()
    return mavsdk.getVehicleClass() == mavsdk.VEHICLECLASS_COPTER
end    


function utils:isPlane()
    return mavsdk.getVehicleClass() == mavsdk.VEHICLECLASS_PLANE
end    


function utils:getGimbalIdStr(compid)
    if compid == mavlink.COMP_ID_GIMBAL then
        return "Gimbal1"
    elseif compid >= mavlink.COMP_ID_GIMBAL2 and compid <= mavlink.COMP_ID_GIMBAL6 then
        return "Gimbal"..tostring(compid - mavlink.COMP_ID_GIMBAL2 + 2)
    end
    return "Gimbal"
end    


function utils:getCameraIdStr(compid)
    if compid >= mavlink.COMP_ID_CAMERA and compid <= mavlink.COMP_ID_CAMERA6 then
        return "Camera"..tostring(compid - mavlink.COMP_ID_CAMERA + 1)
    end
    return "Camera"
end


local gpsFixes = {}
gpsFixes[0]  = "No GPS"
gpsFixes[1]  = "No FIX"
gpsFixes[2]  = "2D FIX"
gpsFixes[3]  = "3D FIX"
gpsFixes[4]  = "DGPS"
gpsFixes[5]  = "RTK Float"
gpsFixes[6]  = "RTK Fixed"
gpsFixes[7]  = "Static"
gpsFixes[8]  = "PPP"


function utils:getGpsFixStr(gpsId)
    if gpsId == 1 then return gpsFixes[mavsdk.getGpsFix()] end
    if gpsId == 2 then return gpsFixes[mavsdk.getGps2Fix()] end
end    

 
return utils
