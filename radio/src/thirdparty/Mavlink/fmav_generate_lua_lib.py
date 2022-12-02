#!/usr/bin/env python
'''
fmav_generate_lua_lib.py
calls fastMavlink generator modules
(c) OlliW, OlliW42, www.olliw.eu
'''
import os, sys

#options to set

mavlinkpathtorepository = os.path.join('fastmavlink')

mavlinkdialect = "opentx.xml"

'''
Imports
'''
sys.path.insert(0, mavlinkpathtorepository)

from generator.modules import fmav_parse as mavparse
from generator.modules import mavtemplate
from generator.modules import fmav_flags as mavflags

'''
Attention: names must not be longer than 32 chars
'''

print_warnings = True
print_verbose = True

def printWarning(txt, warn=True):
    if not print_warnings: return
    if not warn: return
    print(txt)

def printVerbose(txt, verbose=True):
    if not print_verbose: return
    if not verbose: return
    print(txt)

def excludeMessage(msg):
    #return True
    return False
    if msg.name in ['AHRS','ATTITUDE','VIBRATION']: return False
    #if msg.name in ['AHRS']: return False
    return True

def cvtInvalidAttr(invalid_str):
    if invalid_str == 'NaN':
        return 'NAN'
    else:
        return invalid_str

def shortenName(name, width):
    nameshort = name[:]
    if len(nameshort) > width:
        printWarning('WARNING: msg id '+name+' too long')
        nn = str.split(nameshort, '_')
        nameshort = nn[0]+'_'
        for i in range(1,len(nn)-1):
            nameshort += nn[i][:3] + '_'
        nameshort += nn[-1]
        if len(nameshort) > width:
            printWarning(' ! ! !   msg id too long even after shortening')
            nameshort = nameshort[:width]
    return nameshort 

def shortenNameEnum(name, width):
    nameshort = name[:]
    if nameshort[:4] == 'MAV_': nameshort = nameshort[4:]
    
    if len(nameshort) > width:
        printWarning('WARNING: enum field '+name+' too long')
        nn = str.split(nameshort, '_')
        nameshort = nn[0]+'_'
        for i in range(1,len(nn)-1):
            nameshort += nn[i][:3] + '_'
        nameshort += nn[-1]
        if len(nameshort) > width:
            printWarning(' ! ! !   enum field too long even after shortening')
            nameshort = nameshort[:width]
    
    return nameshort 

def msgFieldCount(msg, excludetargets):
    count = 0
    for field in msg.fields:
        if field.array_length == 0:
            if excludetargets and msg.is_target_system_field(field): continue
            if excludetargets and msg.is_target_component_field(field): continue
            count += 1
        else:    
            count += 1
    return count        

def generateLibMessageForPush(msg, m):
    m.append('  case FASTMAVLINK_MSG_ID_'+msg.name+': { // #'+str(msg.id))
    if msgFieldCount(msg,True) > 0:
        m.append('    fmav_'+msg.name.lower()+'_t* payload = (fmav_'+msg.name.lower()+'_t*)(mavmsg->payload_ptr);')
    #for field in msg.ordered_fields:
    for field in msg.fields:
        nameshort = shortenName(field.name, 32)
        if field.array_length == 0:
            #we strip of target fields as their values are already in the msg structure
            if msg.is_target_system_field(field): continue
            if msg.is_target_component_field(field): continue
            m.append('    lua_pushtablenumber(L, "'+nameshort+'", payload->'+field.name+');')
        else:
            m.append('    lua_pushstring(L, "'+nameshort+'"); // array '+field.name+'['+str(field.array_length)+']' )
            m.append('    lua_newtable(L);')
            m.append('    for (int i = 0; i < '+str(field.array_length)+'; i++) { ')
            m.append('      lua_pushtableinumber(L, i+1, payload->'+field.name+'[i]); // lua is 1 indexed') 
            m.append('    }')
            m.append('    lua_rawset(L, -3);')
    m.append('    return;')
    m.append('    }')


def generateLibMessageForCheck(msg, m):
    m.append('  case FASTMAVLINK_MSG_ID_'+msg.name+': { // #'+str(msg.id))
    if msgFieldCount(msg,False) > 0:
        m.append('    fmav_'+msg.name.lower()+'_t* payload = (fmav_'+msg.name.lower()+'_t*)(msg_out->payload);')
    for field in msg.fields:
        nameshort = shortenName(field.name, 32)
        if field.array_length == 0:
            #we substitute target fields
            if msg.is_target_system_field(field): nameshort = 'target_sysid'
            if msg.is_target_component_field(field):  nameshort = 'target_compid'
            if msg.name == 'HEARTBEAT' and nameshort == 'mavlink_version':
                m.append('    payload->mavlink_version = FASTMAVLINK_MAVLINK_VERSION;')
                continue
            invalid = '0'
            if field.invalid:
                invalid = cvtInvalidAttr(field.invalid)
            m.append('    lua_checktablenumber(L, payload->'+field.name+', "'+nameshort+'", '+invalid+');')
        else:
            m.append('    lua_pushstring(L, "'+nameshort+'"); // array '+field.name+'['+str(field.array_length)+']' )
            m.append('    lua_gettable(L, -2);')
            invalid = '[0]'
            if field.invalid:
                invalid = cvtInvalidAttr(field.invalid)
            invalid = invalid[1:-1] #strip off []
            if invalid.find(',') < 0 and invalid.find(':') < 0:
                #is [value]
                invalid = cvtInvalidAttr(invalid)
                m.append('    if (lua_istable(L,-1)) {')
                m.append('      for (int i = 0; i < '+str(field.array_length)+'; i++) { ')
                m.append('        lua_checktableinumber(L, payload->'+field.name+'[i], i+1, '+invalid+'); // lua is 1 indexed')
                m.append('      }')
                m.append('    }')
            else:
                #we do the same for both [value:] and [value,], [value,,,] etc are not supported yet, so let's drop an error
                if invalid.count(',') > 1:
                    print('ERROR: invalid attributes of format [value,,,] are not supported!')
                    exit()
                invalid = cvtInvalidAttr(invalid[:-1])
                m.append('    if (lua_istable(L,-1)) {')
                m.append('      lua_checktableinumber(L, payload->'+field.name+'[0], 1, '+invalid+'); // lua is 1 indexed')
                if field.array_length > 0:
                    m.append('      for (int i = 1; i < '+str(field.array_length)+'; i++) { ')
                    m.append('        lua_checktableinumber(L, payload->'+field.name+'[i], i+1, '+'0'+'); // lua is 1 indexed')
                    m.append('      }')
                m.append('    }')
            m.append('    lua_pop(L, 1);')
            
    for field in msg.fields:
        if msg.is_target_system_field(field): 
            m.append('    msg_out->target_sysid = payload->'+field.name+';')
    for field in msg.fields:
        if msg.is_target_component_field(field): 
            m.append('    msg_out->target_compid = payload->'+field.name+';')
    m.append('    msg_out->crc_extra = FASTMAVLINK_MSG_'+msg.name+'_CRCEXTRA;')
    m.append('    msg_out->payload_max_len = FASTMAVLINK_MSG_'+msg.name+'_PAYLOAD_LEN_MAX;')
    m.append('    return 1;')
    m.append('    }')


def generateLuaLibHeaders(dialectname):
    print("Run XML %s" % os.path.basename(dialectname))
    dialectnamewoext = os.path.splitext(os.path.basename(dialectname))[0]
    xml_list = mavparse.generateXmlList(dialectname)
    messagesoutname = os.path.splitext(os.path.basename(dialectname))[0] + '_lua_lib_messages.h'
    constantsoutname = os.path.splitext(os.path.basename(dialectname))[0] + '_lua_lib_constants.h'

    for xml in xml_list:
        printVerbose(xml.basename)
    print("Found %u messages and %u enums in %u XML files" %
        (mavparse.totalNumberOfMessages(xml_list), mavparse.totalNumberOfEnums(xml_list), len(xml_list)))

    # find dialectxml, it should be actually the last/highest in the list, so could get it easier
    dialectxml = None
    for xml in xml_list:
        if xml.basename == dialectnamewoext:
            dialectxml = xml
        
    # generate complete enum dictionary     
    # dialects may not have enums and hence empty enums_merged
    # so go through the xml list, and take them all
    # if an enum.name already exist, overwrite it with the new one, i.e., always take the last/highest
    enums_all_by_name = {}
    for xml in xml_list:
        for enum in xml.enums_merged:
            enums_all_by_name[enum.name] = enum

    printVerbose("Messages")
    printVerbose('-> '+dialectxml.basename)
    m = []
    m.append('''//------------------------------------------------------------
// mavlink messages
//------------------------------------------------------------
// all message from opentx.xml
// auto generated


//------------------------------------------------------------
// push
//------------------------------------------------------------

#define lua_pushtableinteger_raw(L, k, v)  (lua_pushstring(L,(k)), lua_pushinteger(L,(v)), lua_rawset(L,-3))
#define lua_pushtablenumber_raw(L, k, v)   (lua_pushstring(L,(k)), lua_pushnumber(L,(v)), lua_rawset(L,-3))
#define lua_pushtablestring_raw(L, k, v)   (lua_pushstring(L,(k)), lua_pushstring(L,(v)), lua_rawset(L,-3))
#define lua_pushtableinumber_raw(L, i, v)  (lua_pushnumber(L,(i)), lua_pushnumber(L,(v)), lua_rawset(L,-3))
#define lua_pushtableinumber(L, i, v)      (lua_pushnumber(L,(i)), lua_pushnumber(L,(v)), lua_settable(L,-3))

static void luaMavlinkPushMavMsg(lua_State *L, MavlinkTelem::MavMsg* mavmsg)
{
  lua_newtable(L);
  lua_pushtableinteger(L, "sysid", mavmsg->sysid);
  lua_pushtableinteger(L, "compid", mavmsg->compid);
  lua_pushtableinteger(L, "msgid", mavmsg->msgid);
  lua_pushtableinteger(L, "target_sysid", mavmsg->target_sysid);
  lua_pushtableinteger(L, "target_compid", mavmsg->target_compid);
  lua_pushtableboolean(L, "updated", mavmsg->updated);

  switch (mavmsg->msgid) {''')
    for msgid in sorted(dialectxml.messages_all_by_id.keys()):
        msg = dialectxml.messages_all_by_id[msgid]
        if excludeMessage(msg): continue
        generateLibMessageForPush(msg, m)
    m.append('''  }
}


//------------------------------------------------------------
// check
//------------------------------------------------------------

#define lua_checktableinteger(L, r, k, d)  (lua_pushstring(L,(k)), lua_gettable(L,-2 ), r = (lua_isnumber(L,-1))?lua_tointeger(L,-1):(d), lua_pop(L,1))
#define lua_checktablenumber(L, r, k, d)   (lua_pushstring(L,(k)), lua_gettable(L,-2 ), r = (lua_isnumber(L,-1))?lua_tonumber(L,-1):(d), lua_pop(L,1))
#define lua_checktableinumber(L, r, i, d)  (lua_pushnumber(L,(i)), lua_gettable(L,-2 ), r = (lua_isnumber(L,-1))?lua_tonumber(L,-1):(d), lua_pop(L,1))

static uint8_t luaMavlinkCheckMsgOut(lua_State *L, fmav_message_t* msg_out)
{
  uint32_t msgid;
  lua_checktableinteger(L, msgid, "msgid", UINT32_MAX);
  if (msgid == UINT32_MAX) return 0;
  
  msg_out->sysid = mavlinkTelem.mySysId();
  msg_out->compid = mavlinkTelem.myCompId();
  msg_out->msgid = msgid;
  msg_out->target_sysid = 0;
  msg_out->target_compid = 0;
  
  switch (msgid) {''')  
    for msgid in sorted(dialectxml.messages_all_by_id.keys()):
        msg = dialectxml.messages_all_by_id[msgid]
        if excludeMessage(msg): continue
        generateLibMessageForCheck(msg, m)
    m.append('''  }
  return 0;
}
''')

    # constants
    printVerbose("Constants")
    printVerbose('-> '+dialectxml.basename)
    s = []
    s.append('''//------------------------------------------------------------
// mavlink constants
//------------------------------------------------------------
// all message and enum constants from opentx.xml
// auto generated

#define MAVLINK_LIB_CONSTANTS \\''')
    for msgid in sorted(dialectxml.messages_all_by_id.keys()):
        name = dialectxml.messages_all_by_id[msgid].name
        nameshort = shortenName(name, 30)
        s.append('  { "M_'+nameshort+'", FASTMAVLINK_MSG_ID_'+name+' }, \\')

    for enumname in enums_all_by_name:
        s.append( '  \\')
        for entry in enums_all_by_name[enumname].entries[:-1]:
            nameshort = shortenNameEnum(entry.name, 30)
            s.append('  { "'+nameshort+'", '+str(entry.value)+' }, \\')

    F = open(constantsoutname, mode='w')
    for ss in s:
        F.write(ss)
        F.write('\n')
    F.write('\n')
    F.close()

    F = open(messagesoutname, mode='w')
    for mm in m:
        F.write(mm)
        F.write('\n')
    F.write('\n')
    F.close()


if __name__ == "__main__":
    dialectname = os.path.join(mavlinkdialect)

    from argparse import ArgumentParser as AP
    p = AP(description="Generate fastMAVLink Lua C code for OpenTx from a XML message definition file.")
    p.add_argument("-nw", "--nowarnings", dest="no_warnings", action='store_true',
                   default = False, 
                   help = "Do not print out warnings [default: %(default)s]")
    p.add_argument("-nv", "--noverbose", dest="no_verbose", action='store_true',
                   default = False, 
                   help = "Do not print out verbose [default: %(default)s]")
    opts = p.parse_args()
    if opts.no_warnings:
        print_warnings = False
    if opts.no_verbose:
        print_verbose = False
    
    generateLuaLibHeaders(dialectname)
