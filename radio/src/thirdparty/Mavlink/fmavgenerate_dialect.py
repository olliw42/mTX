#!/usr/bin/env python
'''
fmavgenerate_dialect.py
calls fastMavlink generator for C header files
(c) olliw, olliw42
'''
import os
import shutil
import re
import sys


#options to set

mavlinkpathtorepository = os.path.join('fastmavlink')

mavlinkdialect = "opentx.xml"

mavlinkoutputdirectory = 'out'


'''
Imports
'''

sys.path.insert(0,mavlinkpathtorepository)

from generator import fmavgen
from generator.modules import fmavflags

'''
Generates the header files and place them in the output directory.
'''

outdir = mavlinkoutputdirectory

xmfile = mavlinkdialect

#recreate out directory
print('----------')
print('kill out dir')
try:
    shutil.rmtree(outdir)
except:
    pass    
os.mkdir(outdir)
print('----------')

opts = fmavgen.Opts(outdir, parse_flags=fmavflags.PARSE_FLAGS_WARNING_ENUM_VALUE_MISSING)
args = [xmfile]
try:
    fmavgen.fmavgen(opts,args)
    print('Successfully Generated Headers', 'Headers generated successfully.')

except Exception as ex:
    exStr = str(ex)
    print('Error Generating Headers','{0!s}'.format(exStr))
    exit()

