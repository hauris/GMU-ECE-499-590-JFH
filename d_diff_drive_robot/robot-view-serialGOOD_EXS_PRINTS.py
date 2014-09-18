#!/usr/bin/env python
# /* -*-  indent-tabs-mode:t; tab-width: 8; c-basic-offset: 8  -*- */
# /*
# Copyright (c) 2014, Daniel M. Lofaro <dan (at) danLofaro (dot) com>
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the author nor the names of its contributors may
#       be used to endorse or promote products derived from this software
#       without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
# PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# */
import diff_drive
import ach
import sys
import time
from ctypes import *
import socket
import cv2.cv as cv
import cv2
import numpy as np

import actuator_sim as ser
#-----------------------------------------------------
#--------[ Do not edit above ]------------------------
#-----------------------------------------------------

# Add imports here
import CalcChkSumFile as ccs
import PktMkrFile as pmf
#import matplotlib.pyplot as plt
tsimList = []

#-----------------------------------------------------
#--------[ Do not edit below ]------------------------
#-----------------------------------------------------
dd = diff_drive
ref = dd.H_REF()
tim = dd.H_TIME()

ROBOT_DIFF_DRIVE_CHAN   = 'robot-diff-drive'
ROBOT_CHAN_VIEW   = 'robot-vid-chan'
ROBOT_TIME_CHAN  = 'robot-time'
# CV setup 
r = ach.Channel(ROBOT_DIFF_DRIVE_CHAN)
r.flush()
t = ach.Channel(ROBOT_TIME_CHAN)
t.flush()

i=0


print '======================================'
print '============= Robot-View ============='
print '========== Daniel M. Lofaro =========='
print '========= dan@danLofaro.com =========='
print '======================================'
ref.ref[0] = 0
ref.ref[1] = 0
while True:
    [status, framesize] = t.get(tim, wait=False, last=True)
    if status == ach.ACH_OK or status == ach.ACH_MISSED_FRAME or status == ach.ACH_STALE_FRAMES:
        pass
        #print 'Sim Time = ', tim.sim[0]
    else:
        raise ach.AchException( v.result_string(status) )

#-----------------------------------------------------
#--------[ Do not edit above ]------------------------
#-----------------------------------------------------
    # Main Loop
    # Def:
    # tim.sim[0] = Sim Time

    fs = 20.0   # Hertz
    tsim = tim.sim[0]
    print 'Sim Time = ', tim.sim[0]
    
    tn = tsim + (1.0/fs)
    
    print "BEFORE: tsim = " + repr(tsim) + " " + repr(tn) + " = tn"
    while tsim < tn:
      t.get(tim)
      tsim = tim.sim[0]
    print "AFTER: tsim = " + repr(tsim) + " " + repr(tn) + " = tn"
    #
    tsimList.append(tsim)
    if len(tsimList) > 1000:
      break
    print "############################################################"

    # Sleeps
    #time.sleep(0.1)
    #time.sleep(0.001)   
    buff = [0,0,0,0,0,0,0,0]
    buff = [255,255,3,40,67,24,0,0]
    chksum = ccs.CalcChkSum(buff)
    print "chksum !!!!!!!!!!!!!!!!!!!!!!!!!!!!! = ", chksum
    #bfr = pmf.PktMkr(241)
    #bfr = pmf.PktMkr(65535)
    bfr = pmf.PktMkr(512+241)
    print "Pkt Bfr = ", bfr
    quit()
    #plt.plot([1, 2, 3, 4],[2, 4, 6, 8])
    #quit()
    ref = ser.serial_sim(r,ref,buff)

print "Length tsimList = ", len(tsimList)
f = open("Assign1Q5.txt", "w")
for tsimE in tsimList:
  f.write( str(tsimE)  + "\n"  )      # str() converts to string
f.close()
print "Done writing:"   
#-----------------------------------------------------
#--------[ Do not edit below ]------------------------
#-----------------------------------------------------
