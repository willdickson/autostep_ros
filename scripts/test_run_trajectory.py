#!/usr/bin/env python
from __future__ import print_function
import json
import roslib
import rospy
import scipy 
import time

from autostep import Autostep
from autostep_ros.srv import Command
import matplotlib.pyplot as plt


dt = Autostep.TrajectoryDt
num_cycle = 4
period = 3.0
num_pts = int(period*num_cycle/dt)
t = dt*scipy.arange(num_pts)
p = 80*scipy.cos(2.0*scipy.pi*t/period) - 20*scipy.cos(4.0*scipy.pi*t/period)

command_srv = rospy.ServiceProxy('/autostep/command',Command)
rsp = command_srv('run_trajectory', json.dumps({'position': p.tolist()}))
print(rsp)

while True:
    rsp = command_srv('is_busy', json.dumps(''))
    rsp_dict = json.loads(rsp.rval_json)
    print('is_busy = {}'.format(rsp_dict['is_busy']))
    if not rsp_dict['is_busy']:
        break
    time.sleep(0.1)








