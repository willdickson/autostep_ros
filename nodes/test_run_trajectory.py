#!/usr/bin/env python
from __future__ import print_function
import json
import roslib
import rospy

import scipy 
from autostep import Autostep

from autostep_ros.srv import Command

import matplotlib.pyplot as plt


dt = Autostep.TrajectoryDt

num_cycle = 2
period = 5.0

num_pts = int(period*num_cycle/dt)

t = dt*scipy.arange(num_pts)
p = 50*scipy.cos(2.0*scipy.pi*t/period) + 20*scipy.cos(4.0*scipy.pi*t/period)


command_srv = rospy.ServiceProxy('command',Command)

rsp = command_srv('run_trajectory', json.dumps({'position': p.tolist()}))
print(rsp)

#plt.plot(t,p)
#plt.show()





