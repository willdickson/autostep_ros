#!/usr/bin/env python
from __future__ import print_function
import json
import roslib
import rospy
import numpy
import time

from autostep import Autostep
from autostep_ros.srv import Command
import matplotlib.pyplot as plt


class AutostepProxy(object):

    def __init__(self, namespace='autostep'):
        self.namespace = namespace
        self.command_proxy = rospy.ServiceProxy('/{}/command'.format(self.namespace),Command)

    def send_command(self,command_name, command_args=None):
        if command_args is None:
            json_args = json.dumps('')
        else:
            command_args_tmp = {}
            for k,v in command_args.iteritems():
                if type(v) == numpy.ndarray:
                    command_args_tmp[k] = v.tolist()
                else:
                    command_args_tmp[k] = v
            json_args = json.dumps(command_args_tmp)
        rsp = self.command_proxy(command_name, json_args)
        rsp_dict = json.loads(rsp.rval_json)
        return rsp_dict

    def move_to(self,position):
        command_name = 'move_to'
        command_args = {'position': position}
        rsp_dict = self.send_command(command_name,command_args)
        print(rsp_dict)




#dt = Autostep.TrajectoryDt
#num_cycle = 4
#period = 3.0
#num_pts = int(period*num_cycle/dt)
#t = dt*scipy.arange(num_pts)
#p = 80*scipy.cos(2.0*scipy.pi*t/period) - 20*scipy.cos(4.0*scipy.pi*t/period)
#
#command_srv = rospy.ServiceProxy('/autostep/command',Command)
#rsp = command_srv('run_trajectory', json.dumps({'position': p.tolist()}))
#print(rsp)
#
#while True:
#    rsp = command_srv('is_busy', json.dumps(''))
#    rsp_dict = json.loads(rsp.rval_json)
#    print('is_busy = {}'.format(rsp_dict['is_busy']))
#    if not rsp_dict['is_busy']:
#        break
#    time.sleep(0.1)
