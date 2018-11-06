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


class AutostepProxyException(Exception):
    pass


class AutostepProxy(object):
    
    TrajectoryDt = Autostep.TrajectoryDt
    BusyWaitDt = 0.1

    def __init__(self, namespace='autostep'):
        self.namespace = namespace
        service_name = '/{}/command'.format(self.namespace)
        self.rospy.wait_for_service(service_name)
        self.command_proxy = rospy.ServiceProxy(service_name,Command)

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

    def run(self,velocity):
        command_name = 'run'
        command_args = {'velocity': velocity}
        rsp_dict = self.send_command(command_name, command_args)
        self.check_rsp_dict(rsp_dict)

    def enable(self):
        command_name = 'enable'
        command_args = None
        rsp_dict = self.send_command(command_name, command_args)
        self.check_rsp_dict(rsp_dict)

    def release(self):
        command_name = 'release'
        command_args = None
        rsp_dict = self.send_command(command_name, command_args)
        self.check_rsp_dict(rsp_dict)

    def is_busy(self):
        command_name = 'is_busy'
        command_args = None
        rsp_dict = self.send_command(command_name, command_args)
        self.check_rsp_dict(rsp_dict)
        return rsp_dict['is_busy']

    def busy_wait(self):
        while self.is_busy():
            time.sleep(self.BusyWaitDt)

    def move_to(self,position):
        command_name = 'move_to'
        command_args = {'position': position}
        rsp_dict = self.send_command(command_name, command_args)
        self.check_rsp_dict(rsp_dict)

    def move_by(self,step):
        command_name = 'move_by'
        command_args = {'step': step}
        rsp_dict = self.send_command(command_name, command_args)

    def soft_stop(self):
        command_name = 'soft_stop'
        command_args = None
        rsp_dict = self.send_command(command_name, command_args)
        self.check_rsp_dict(rsp_dict)

    def set_position(self,position):
        command_name = 'set_position'
        command_args = {'position': position}
        rsp_dict = self.send_command(command_name, command_args)
        self.check_rsp_dict(rsp_dict)

    def get_position(self):
        command_name = 'get_position'
        command_args = None
        rsp_dict = self.send_command(command_name, command_args)
        self.check_rsp_dict(rsp_dict)
        return rsp_dict['position']

    def set_move_mode(self,mode):
        command_name = 'set_move_mode'
        command_args = {'mode': mode} 
        rsp_dict = self.send_command(command_name, command_args)
        self.check_rsp_dict(rsp_dict)

    def get_jog_mode_params(self):
        pass

    def set_jog_mode_params(self,params):
        pass

    def get_max_mode_params(self):
        pass

    def set_jog_mode_params(self,params):
        pass

    def get_params(self):
        command_name = 'get_params'
        command_args = None
        rsp_dict = self.send_command(command_name, command_args)
        self.check_rsp_dict(rsp_dict)
        return rsp_dict['params']

    def print_params(self):
        params = self.get_params()
        print()
        print('params')
        print('------')
        for k,v in params.iteritems():
            print('{} = {}'.format(k,v))
        print()

    def sinusoid(self,param):
        command_name = 'sinusoid'
        command_args = param
        rsp_dict = self.send_command(command_name, command_args)
        self.check_rsp_dict(rsp_dict)

    def move_to_sinusoid_start(self,param):
        command_name = 'move_to_sinusoid_start'
        command_args = param
        rsp_dict = self.send_command(command_name, command_args)
        self.check_rsp_dict(rsp_dict)

    def run_trajectory(self,position):
        command_name = 'run_trajectory'
        command_args = {'position': position}
        rsp_dict = self.send_command(command_name, command_args)
        self.check_rsp_dict(rsp_dict)

    def enable_tracking_mode(self):
        command_name = 'enable_tracking_mode'
        command_args = None
        rsp_dict = self.send_command(command_name, command_args)
        self.check_rsp_dict(rsp_dict)

    def disable_tracking_mode(self):
        command_name = 'disable_tracking_mode'
        command_args = None
        rsp_dict = self.send_command(command_name, command_args)
        self.check_rsp_dict(rsp_dict)
    
    def check_rsp_dict(self,rsp_dict):
        if not rsp_dict['success']:
            except_tuple = rsp_dict['success'], rsp_dict['message']
            raise AutostepProxyException('command failed: success = {}, message = {}'.format(except_tuple))

