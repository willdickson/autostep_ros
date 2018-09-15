#!/usr/bin/env python
from __future__ import print_function
import threading
import roslib
import rospy
import std_msgs.msg
import sensor_msgs.msg

from autostep_ros.msg import TrackingData


class DynamicModel(object):

    def __init__(self):

        self.dt = 0.01

        # Constants
        self.mass = 3.0
        self.damping_coeff = 3.0
        self.spring_const = 10.0
        self.spring_zero = 0.0
        self.force_coeff = -800.0

        # Dyanmic state
        self.position = 0.0
        self.velocity = 0.0
        self.external_force = 0.0

        self.lock = threading.Lock()

        rospy.init_node('dynamic_model')
        self.joy_sub = rospy.Subscriber('joy', sensor_msgs.msg.Joy, self.on_joystick)
        self.tracking_data_pub = rospy.Publisher('tracking_data', TrackingData, queue_size=10) 

    def set_external_force(self,value):
        with self.lock:
            self.external_force = value

    def get_external_force(self):
        with self.lock:
            external_force = self.external_force
        return external_force


    def on_joystick(self,data):
        self.set_external_force(self.force_coeff*data.axes[0])

    def run(self):
        while not rospy.is_shutdown():
            damping_force = -self.damping_coeff*self.velocity
            spring_force = self.spring_const*(self.spring_zero - self.position)
            total_force = self.get_external_force() + damping_force + spring_force
            self.velocity += self.dt*(1.0/self.mass)*total_force
            self.position += self.dt*self.velocity

            header = std_msgs.msg.Header()
            header.stamp = rospy.Time.now()
            self.tracking_data_pub.publish(TrackingData(header,self.position,self.velocity))
            rospy.sleep(self.dt)


# ---------------------------------------------------------------------------------------
if __name__ == '__main__':

    node = DynamicModel()
    node.run()
