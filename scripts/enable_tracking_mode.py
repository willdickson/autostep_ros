#!/usr/bin/env python
from __future__ import print_function
import sys
import time
import scipy
import rospy

import std_msgs.msg
from autostep_proxy import AutostepProxy
from autostep_ros.msg import TrackingData

autostep = AutostepProxy()

cmd = sys.argv[1]
if cmd.lower() == 'true':
    print('* testing enable/disable tracking mode')
    print('  enabling')
    autostep.set_position(0.0)
    autostep.enable_tracking_mode()

else:
    print('  disabling')
    autostep.disable_tracking_mode()

