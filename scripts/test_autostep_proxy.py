#!/usr/bin/env python
from __future__ import print_function
import sys
import time
import scipy
from autostep_proxy import AutostepProxy

autostep = AutostepProxy()

# -----------------------------------------------------------------------------
print()
print('testing run command')
autostep.set_move_mode('jog')
for vel in scipy.linspace(-200,200,11):
    print('vel: {}'.format(vel))
    autostep.run(vel)
    time.sleep(0.5)
autostep.run(0.0)
print()

# -----------------------------------------------------------------------------
print('testing enable/release commands')
dt = 2.0
print('releasing for  {} secs ...'.format(dt), end='')
sys.stdout.flush()
autostep.release()
time.sleep(dt)
print('done')
print('enabling')
autostep.enable()
print()

# ----------------------------------------------------------------------------
print('testing move_to command')
autostep.set_move_mode('jog')
pos_list = [0, 5, 10, 20, 45, 90, 180, 360, -360, 0]
for pos in pos_list:
    print('moving to pos = {}  '.format(pos), end ='')
    sys.stdout.flush()
    autostep.move_to(pos)
    while autostep.is_busy():
        time.sleep(0.1)
    print('done')
print()


# ----------------------------------------------------------------------------
print('testing move_by command')
autostep.set_move_mode('jog')
pos_list = [0, 1, 2, 3, 4, 5, 10, 20, 45, 90, 180, 360, 0]
for pos in pos_list:
    print('moving to pos = {}  '.format(pos), end ='')
    sys.stdout.flush()
    autostep.move_to(pos)
    while autostep.is_busy():
        time.sleep(0.1)
    print('done')
print('returning to zero')
autostep.move_to(0.0)
while autostep.is_busy():
    time.sleep(0.1)
print()

# -----------------------------------------------------------------------------
print('testing soft_stop command')
vel = 100.0
print('running at vel = {}'.format(vel))
autostep.run(100)
time.sleep(2.0)
print('execulte soft_stop')
autostep.soft_stop()
print('returning to zero')
autostep.move_to(0.0)
while autostep.is_busy():
    time.sleep(0.1)
print()


# -----------------------------------------------------------------------------


# -----------------------------------------------------------------------------
print('testing get_position command')
position = autostep.get_position()
print('position = {}'.format(position))


