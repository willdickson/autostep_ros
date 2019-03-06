from __future__ import print_function
import time
import scipy
from autostep_proxy import AutostepProxy

import matplotlib.pyplot as plt


# Create trajectory
dt = AutostepProxy.TrajectoryDt
num_cycle = 4
period = 3.0
num_pts = int(period*num_cycle/dt)
t = dt*scipy.arange(num_pts)
p = 80*scipy.cos(2.0*scipy.pi*t/period) - 50*scipy.cos(4.0*scipy.pi*t/period)

plt.plot(t,p)
plt.xlabel('t (sec)')
plt.ylabel('position')
plt.grid('on')
plt.show()

# Run Trajectory
autostep = AutostepProxy()
autostep.run_trajectory(p)
while autostep.is_busy():
    print('running')
    time.sleep(0.1)
print('done')









