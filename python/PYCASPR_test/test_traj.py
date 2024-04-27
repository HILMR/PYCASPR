"""
Test Trajectory Interpolation of PYCASPR
Author: Mingrui Luo
Version: 2024/04/26
"""
import sys 
sys.path.insert(0, sys.path[0]+"/../")
from PYCASPR_core.wrapper import *
import os,time
import numpy as np
import matplotlib.pyplot as plt
# Find the workspace, it's important!
wkspace=os.path.dirname(os.path.abspath(__file__))+'/../'
# Construct the Core
pycaspr=PYCASPR(wkspace,'Example spatial','basic_7_cables',sharemode=True)
# Set keypoints
values=np.sin(np.linspace(0,2*np.pi,50))
q_p=[]
for i in range(len(values)):
    q_p.append([values[i]*0.1,values[i]*0.2,values[i]*0.3,
    values[i]*0.01,values[i]*0.02,values[i]*0.03])
q_p=np.array(q_p)
# Interpolation
q,q_dot,q_ddot,tv=pycaspr.Help_traj(q_p=q_p,te=5,dt=0.1,method_id=2)
# Plot
plt.plot(tv,q)
plt.plot(np.linspace(0,5,q_p.shape[0]),q_p,'.')
plt.figure()
plt.plot(tv,q_dot)
plt.figure()
plt.plot(tv,q_ddot)
plt.show()