"""
Test Inverse Kinematics of PYCASPR
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
pycaspr=PYCASPR(wkspace,'Example spatial','basic_7_cables',sharemode=False)
# Load the trajectory
q,q_dot,q_ddot,tv=pycaspr.Help_traj('traj_processor_test')
# Loop
cablelengths_list=[]
t_st=time.time()
for t in range(len(tv)):
    # Wrapper API
    cablelengths,cableLengthsDot,segments=pycaspr.IK_update(q[t,:],q_dot[t,:],q_ddot[t,:])
    cablelengths_list.append(cablelengths)
t_ed=time.time()
print('Time consuming: ',(t_ed-t_st)/len(tv))
# Plot
plt.plot(np.array(cablelengths_list))
plt.show()