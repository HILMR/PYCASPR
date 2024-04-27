"""
Test Inverse Dynamics of PYCASPR
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
# Load the trajectory
q,q_dot,q_ddot,tv=pycaspr.Help_traj('traj_processor_test')
# Init the solver
pycaspr.ID_init(method_id=1)
# Loop
forces_list=[]
t_st=time.time()
for t in range(len(tv)):
    # Wrapper API
    forces,cablelengths,cableLengthsDot=pycaspr.ID_update(q[t,:],q_dot[t,:],q_ddot[t,:])
    forces_list.append(forces)
t_ed=time.time()
print('Time consuming: ',(t_ed-t_st)/len(tv))
# Compare results
plt.plot(np.array(forces_list))
with open('PYCASPR_test/cable_forces.csv',encoding = 'utf-8') as f: 
    data_forces = np.loadtxt(f,float,delimiter = ",")
plt.figure()
plt.plot(data_forces.T)
plt.show()