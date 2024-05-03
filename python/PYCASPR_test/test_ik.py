"""
Test Inverse Kinematics of PYCASPR
Author: Mingrui Luo
Version: 2024/05/03
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
cablelengths_list=np.array(cablelengths_list)
plt.figure('Results')
plt.plot(cablelengths_list)
# Load the truth
with open('PYCASPR_test/cable_lengths.csv',encoding = 'utf-8') as f: 
    data_lengths = np.loadtxt(f,float,delimiter = ",")
plt.figure('Errors')
plt.plot(cablelengths_list-data_lengths.T) # Errors
plt.show()