"""
Test Forward Dynamics of PYCASPR
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
# Load the dataset
with open('PYCASPR_test/cable_forces.csv',encoding = 'utf-8') as f: 
    data_forces = np.loadtxt(f,float,delimiter = ",")
# Init the solver
pycaspr.FD_init([0.5,0.5,0.5,0,0,0])
# Loop
q_list=[]
t_st=time.time()
for i in range(data_forces.shape[1]):
    q,q_dot,q_ddot,l,l_dot,segments=pycaspr.FD_update(data_forces[:,i])
    q_list.append(q)
t_ed=time.time()
print('Time consuming: ',(t_ed-t_st)/(data_forces.shape[1]))
# Plot
q_list=np.array(q_list)
plt.figure('Results')
plt.plot(q_list)
q,q_dot,q_ddot,tv=pycaspr.Help_traj('traj_processor_test')
plt.figure('Errors')
plt.plot(q_list-q)
plt.show()