"""
Test Forward Kinematics of PYCASPR
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
with open('PYCASPR_test/cable_lengths.csv',encoding = 'utf-8') as f: 
    data_lengths = np.loadtxt(f,float,delimiter = ",")
# Init the solver
q_list=[[0.5,0.5,0.5,0,0,0]]
pycaspr.FK_init(data_lengths[:,0],q_list[0])
# Loop
t_st=time.time()
for i in range(1,data_lengths.shape[1]):
    q,q_dot=pycaspr.FK_update(data_lengths[:,i])
    q_list.append(q)
t_ed=time.time()
print('Time consuming: ',(t_ed-t_st)/(data_lengths.shape[1]-1))
# Plot
plt.figure('Results')
q_list=np.array(q_list)
plt.plot(q_list)
q,q_dot,q_ddot,tv=pycaspr.Help_traj('traj_processor_test')
plt.figure('Errors')
plt.plot(q_list-q)
plt.show()