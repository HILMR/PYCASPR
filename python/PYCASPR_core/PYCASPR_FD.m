% Forward Dynamics
function [traj_q,traj_q_dot,traj_q_ddot,cableLengths,cableLengthsDot,segments]=PYCASPR_FD(cableForces,dt,w_ext)
global FDSolver modelObj
cableForces=cableForces';
cable_indices_active=1:modelObj.numCables;
% Core
[traj_q,traj_q_dot,traj_q_ddot, modelObj] = FDSolver.compute(modelObj.q, modelObj.q_dot, ...
    cableForces, cable_indices_active, w_ext', dt, modelObj);

traj_q=traj_q';
traj_q_dot=traj_q_dot';
traj_q_ddot=traj_q_ddot';

[cableLengths,cableLengthsDot,segments]=PYCASPR_get_cables();
end