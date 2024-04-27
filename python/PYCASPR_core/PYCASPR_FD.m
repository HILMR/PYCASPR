% Forward Dynamics
function [traj_q,traj_q_dot,traj_q_ddot,cableLengths,cableLengthsDot]=PYCASPR_FD(cableForces,dt)
global FDSolver modelObj
cableForces=cableForces';
cable_indices_active=1:modelObj.numCables;
% Core
[traj_q,traj_q_dot,traj_q_ddot, modelObj] = FDSolver.compute(modelObj.q, modelObj.q_dot, ...
    cableForces, cable_indices_active, zeros(modelObj.numDofs,1),dt, modelObj);
cableLengths = modelObj.cableLengths';
cableLengthsDot = modelObj.cableLengthsDot'; 
traj_q=traj_q';
traj_q_dot=traj_q_dot';
traj_q_ddot=traj_q_ddot';
end