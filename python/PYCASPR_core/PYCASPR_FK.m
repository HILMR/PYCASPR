% Forward Kinematics
function [traj_q,traj_q_dot]=PYCASPR_FK(lengths,lengths_prev,q_prev, q_d_prev,dt)
global FKSolver modelObj
cable_indices=1:modelObj.numCables;
% Core
[traj_q, traj_q_dot, ~] = FKSolver.compute(lengths', lengths_prev', q_prev', q_d_prev', dt, cable_indices);
traj_q=traj_q';
traj_q_dot=traj_q_dot';
end