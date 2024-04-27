% Inverse Dynamics
function [cableForces,cableLengths,cableLengthsDot]=PYCASPR_ID(traj_q,traj_q_dot,traj_q_ddot)
global IDSolver modelObj
% Core
[cableForces, modelObj, ~, ~, ~] = IDSolver.resolve(traj_q',traj_q_dot',traj_q_ddot', zeros(modelObj.numDofs,1));
cableForces = cableForces';
cableLengths = modelObj.cableLengths';
cableLengthsDot = modelObj.cableLengthsDot';
end