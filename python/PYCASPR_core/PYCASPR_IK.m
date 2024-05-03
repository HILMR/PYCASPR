% Inverse Kinematics
function [cableLengths,cableLengthsDot,segments]=PYCASPR_IK(q,q_dot,q_ddot)
global modelObj
% Core
modelObj.update(q', q_dot', q_ddot',zeros(size(q_dot')));
[cableLengths,cableLengthsDot,segments]=PYCASPR_get_cables();
end