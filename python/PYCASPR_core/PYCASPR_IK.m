% Inverse Kinematics
function [cableLengths,cableLengthsDot,segments]=PYCASPR_IK(q,q_dot,q_ddot)
global modelObj
% Core
modelObj.update(q', q_dot', q_ddot',zeros(size(q_dot')));
cableLengths = modelObj.cableLengths';
cableLengthsDot = modelObj.cableLengthsDot';
for i = 1:modelObj.cableModel.numCables
    for j = 1:modelObj.cableModel.cables{i}.numSegments
        segments(i,j*2-1,:)=modelObj.cableModel.cables{i}.segments{j}.attachments{1}.r_OA';
        segments(i,j*2,:)=modelObj.cableModel.cables{i}.segments{j}.attachments{2}.r_OA';
    end
end
end