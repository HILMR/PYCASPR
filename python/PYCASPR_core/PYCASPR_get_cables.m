function [cableLengths,cableLengthsDot,segments]=PYCASPR_get_cables()
global modelObj
cableLengths = modelObj.cableLengths';
cableLengthsDot = modelObj.cableLengthsDot';
for i = 1:modelObj.cableModel.numCables
    for j = 1:modelObj.cableModel.cables{i}.numSegments
        segments(i,j*2-1,:)=modelObj.cableModel.cables{i}.segments{j}.attachments{1}.r_OA';
        segments(i,j*2,:)=modelObj.cableModel.cables{i}.segments{j}.attachments{2}.r_OA';
    end
end
end