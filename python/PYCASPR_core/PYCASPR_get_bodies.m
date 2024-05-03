function [r_OPs,r_OGs,r_OPes,R]=PYCASPR_get_bodies()
global modelObj
for k = 1:modelObj.numLinks
    r_OP0 = modelObj.bodyModel.bodies{k}.R_0k*modelObj.bodyModel.bodies{k}.r_OP;
    r_OG0 = modelObj.bodyModel.bodies{k}.R_0k*modelObj.bodyModel.bodies{k}.r_OG;
    r_OPe0 = modelObj.bodyModel.bodies{k}.R_0k*modelObj.bodyModel.bodies{k}.r_OPe;
    R(k,:,:) = modelObj.bodyModel.bodies{k}.R_0k;
    r_OPs(k,:)=r_OP0';
    r_OGs(k,:)=r_OG0';
    r_OPes(k,:)=r_OPe0';
end
end