% Trajectory Interpolation
function [q,q_dot,q_ddot,tv]=PYCASPR_traj_gen(q,q_d,q_dd,time_points_abs,time_step,method_id)
global model_config
bodiesObj=model_config.bodiesModel;
num_points=size(q,2);
q_pj = cell(num_points,1);
q_d_pj = cell(num_points,1);
q_dd_pj = cell(num_points,1);
for p = 1:num_points
    q_pj{p} = mat2cell(q(:,p), bodiesObj.jointsNumDofVars);
    q_d_pj{p} = mat2cell(q_d(:,p), bodiesObj.jointsNumDofs);
    q_dd_pj{p} = mat2cell(q_dd(:,p), bodiesObj.jointsNumDofs);
end
% Main Functions
if method_id==1 
    trajectory = JointTrajectory.LinearTrajectoryCreate(q_pj, time_points_abs, time_step, bodiesObj);
elseif method_id==2
    trajectory = JointTrajectory.QuinticTrajectoryCreate(q_pj, q_d_pj, q_dd_pj, ...
                time_points_abs, time_step, bodiesObj);
end
q=cell2mat(trajectory.q);
q_dot=cell2mat(trajectory.q_dot);
q_ddot=cell2mat(trajectory.q_ddot);
tv=trajectory.timeVector;
end