% Trajectory Loader
function [q,q_dot,q_ddot,tv]=PYCASPR_traj(traj_id)
global model_config
trajectory = model_config.getJointTrajectory(traj_id);
q=cell2mat(trajectory.q);
q_dot=cell2mat(trajectory.q_dot);
q_ddot=cell2mat(trajectory.q_ddot);
tv=trajectory.timeVector;
end