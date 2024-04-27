% Forward Dynamics Configuration
function PYCASPR_init_FD(method_id,q0,q0_dot)
global FDSolver modelObj
method_list=FDSolverType(["ODE45" "ODE23" "ODE113" "ODE15S" "ODE23S" "ODE23T" "ODE23TB" "ODE4" "ODE1"]);
% Main Object
FDSolver= ForwardDynamics(method_list(method_id));

q0=q0';
q0_dot=q0_dot';
modelObj.update(q0, q0_dot, zeros(modelObj.numDofs,1), zeros(modelObj.numDofs,1));
q0_ddot = modelObj.q_ddot_dynamics;
modelObj.update(q0, q0_dot, q0_ddot, zeros(modelObj.numDofs,1));
end