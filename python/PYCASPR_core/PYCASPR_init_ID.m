% Inverse Dynamics Configuration
function PYCASPR_init_ID(method_id)
global IDSolver modelObj
if method_id==1
    % linprog
    id_objective = IDObjectiveMinLinCableForce(ones(modelObj.numActuatorsActive,1));
    IDSolver = IDSolverLinProg(modelObj, id_objective, ID_LP_SolverType.MATLAB);
elseif method_id==2
    % quadprog
    id_objective = IDObjectiveMinQuadCableForce(ones(modelObj.numActuatorsActive,1));
    IDSolver = IDSolverQuadProg(modelObj, id_objective, ID_QP_SolverType.MATLAB);
elseif method_id==3
    % feasible_polygon
    IDSolver = IDSolverFeasiblePolygon(modelObj, ID_FP_SolverType.NORM_2);
elseif method_id==4
    % optimally_safe
    IDSolver = IDSolverOptimallySafe(modelObj, 1.0, ID_OS_SolverType.LP);
elseif method_id==5
    % closed_form
    IDSolver = IDSolverClosedForm(modelObj, ID_CF_SolverType.IMPROVED_CLOSED_FORM);
elseif method_id==6
    % min_inf_norm
    id_objective = IDObjectiveMinInfCableForce(ones(modelObj.numActuatorsActive,1));
    IDSolver = IDSolverMinInfNorm(modelObj, id_objective, ID_LP_SolverType.MATLAB);
end
end