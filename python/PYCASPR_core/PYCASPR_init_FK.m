% Forward Kinematics Configuration
function PYCASPR_init_FK(method_id)
    global FKSolver modelObj
    % How the initial guess for the FK is made (FK_LS_ApproxOptionType enum)
    FK_q_estimation_method = FK_LS_ApproxOptionType.FIRST_ORDER_INTEGRATE_PSEUDOINV; 
    % How to q_dot is estimated (FK_LS_QdotOptionType enum)
    FK_q_dot_estimation_method = FK_LS_QdotOptionType.FIRST_ORDER_DERIV; 
    % Initialise the FK solver
    if method_id==1
        FKSolver = FKDifferential(modelObj);
    elseif method_id==2
        FKSolver = FKLeastSquares(modelObj, FK_q_estimation_method, FK_q_dot_estimation_method);
    elseif method_id==3
        FK_hybrid_frequency = 10;
        FKSolver = FKHybridLeastSquaresDifferential(modelObj, FK_q_estimation_method, FK_q_dot_estimation_method, FK_hybrid_frequency);
    end
end
