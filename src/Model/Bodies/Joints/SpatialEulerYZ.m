% Spatial (5-DOF) joint using the YZ-Euler orientation representation
%
% Author        : Mingrui Luo
% Created       : 2024
% Description   : Customized joint types

classdef SpatialEulerYZ < JointBase
    properties
        translation
        orientation
    end
        
    properties (Constant = true)
        numDofs = TranslationalXYZ.numDofs + UniversalYZ.numDofs;
        numVars = TranslationalXYZ.numVars + UniversalYZ.numVars;
        q_default = [TranslationalXYZ.q_default; UniversalYZ.q_default];
        q_dot_default = [TranslationalXYZ.q_dot_default; UniversalYZ.q_dot_default];
        q_ddot_default = [TranslationalXYZ.q_ddot_default; UniversalYZ.q_ddot_default];
        q_lb = [TranslationalXYZ.q_lb; UniversalYZ.q_lb];
        q_ub = [TranslationalXYZ.q_ub; UniversalYZ.q_ub];
        q_dofType = [DoFType.TRANSLATION; DoFType.TRANSLATION; DoFType.TRANSLATION; DoFType.ROTATION; DoFType.ROTATION];
    end
    
    properties (Dependent)        
        x
        y
        z
        x_dot
        y_dot
        z_dot
        
        alpha
        beta
        alpha_dot
        beta_dot
    end
    
    methods
        % Constructors
        function j = SpatialEulerYZ()
            j.translation = TranslationalXYZ;
            j.orientation = UniversalYZ;
        end
        
        % Update the joint space
        function update(obj, q, q_dot, q_ddot)
            obj.translation.update(SpatialEulerYZ.GetTranslationQ(q), SpatialEulerYZ.GetTranslationQd(q_dot), SpatialEulerYZ.GetTranslationQd(q_ddot));
            obj.orientation.update(SpatialEulerYZ.GetOrientationQ(q), SpatialEulerYZ.GetOrientationQd(q_dot), SpatialEulerYZ.GetOrientationQd(q_ddot));
            update@JointBase(obj, q, q_dot, q_ddot);
        end
        
        % Generate the trajectory functions
        function [q, q_dot, q_ddot] = generateTrajectoryLinearSpline(obj, q_s, q_e, time_vector)
            [q_trans, q_trans_dot, q_trans_ddot] = obj.translation.generateTrajectoryLinearSpline( ...
                obj.GetTranslationQ(q_s), ...
                obj.GetTranslationQ(q_e), time_vector);
            [q_orient, q_orient_dot, q_orient_ddot] = obj.orientation.generateTrajectoryLinearSpline( ...
                obj.GetOrientationQ(q_s), ...
                obj.GetOrientationQ(q_e), time_vector);
            q = [q_trans; q_orient];
            q_dot = [q_trans_dot; q_orient_dot];
            q_ddot = [q_trans_ddot; q_orient_ddot];
        end
        
        function [q, q_dot, q_ddot] = generateTrajectoryCubicSpline(obj, q_s, q_s_d, q_e, q_e_d, time_vector)
            [q_trans, q_trans_dot, q_trans_ddot] = obj.translation.generateTrajectoryCubicSpline( ...
                obj.GetTranslationQ(q_s), obj.GetTranslationQd(q_s_d), ...
                obj.GetTranslationQ(q_e), obj.GetTranslationQd(q_e_d), time_vector);
            [q_orient, q_orient_dot, q_orient_ddot] = obj.orientation.generateTrajectoryCubicSpline( ...
                obj.GetOrientationQ(q_s), obj.GetOrientationQd(q_s_d), ...
                obj.GetOrientationQ(q_e), obj.GetOrientationQd(q_e_d), time_vector);
            q = [q_trans; q_orient];
            q_dot = [q_trans_dot; q_orient_dot];
            q_ddot = [q_trans_ddot; q_orient_ddot];
        end
        
        function [q, q_dot, q_ddot] = generateTrajectoryQuinticSpline(obj, q_s, q_s_d, q_s_dd, q_e, q_e_d, q_e_dd, time_vector)
            [q_trans, q_trans_dot, q_trans_ddot] = obj.translation.generateTrajectoryQuinticSpline( ...
                obj.GetTranslationQ(q_s), obj.GetTranslationQd(q_s_d), obj.GetTranslationQd(q_s_dd), ...
                obj.GetTranslationQ(q_e), obj.GetTranslationQd(q_e_d), obj.GetTranslationQd(q_e_dd), time_vector);
            [q_orient, q_orient_dot, q_orient_ddot] = obj.orientation.generateTrajectoryQuinticSpline( ...
                obj.GetOrientationQ(q_s), obj.GetOrientationQd(q_s_d), obj.GetOrientationQd(q_s_dd), ...
                obj.GetOrientationQ(q_e), obj.GetOrientationQd(q_e_d), obj.GetOrientationQd(q_e_dd), time_vector);
            q = [q_trans; q_orient];
            q_dot = [q_trans_dot; q_orient_dot];
            q_ddot = [q_trans_ddot; q_orient_ddot];
        end
        
        function [q, q_dot, q_ddot] = generateTrajectoryParabolicBlend(obj, q_s, q_e, time_vector, time_blend)
            [q_trans, q_trans_dot, q_trans_ddot] = obj.translation.generateTrajectoryParabolicBlend( ...
                obj.GetTranslationQ(q_s), ...
                obj.GetTranslationQ(q_e), time_vector, time_blend);
            [q_orient, q_orient_dot, q_orient_ddot] = obj.orientation.generateTrajectoryParabolicBlend( ...
                obj.GetOrientationQ(q_s),  ...
                obj.GetOrientationQ(q_e), time_vector, time_blend);
            q = [q_trans; q_orient];
            q_dot = [q_trans_dot; q_orient_dot];
            q_ddot = [q_trans_ddot; q_orient_ddot];
        end
        
        % -------
        % Getters
        % -------
        function value = get.x(obj)
            value = obj.translation.x;
        end
        function value = get.y(obj)
            value = obj.translation.y;
        end
        function value = get.z(obj)
            value = obj.translation.z;
        end
        function value = get.x_dot(obj)
            value = obj.translation.x_dot;
        end
        function value = get.y_dot(obj)
            value = obj.translation.y_dot;
        end
        function value = get.z_dot(obj)
            value = obj.translation.z_dot;
        end
        function value = get.alpha(obj)
            value = obj.orientation.alpha;
        end
        function value = get.beta(obj)
            value = obj.orientation.beta;
        end
        function value = get.alpha_dot(obj)
            value = obj.orientation.alpha_dot;
        end
        function value = get.beta_dot(obj)
            value = obj.orientation.beta_dot;
        end
    end
    
    methods (Static)
        function q_t = GetTranslationQ(q)
            q_t = q(1:TranslationalXYZ.numVars);
        end
        function q_t = GetTranslationQd(q_d)
            q_t = q_d(1:TranslationalXYZ.numVars);
        end
        function q_t = GetOrientationQ(q)
            q_t = q(TranslationalXYZ.numVars+1:SpatialEulerYZ.numVars);
        end
        function q_t_d = GetOrientationQd(q_d)
            q_t_d = q_d(TranslationalXYZ.numDofs+1:SpatialEulerYZ.numDofs);
        end
        
        % Get the relative rotation matrix
        function R_pe = RelRotationMatrix(q)
            R_pe = UniversalYZ.RelRotationMatrix(SpatialEulerYZ.GetOrientationQ(q));
        end

        % Get the relative translation vector
        function r_rel = RelTranslationVector(q)
            r_rel = TranslationalXYZ.RelTranslationVector(SpatialEulerYZ.GetTranslationQ(q));
        end
        
        % Generate the S matrix
        function S = RelVelocityMatrix(q)
            S = [TranslationalXYZ.RelVelocityMatrix(SpatialEulerYZ.GetTranslationQ(q)) ...
                UniversalYZ.RelVelocityMatrix(SpatialEulerYZ.GetOrientationQ(q))];
        end
        
        % Generate the S gradient tensor
        function S_grad = RelVelocityMatrixGradient(q)
            S_grad              =   MatrixOperations.Initialise([6,5,5],isa(q, 'sym'));
            S_grad(:,4:5,4:5)   =  UniversalYZ.RelVelocityMatrixGradient(SpatialEulerYZ.GetOrientationQ(q));
        end
        
        % Generate the \dot{S} gradient tensor
        function [S_dot_grad] = RelVelocityMatrixDerivGradient(q,q_dot)
            S_dot_grad              =   MatrixOperations.Initialise([6,5,5],isa(q, 'sym'));
            S_dot_grad(:,4:5,4:5)   =  UniversalYZ.RelVelocityMatrixDerivGradient(SpatialEulerYZ.GetOrientationQ(q),SpatialEulerYZ.GetOrientationQd(q_dot));
        end
        
        % The derivative matrix
        function q_deriv = QDeriv(q, q_d)
            q_deriv = [TranslationalXYZ.QDeriv(SpatialEulerYZ.GetTranslationQ(q), SpatialEulerYZ.GetTranslationQd(q_d)); ...
                UniversalYZ.QDeriv(SpatialEulerYZ.GetOrientationQ(q), SpatialEulerYZ.GetOrientationQd(q_d))];
        end
        
        % Generate the N matrix for the joint
        function [N_j,A] = QuadMatrix(~)
            g = SphericalEulerXYZ.GetBeta(q);
            N_j = zeros(SpatialEulerYZ.numDofs,SpatialEulerYZ.numDofs^2);

%             N_j(4:5,24:25) = [0,-0.5*sin(g);-0.5*sin(g),0];
%             N_j(4:5,20:21) = [0,0.5*cos(g);0.5*cos(g),0];
            
            A = [eye(5);zeros(1,5)];
        end
    end
end
