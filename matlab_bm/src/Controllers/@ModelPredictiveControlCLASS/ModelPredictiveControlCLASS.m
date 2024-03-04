classdef ModelPredictiveControlCLASS
    %MODELPREDICTIVECONTROLCLASS Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        % Initiate optimization parameters
        tol_opt = 1e-8;
        options;
        
        model;
        trajectory;
        
        stage_cost;
        inequality_constraints;
        equality_constrain;
        lb;
        ub;
        
        % Inputs
        N;
        
        % Initial state
        x0;
        u0;
        
        x_optimal;
        u_optimal;
        x_optimal_reshaped;
        u_optimal_reshaped;
    end
    
    methods
        function obj = ModelPredictiveControlCLASS(model, trajectory)
            
            obj.model = model;
            obj.trajectory = trajectory;
            
            % Quadratic Programming setting
            obj.options = optimset('Display','off',...
            'TolFun', obj.tol_opt,...
            'MaxIter', 10000,...
            'TolConSQP', 1e-6);
            
            % Model predictive control
            obj.stage_cost.H = [];
            obj.stage_cost.f = [];
            
            obj.inequality_constraints.Ai = [];
            obj.inequality_constraints.Bi = [];
            
            obj.equality_constrain.Aeq = [];
            obj.equality_constrain.Beq = [];
            
            obj.lb =[];
            
            obj.ub =[];
        end
        
        function obj = Init(obj)
            
            obj.N = 5;
            
            obj.x0 = zeros(obj.model.nx*(obj.N+1), 1);
            
            obj.u0 = zeros(obj.model.nu*obj.N, 1);
                        
            % Set up Stage Cost
            [obj.stage_cost.H, obj.stage_cost.f] = obj.SetStageCost();
            
            % Set up Inequality Constraints
            [obj.inequality_constraints.Ai, obj.inequality_constraints.Bi] = obj.SetInequalityConstraints(); 
           
        end
        
        function [obj, status, u_out] = Run(obj, dt, y, index)
            status = true;
            
            iterations = length(obj.trajectory.state_ref) - obj.N;
            
            if index > iterations
                status = false;

                u_out = zeros(obj.model.nu, 1);

                return
            end
            
            x_in_horizon = obj.trajectory.state_ref(:, index:index+obj.N);
            u_in_horizon = obj.trajectory.input_ref(:, index:index+obj.N);
            
            % Set up Equality Constraints
            [obj.equality_constrain.Aeq, obj.equality_constrain.Beq] = obj.SetEqualityConstraints(dt, x_in_horizon, u_in_horizon, y);
            
            z0 = [obj.x0; obj.u0];
            
            [z,~,~,~] = quadprog(obj.stage_cost.H, ...
                                 obj.stage_cost.f, ...
                                 obj.inequality_constraints.Ai, obj.inequality_constraints.Bi, ... 
                                 obj.equality_constrain.Aeq, obj.equality_constrain.Beq, ...
                                 obj.lb, obj.ub, ... 
                                 z0, ... 
                                 obj.options);
            
            obj.x_optimal = z(1:obj.model.nx*(obj.N+1), 1);
            obj.u_optimal = z(obj.model.nx*(obj.N+1)+1:end, 1);
            obj.x_optimal_reshaped = reshape(obj.x_optimal, obj.model.nx, obj.N+1);
            obj.u_optimal_reshaped = reshape(obj.u_optimal, obj.model.nu, obj.N);
            
            u_out = obj.u_optimal_reshaped(:,1) + u_in_horizon(:,1);
            
            obj.x0 = [obj.x_optimal(obj.model.nx+1:end); y - x_in_horizon(:, obj.N)];
            obj.u0 = [obj.u_optimal(obj.model.nu+1:end); zeros(obj.model.nu, 1)];
        end
        
        function [H,f] = SetStageCost(obj)

            Q = diag([100, 100, 0.0001]);
            R = diag([0.0001, 0.0001]);
            
            Qstack = [];
            Rstack = [];
            
            for k = 1:obj.N
                % Stage cost : weighting matrics for input (in discrete horizon) (40x40)
                Qstack = blkdiag(Qstack, Q);
                % Stage cost : weighting matrics for output (in discrete horizon) (20x20)
                Rstack = blkdiag(Rstack, R);
            end
            clear('k')
            
            % Add terminal cost = 0 (At end of horizon)
            Q_terminal = zeros(obj.model.nx);
%             Q_terminal = Q;
            Qstack = blkdiag(Qstack, Q_terminal);
            
            % Return
            % cost function (1x64)
            H = blkdiag(Qstack, Rstack);
            % system function (1x64)
            f = zeros(1, obj.model.nx*(obj.N+1) + obj.model.nu*obj.N);
        end
        
        function [Ai, Bi] = SetInequalityConstraints(obj)

            H_u = [ 1,  0,  0,  0;
                   -1,  0,  0,  0;
                    0,  1,  0,  0;
                    0, -1,  0,  0;
                    1,  0, -1,  0;
                   -1,  0,  1,  0;
                    0,  1,  0, -1;
                    0, -1,  0,  1];
            
            v_max = 1.5;
            v_min = 1.5;
            delta_max = pi/2;
            delta_min = pi/2;
            delta_v_max = 1.5/0.05; 
            delta_v_min = 1.5/0.05;
            ddelta_max = pi/(2*0.05);
            ddelta_min = pi/(2*0.05);
            k_u = [v_max, v_min, delta_max, delta_min, delta_v_max, delta_v_min, ddelta_max, ddelta_min]';

            Au = [];
            bu = [];
            
            for k=0:obj.N-2
                Au = [Au, zeros(size(Au,1), obj.model.nu);
                      zeros(size(H_u,1),k*obj.model.nu), H_u];
                bu = [bu; k_u];
            end
            clear('k')
            
            % Return
            Ai = [zeros(size(Au,1),obj.model.nx*(obj.N+1)), Au];
            Bi = bu;
            
        end
        
        function [Aeq, Beq] = SetEqualityConstraints(obj, dt, x_ref, u_ref, y_out)
                % Build equality constraints (dynamics)
                Aeq = zeros(obj.model.nx*(obj.N+1), obj.model.nx*(obj.N+1) + obj.model.nu*obj.N);
                Beq = zeros(obj.model.nx*(obj.N+1), 1);
                
                for k = 0:obj.N-1
                    % System matrices for system linearized around the reference
                    x_lin = x_ref(:, 1+k);
                    u_lin = u_ref(:, 1+k);

                    A_d = obj.model.A_d_slip(x_lin, u_lin, dt, obj.model.p);
                    B_d = obj.model.B_d_slip(x_lin, u_lin, dt , obj.model.p);

                        
                    Aeq(obj.model.nx*k+1:obj.model.nx*(k+1), 1:obj.model.nx*(obj.N+1)) = [zeros(obj.model.nx, obj.model.nx*k), A_d, -eye(obj.model.nx), zeros(obj.model.nx, obj.model.nx*(obj.N-1-k))];

                    Aeq(obj.model.nx*k+1:obj.model.nx*(k+1), obj.model.nx*(obj.N+1)+1:end) = [zeros(obj.model.nx, obj.model.nu*k), B_d, zeros(obj.model.nx, obj.model.nu*(obj.N-1-k))];
                end
                clear('k')
                
                % Equality constraints (initial constraint)
                Aeq(obj.model.nx*obj.N+1:obj.model.nx*(obj.N+1), :) = [eye(obj.model.nx), zeros(obj.model.nx, obj.model.nx*obj.N+obj.model.nu*obj.N)];
                Beq(obj.model.nx*obj.N+1:obj.model.nx*(obj.N+1))    = y_out - x_ref(:, 1);
        end
    end
end

