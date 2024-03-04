classdef EKFObserverCLASS
    %EKFOBSERVERCLASS Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        model;
        trajectory;
        measurement;
        is_noisy;
        
        estimates;
        
        o;
        n_ekf;
        C_d;
        W_d;
        V_d;
        
        R_ekf;
        Q_ekf;
        x_tilde_k;
        x_tilde_measure;
        P_k_ekf;
        
        state_estim;
        slip_params;
        p_estim;
        
        x_tilde_pred;
        P_pred;
        
        slip_estim;
        dt = 0.05;
    end
    
    methods
        function obj = EKFObserverCLASS(model,trajectory, mode)
            
            obj.model = model;
        
            obj.trajectory = trajectory;
            
            obj.measurement = NormalObserverCLASS(model,mode);
            
            obj.is_noisy = mode;
        end
        
        function obj = Init(obj, x0)
            
            obj.o = obj.model.nx;
            
            obj.n_ekf = obj.model.nx + length(obj.model.wheel_slip);
            
            obj.C_d = [eye(obj.o), zeros(obj.o,obj.n_ekf-obj.o)];
            
            obj.W_d = eye(obj.n_ekf);

            obj.V_d = eye(obj.o);
            % ====================================================================
            % covariance of measurement noise
            obj.R_ekf = diag([1 1 1 1]);
            % covariance of system noise
            obj.Q_ekf = diag([0.1 0.1 0.1 0.1 10]);
            
            obj.x_tilde_k = [x0; obj.model.wheel_slip];
            
            obj.x_tilde_measure = zeros(obj.model.nx, 1);
            
            obj.P_k_ekf = diag([0.1 0.1 0.1 0.1 10]);
            % ===================================================================
            obj.state_estim = x0;
            
            obj.slip_estim = obj.model.wheel_slip;
            
        end
        
        function [obj, observe_output] = Observe(obj,xM,u)
            
            yM = obj.measurement.Observe(xM);

            obj = obj.PredictionSTEP(u);

            obj = obj.CorrectionSTEP(yM);
            
            observe_output = obj.x_tilde_k'*obj.C_d';
                        
            if ~obj.is_noisy
                observe_output = yM;
            end            
        end
        
        function obj = PredictionSTEP(obj, u)
            
            p = obj.model.p(:,1);
            
            obj.x_tilde_pred = obj.model.f_d_extended(obj.x_tilde_k, u, obj.dt, p);
            
            A_d_ekf = obj.model.A_d_extended(obj.x_tilde_k, u, obj.dt, p);
            
            obj.P_pred = A_d_ekf.* obj.P_k_ekf.* A_d_ekf' + obj.W_d.* obj.Q_ekf.* obj.W_d;
            
        end
        
        function obj = CorrectionSTEP(obj, measurement)
            
            K_ekf = obj.P_pred*obj.C_d' / (obj.C_d*obj.P_pred*obj.C_d' + obj.V_d*obj.R_ekf*obj.V_d');
            
            obj.x_tilde_k = obj.x_tilde_pred + K_ekf*(measurement - obj.C_d*obj.x_tilde_pred);
            
            obj.P_k_ekf = (eye(obj.n_ekf) - K_ekf*obj.C_d)*obj.P_pred;
            
        end
    end
end

