classdef TimeSteppingCLASS
    %TIMESTEPPING Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        t_start = 0;
        t_end;
        dt     = 0.5;
        
        t_out    = [];
        x_out    = [];
        y_out    = [];
        u_out    = [];
        n_t;
        
        trajectory;
        model;
        controller;
        observer;
    end
    
    methods
        function obj = TimeSteppingCLASS(trajectory, model, controller, observer)
            obj.trajectory = trajectory;
            obj.model = model;
            obj.controller = controller;
            obj.observer = observer;
            
            obj.t_end = obj.trajectory.t_ref(1,end);
            obj.t_out = linspace(obj.t_start, obj.t_end, (1/obj.dt) * obj.t_end);
            obj.n_t = length(obj.t_out); 
            obj.x_out = zeros(obj.model.nx,obj.n_t);
            obj.y_out = zeros(obj.model.nx,obj.n_t);
            obj.u_out = zeros(obj.model.nu,obj.n_t);
            
        end
        
        function obj = Loop(obj, x0)
            
            % Initialization
            obj.x_out(:,1) = x0;        % State Real in Math Logic
            obj.y_out(:,1) = x0;        % Measurement

            % Loop
            for i=1:obj.n_t

                [obj.controller, status, obj.u_out(:,i)] = obj.controller.Run(obj.dt, obj.y_out(:,i), i);
                
                steering_limit = 2*pi/9;
                if obj.u_out(2,i) > steering_limit
                    obj.u_out(2,i) = steering_limit;
                elseif obj.u_out(2,i) < -steering_limit
                    obj.u_out(2,i) = -steering_limit;
                end
                
                if obj.u_out(1,i) > 1.5
                    obj.u_out(1,i) = 1.5;
                elseif obj.u_out(1,i) < 0
                    obj.u_out(1,i) = 0;
                end
                
                if ~status
                    obj.u_out(:,i) = zeros(obj.model.nu,1);
                end
               
                xM = obj.model.f_d_slip(obj.y_out(:,i), obj.u_out(:,i), obj.dt, obj.model.p);
                
                x_hat = obj.observer.Observe(xM, obj.u_out(:,i)); 
                
                if i <= obj.n_t
                    obj.x_out(:, i+1) = xM;
                    obj.y_out(:, i+1) = x_hat;
                end
            end
        end
    end
end

