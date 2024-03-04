classdef NormalObserverCLASS
    %NORMALOBSERVERCLASS Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        model;
        
        is_noisy;
    end
    
    methods
        function obj = NormalObserverCLASS(model,mode)
            obj.model = model;
        
            obj.is_noisy = mode;
        end
        
        function obj = Init(obj,x0) 
        end
        
        function measured_output = Observe(obj,system_output, u)

            noise_sigma = diag(ones(obj.model.nx, 1))*1e-4;
            
            noise_mu = zeros(obj.model.nx,1);
            
            noise_variance = chol(noise_sigma);

            measure_noise = randn(1, obj.model.nx) * noise_variance;

            measure_noise = noise_mu + measure_noise';
            
            measured_output = system_output + measure_noise;

            if ~obj.is_noisy
                measured_output = system_output;
            end
        end
    end
end

