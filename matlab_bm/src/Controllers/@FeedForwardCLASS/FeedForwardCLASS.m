classdef FeedForwardCLASS
    %FEEDFORWARDCLASS Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        
        trajectory;
            
    end
    
    methods
        function obj = FeedForwardCLASS(trajectory)
            
            obj.trajectory = trajectory;
        
        end
        
        function obj = Init(obj)
                   
        end
        
        function [obj, status, system_input] = Run(obj, dt, measure_output, i)
            
            status = true;
            
            system_input = obj.trajectory.input_ref(:,i);
        
        end
        
        
    end
end

