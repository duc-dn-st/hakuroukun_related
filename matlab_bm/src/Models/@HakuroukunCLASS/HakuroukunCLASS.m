classdef HakuroukunCLASS
    %@HakuroukunCLASS Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        nx;
        nu;
        
        p;
    
        length_base = 0.3; % m
        wheel_slip = 0.0;
        v_max = 1.5; % m/s
        delta_max = pi/2; % rad
    end
    
    methods
        function obj = HakuroukunCLASS()
            %@HakuroukunCLASS Construct an instance of this class
            %   Detailed explanation goes here
            obj.nx = 3;
            obj.nu = 2;
        end
        
        function obj = Init(obj)
            %@HakuroukunCLASS Construct an instance of this class
            %   Detailed explanation goes here

            obj.p  = [obj.length_base, obj.wheel_slip];
        end
    end
    
    
    methods(Static)
        EquationOfMotionSlip();
    end
    
end

