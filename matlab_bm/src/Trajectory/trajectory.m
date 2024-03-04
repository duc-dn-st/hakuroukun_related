classdef trajectory
% Dinh Ngoc Duc - TUT
% Create and Read trajectory reference csv file
    
    properties
        ref;
        
        % Returb
        t_ref;
        
        % output_ref
        state_ref;

        % input_ref
        input_ref;

    end
    
    methods
        % Constructor
        function trajectory = trajectory()
            trajectory.ref = [];
            
            trajectory.t_ref = [];
            
            trajectory.state_ref=[];

            trajectory.input_ref=[];
        end
        
        function trajectory = read_ref(trajectory, file_name)

            % Read reference file
            ref_ = csvread(file_name, 1, 0);
            trajectory.ref = ref_';

            % get reference time
            trajectory.t_ref = trajectory.ref(1,:);
            
            % get reference state (output)
            x = trajectory.ref(2,:);
            y = trajectory.ref(3,:);
            theta = trajectory.ref(4,:);
            trajectory.state_ref=[x; y; theta];
           
            % get reference input
            v = trajectory.ref(5,:);
            delta = trajectory.ref(6,:);
            % input_ref = 0.1*ones(2,length(ref_v));        
            trajectory.input_ref=[v; delta];
        end
    end
end