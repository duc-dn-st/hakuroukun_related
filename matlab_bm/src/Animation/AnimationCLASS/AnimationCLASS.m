classdef AnimationCLASS
    %ANIMATIONCLASS Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        Color;
        
        figure;
        
        
    end
    
    methods
        function obj = AnimationCLASS()
            obj = obj.ReadAllColor();
        end
        
        function obj = ReadAllColor(obj)
            fileName = 'colors.json';
            str = fileread(fileName);
            colors = rows2vars(struct2table(jsondecode(str)));
            
            for i=1:size(colors)
               obj.Color.(string(colors.OriginalVariableNames(i))) = ReadColor(string(colors.OriginalVariableNames(i)));
            end
        end
    end
end

