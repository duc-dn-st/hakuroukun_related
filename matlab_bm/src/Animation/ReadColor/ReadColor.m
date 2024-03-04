function color_o = ReadColor(color_i)
    fileName = 'colors.json';
    str = fileread(fileName);
    colors = rows2vars(struct2table(jsondecode(str)));
    color_o = [0,0,0];
    for i=1:size(colors,1)
        if string(color_i) == string(colors.OriginalVariableNames(i))
            color_o = [colors.Var1(i).R, colors.Var1(i).G, colors.Var1(i).B];
        end
    end
end
