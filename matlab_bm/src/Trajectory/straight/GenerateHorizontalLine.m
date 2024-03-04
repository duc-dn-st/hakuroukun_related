function [x, dxdt, ddxddt] = GenerateHorizontalLine(x0, v, t)
        x      = x0 + [v*t; 0; 0];

        dxdt   = [v; 0];

        ddxddt = [0; 0];
end