function [x, dxdt, ddxddt] = GenerateVerticalLine(x0, v, t)
        x      = x0 + [0; v*t; 0];

        dxdt   = [0; v];

        ddxddt = [0; 0];
end