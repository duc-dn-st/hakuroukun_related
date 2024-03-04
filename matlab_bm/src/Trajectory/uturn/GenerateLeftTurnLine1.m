function [x, dxdt, ddxddt] = GenerateLeftTurnLine1(x0, R, T, t)
        
    % Circle trajectory specs
    w = 2*pi/T;     % rad/sec

    % Circular Trajectory Function
    x_r = R*cos(w*t+3*pi/2);

    y_r = R*sin(w*t+3*pi/2);

    x_r_d = -R*w*sin(w*t+3*pi/2);
    
    y_r_d = R*w*cos(w*t+3*pi/2);

    x_r_dd = -R*w*w*cos(w*t+3*pi/2);
    
    y_r_dd = -R*w*w*sin(w*t+3*pi/2);

    %--------------------------------
    x      =  x0 + [x_r; R+y_r; 0];

    dxdt   = [x_r_d; y_r_d];

    ddxddt = [x_r_dd; y_r_dd];
end