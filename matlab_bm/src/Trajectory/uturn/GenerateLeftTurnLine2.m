function [x, dxdt, ddxddt] = GenerateLeftTurnLine2(x0, R, T, t)

    % Circle trajectory specs
    w = 2*pi/T;     % rad/sec

    % Circular Trajectory Function
    x_r = R*cos(w*t);

    y_r = R*sin(w*t);

    x_r_d = -R*w*sin(w*t);

    y_r_d = R*w*cos(w*t);

    x_r_dd = -R*w*w*cos(w*t);

    y_r_dd = -R*w*w*sin(w*t);

    % ====================================
    x      =  x0 + [-R+x_r; y_r; 0];

    dxdt   = [x_r_d; y_r_d];

    ddxddt = [x_r_dd; y_r_dd];
end