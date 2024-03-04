function EquationOfMotionSlip()
    
    syms length_base wheel_slip dt;
    
    p = [length_base, wheel_slip];

    nx = 3;

    nu = 2;
    
    % [x, y, theta]
    x = sym('x',[nx,1]);
    % [v, delta]
    u = sym('u',[nu,1]);
    
    % Linear state and input
        x_lin = sym('x_lin',[nx,1]);
        u_lin = sym('u_lin',[nu,1]);
    % Non-linear state and input
        x_k   = sym('x_k',[nx,1]);
        u_k   = sym('u_k',[nu,1]);
    
% Dynamics :
    % v = u(1) * (1- p(2));
    v = u(1);
    f = [v * cos(u(2)) * cos(x(3));
         v * cos(u(2)) * sin(x(3));
         v * sin(u(2)) / p(1)];
    
% Linearized

    dfdx = jacobian(f,x);
    dfdu = jacobian(f,u);
    
    A = subs(dfdx, [x;u], [x_lin;u_lin]);
    B = subs(dfdu, [x;u], [x_lin;u_lin]);
    
% Discreted
    f_d_slip = x_k + subs(f, [x;u], [x_k;u_k]) * dt;

    A_d_slip = eye(nx) + A * dt;
    B_d_slip = B * dt;
        
% Create MATLAB-functions:
% identify the current file location, to place all functions there
    filename = mfilename('fullpath');
    [filepath,~,~] = fileparts(filename);
% dummy variable for obj, so that these can be used within the CLASS
    syms obj 
% MATLAB Function:

%     matlabFunction(f,'File',[filepath,'/f'],'Vars',{obj,x,u,dt,parameters});
    matlabFunction(f_d_slip,'File',[filepath,'/f_d_slip'],'Vars',{obj,x_k,u_k,dt,p});
    matlabFunction(A_d_slip,'File',[filepath,'/A_d_slip'],'Vars',{obj,x_lin,u_lin,dt,p});
    matlabFunction(B_d_slip,'File',[filepath,'/B_d_slip'],'Vars',{obj,x_lin,u_lin,dt,p});
    
end