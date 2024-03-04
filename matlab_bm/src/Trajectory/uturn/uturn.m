clear
clc
close all

L = 0.3;
R = 1.5;
v = 1;
T = 10; % 1 circle with R(m)-radius would run in T(sec)

T_Start = 0;
delta_t = 0.05;

horizontal_time = 0.0;
turn_time = T/4;
vertical_time = 0.0;

T_End = horizontal_time + turn_time + vertical_time + turn_time + horizontal_time;
t_ = linspace(T_Start,T_End,(1/delta_t)*T_End);
t_horizontal = linspace(T_Start, horizontal_time,(1/delta_t)*horizontal_time);
t_turn = linspace(T_Start, turn_time,(1/delta_t)*turn_time);
t_vertical = linspace(T_Start,vertical_time,(1/delta_t)*vertical_time);

x = [];
dx_dt = [];
ddx_ddt = [];
%% Trajectory generate
x0 = [0;0;0];
for i = 1:length(t_horizontal)
    [x_out,dx_dt_out,ddx_ddt_out] = GenerateHorizontalLine(x0, v, t_horizontal(i));
    x = [x, x_out];
    dx_dt = [dx_dt, dx_dt_out];
    ddx_ddt = [ddx_ddt, ddx_ddt_out];
end

x0 = [0;0;0];
for i = 1:length(t_turn)
    [x_out,dx_dt_out,ddx_ddt_out] = GenerateLeftTurnLine1(x0, R, T, t_turn(i));
    x = [x, x_out];
    dx_dt = [dx_dt, dx_dt_out];
    ddx_ddt = [ddx_ddt, ddx_ddt_out];
end

x0 = x(:,end);
for i = 1:length(t_vertical)
    [x_out,dx_dt_out,ddx_ddt_out] = GenerateVerticalLine(x0, v, t_vertical(i));
    x = [x, x_out];
    dx_dt = [dx_dt, dx_dt_out];
    ddx_ddt = [ddx_ddt, ddx_ddt_out];
end

x0 = x(:,end);
run_time = 1;
for i = 1:length(t_turn)
    [x_out,dx_dt_out,ddx_ddt_out] = GenerateLeftTurnLine2(x0, R, T, t_turn(i));
    x = [x, x_out];
    dx_dt = [dx_dt, dx_dt_out];
    ddx_ddt = [ddx_ddt, ddx_ddt_out];
end

x0 = x(:,end);
for i = 1:length(t_horizontal)
    [x_out,dx_dt_out,ddx_ddt_out] = GenerateHorizontalLine(x0, -v, t_horizontal(i));
    x = [x, x_out];
    dx_dt = [dx_dt, dx_dt_out];
    ddx_ddt = [ddx_ddt, ddx_ddt_out];
end

%% -------------------
%  Inverse Kinematic
% -------------------

x_ = x(1,:);
y_ = x(2,:);  

theta_ = atan2(dx_dt(2,:),dx_dt(1,:));
for i = 1:length(x_)
    dtheta_dt(1,i) = (ddx_ddt(2,i)*dx_dt(1,i) - dx_dt(2,i)*ddx_ddt(1,i))/(dx_dt(1,i).^2 + dx_dt(2,i).^2);
end

delta_ = atan2(L*dtheta_dt,sqrt(dx_dt(1,:).^2 + dx_dt(2,:).^2));
for i = 1:length(x_)
    v_(1,i) = sqrt((dx_dt(1,i)^2 + dx_dt(2,i)^2)/(cos(delta_(1,i)).^2));
%     v_(1,i) = sqrt(dx_dt(1,i).^2 + dx_dt(2,i).^2 + L*dtheta_dt(1,i).^2);
end

%% >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
% plot the reference
% >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
% Corporate design colors
grey = [0.2431,    0.2667,    0.2980];

% xy-plot
f1 = figure(1);
f1.Color = 'w';
plot(x_, y_, '--', 'Color', grey, 'linewidth', 1.5);
grid on;
box on;
xlim([-1 6]);
ylim([-0.5 3.5]);
ax = gca();
ax.TickLabelInterpreter = 'latex';
xlabel('position $x$ ($\mathrm{m}$)', 'interpreter', 'latex');
ylabel('position $y$ ($\mathrm{m}$)', 'interpreter', 'latex');

% save with matlab2tikz
cleanfigure('targetResolution', 300)

%% >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
% save reference
% >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
% save as csv for further use and easy import in C++
ref = [t_', x_', y_', theta_', v_', delta_'];
% round reference to reduce file size
ref = round(ref, 6);
% header line to explain columns
header = {'Time', 'x_d', 'y_d', 'theta_d', 'v_d', 'delta_'};
output = [header; num2cell(ref)];
filename = mfilename('fullpath') + ".csv";
writecell(output, filename); % introduced in Matlab 2019a