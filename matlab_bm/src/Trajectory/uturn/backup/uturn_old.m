clear
clc
close all

T_Start = 0;
delta_t = 0.05;

horizontal_time = 4.5;
turn_time = 1;
vertical_time = 2;

T_End = horizontal_time + turn_time + vertical_time + turn_time + horizontal_time;
t = linspace(T_Start,T_End,(1/delta_t)*T_End);


t_horizontal = linspace(T_Start,horizontal_time,(1/delta_t)*horizontal_time);
t_turn = linspace(T_Start,turn_time,(1/delta_t)*turn_time);
t_vertical = linspace(T_Start,vertical_time,(1/delta_t)*vertical_time);

R = 0.5;
L = 0.3;
v = 1;

x = [];
dx_dt = [];
ddx_ddt = [];

x0 = [0;0;0];
for i = 1:length(t_horizontal)
    [x_out,dx_dt_out,ddx_ddt_out] = GenerateHorizontalLine(v, x0, t_horizontal(i));
    x = [x, x_out];
    dx_dt = [dx_dt, dx_dt_out];
    ddx_ddt = [ddx_ddt, ddx_ddt_out];
end

x0 = x(:,end);
run_time = 1;
for i = 1:length(t_turn)
    [x_out,dx_dt_out,ddx_ddt_out] = GenerateLeftTurnLine1(x0,run_time,t_turn(i),R);
    x = [x, x_out];
    dx_dt = [dx_dt, dx_dt_out];
    ddx_ddt = [ddx_ddt, ddx_ddt_out];
end

x0 = x(:,end);
for i = 1:length(t_vertical)
    [x_out,dx_dt_out,ddx_ddt_out] = GenerateVerticalLine(v, x0, t_vertical(i));
    x = [x, x_out];
    dx_dt = [dx_dt, dx_dt_out];
    ddx_ddt = [ddx_ddt, ddx_ddt_out];
end

x0 = x(:,end);
run_time = 1;
for i = 1:length(t_turn)
    [x_out,dx_dt_out,ddx_ddt_out] = GenerateLeftTurnLine2(x0,run_time,t_turn(i),R);
    x = [x, x_out];
    dx_dt = [dx_dt, dx_dt_out];
    ddx_ddt = [ddx_ddt, ddx_ddt_out];
end

x0 = x(:,end);
for i = 1:length(t_horizontal)
    [x_out,dx_dt_out,ddx_ddt_out] = GenerateHorizontalLine(-v, x0, t_horizontal(i));
    x = [x, x_out];
    dx_dt = [dx_dt, dx_dt_out];
    ddx_ddt = [ddx_ddt, ddx_ddt_out];
end

x_ = x(1,:);
y_ = x(2,:);

theta_ = atan2(dx_dt(2,:),dx_dt(1,:));

for i = 1:length(t)
    dtheta_dt(1,i) = (ddx_ddt(2,i)*dx_dt(1,i) - dx_dt(2,i)*ddx_ddt(1,i))/(dx_dt(1,i).^2 + dx_dt(2,i).^2);
end

delta_ = atan2(L*dtheta_dt,sqrt(dx_dt(1,:).^2 + dx_dt(2,:).^2));
% delta_ = asin(L * dtheta_dt)
for i = 1:length(t)
    v_(1,i) = sqrt((dx_dt(1,i)^2 + dx_dt(2,i)^2)/(cos(delta_(1,i)).^2));
end

ddelta_dt = zeros(1,length(t));
for i = 1:length(t)-1
    ddelta_dt(1,i) = (delta_(1,i+1) - delta_(1,i))/delta_t;
    if (delta_(1,i+1) - delta_(1,i)) == 0 && delta_(1,i+1) > 0
        ddelta_dt(1,i) = ddelta_dt(1,i-1);
    end
end

% state = [x_;y_;theta_;delta_];
% input = [v_;ddelta_dt];

state = [x_;y_;theta_];
input = [v_;delta_];
%% >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
% plot the reference
% >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
% Corporate design colors
grey = [0.2431,    0.2667,    0.2980];
red = [0.7529, 0.0000, 0.000];
green = [0.0000, 0.6902, 0.3137];
blue = [0.0000, 0.3176, 0.6196];
yellow = [1.0000, 0.8353, 0.0000];


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

% %% >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
% % save reference
% % >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
% % save as csv for further use and easy import in C++
% ref = [t', x_', y_', theta_', delta_', v_', ddelta_dt'];
% % round reference to reduce file size
% ref = round(ref, 6);
% % header line to explain columns
% header = {'Time', 'x_d', 'y_d', 'theta_d', 'delta_', 'v_d', 'ddelta_dt'};
% output = [header; num2cell(ref)];
% filename = mfilename('fullpath') + ".csv";
% writecell(output, filename); % introduced in Matlab 2019a

%% >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
% save reference
% >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
% save as csv for further use and easy import in C++
ref = [t', x_', y_', theta_', v_', delta_'];
% round reference to reduce file size
ref = round(ref, 6);
% header line to explain columns
header = {'Time', 'x_d', 'y_d', 'theta_d', 'v_d', 'delta_'};
output = [header; num2cell(ref)];
filename = mfilename('fullpath') + ".csv";
writecell(output, filename); % introduced in Matlab 2019a