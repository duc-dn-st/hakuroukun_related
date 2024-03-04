%% generate reference trajectory for wmr
% Needs MATLAB 2019a starting in line 133 (because of writecell).
% But you can just use the already generated reverence.csv for the other
% files. Another way is to use csvwrite, but then you cannot include
% headers for the columns (and you have to change all the files using
% reference.csv, because I removed the header line there)
% >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
clear 
close all
clc

T_max = 60;
t = linspace(0,T_max,20*T_max);     % 1200 steps -  should take 60s to complete with 20 Hz sampling rate
delta_t = 0.05;                     % sampling time
L = 0.3;                           % distance between the front and rear axles
R = 5;                              % 'radius'
w = 2*pi/T_max;                     % angular velocity
dir = 1;                            % direction of traversion, only +/- 1 possilbe

% Lemniscate of gerono with constant angular velocity w
x_ = R*sin(2*w*t) / 2;
y_ = R*(cos(w*t)-1);
% First derivative of adapted lemniscate of gerono
dx_dt = R*w*cos(2*w*t);
dy_dt = -R*w*sin(w*t);
% Second derivative of adapted lemniscate of gerono
ddx_ddt = -2*R*w*w*sin(2*w*t);
ddy_ddt = -R*w*w*cos(w*t);

theta_ = atan2(dy_dt,dx_dt);

for i = 1:length(t)
    dtheta_dt(1,i) = (ddy_ddt(1,i)*dx_dt(1,i) - dy_dt(1,i)*ddx_ddt(1,i))/(dx_dt(1,i)^2 + dy_dt(1,i)^2);
end

delta_ = atan2(L*dtheta_dt,sqrt(dx_dt(1,:).^2 + dy_dt(1,:).^2));
% delta_ = asin(L * dtheta_dt)
for i = 1:length(t)
    v_(1,i) = sqrt((dx_dt(1,i)^2 + dy_dt(1,i)^2)/(cos(delta_(1,i)).^2));
end

ddelta_dt = zeros(1,length(t));
for i = 1:length(t)-1
    ddelta_dt(1,i) = (delta_(1,i+1) - delta_(1,i))/delta_t;
    if (delta_(1,i+1) - delta_(1,i)) == 0 && delta_(1,i+1) > 0
        ddelta_dt(1,i) = ddelta_dt(1,i-1);
    end
end

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
plot(x_, y_, '-', 'Color', blue, 'linewidth', 1.5);
grid on;
box on;

ax = gca();
ax.TickLabelInterpreter = 'latex';
xlabel('position $x$ in $\mathrm{m}$', 'interpreter', 'latex');
ylabel('position $y$ in $\mathrm{m}$', 'interpreter', 'latex');

% save with matlab2tikz
cleanfigure('targetResolution', 300)
% matlab2tikz('figures/xy_reference.tex','width','\fwidth', 'encoding', 'utf8')


% theta and theta_dot plot
f2 = figure(2);
f2.Color = 'w';
hold on
% plot(t, delta_, '-', 'Color', yellow, 'linewidth', 1.5);
plot(t, theta_, '-', 'Color', blue, 'linewidth', 1.5);
% plot(t, theta_d_dot, '-', 'Color', red, 'linewidth', 1.5);
grid on;
box on;
hold off;
xlim([0, T_max])
ylim([-2.5,2.5])

ax = gca();
ax.TickLabelInterpreter = 'latex';
xlabel('time $t$ in $\mathrm{s}$', 'interpreter', 'latex');
ylabel('$\theta$ in $\mathrm{rad}$, $\dot{\theta}$ in $\mathrm{rad/s}$',...
        'interpreter', 'latex');
legend({'$\theta$', '$\dot{\theta}$'}, 'interpreter', 'latex',...
                                                            'orientation', 'vertical',...
                                                            'location', 'southeast');
% save with matlab2tikz
cleanfigure('targetResolution', 300)
% matlab2tikz('figures/theta_reference.tex','width','\fwidth', 'encoding', 'utf8')


% input (v_d, delta_d) plot
f3 = figure(3);
f3.Color = 'w';
hold on
plot(t, v_, '-', 'Color', blue, 'linewidth', 1.5);
plot(t, ddelta_dt, '-', 'Color', red, 'linewidth', 1.5);
grid on;
box on;
hold off;
xlim([0, T_max]);
ylim([-3, 3]);

ax = gca();
ax.TickLabelInterpreter = 'latex';
xlabel('time $t$ in $\mathrm{s}$', 'interpreter', 'latex');
ylabel('wheel velocities in $\mathrm{m/s}$', 'interpreter', 'latex');
legend({'$v_{d}$', '$delta dot_{d}$'}, 'interpreter', 'latex',...
                                               'orientation', 'vertical',...
                                               'location', 'southeast');
% save with matlab2tikz
cleanfigure('targetResolution', 300)
% matlab2tikz('figures/input_reference.tex','width','\fwidth', 'encoding', 'utf8')

% >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
% save reference
% >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
% save as csv for further use and easy import in C++
ref = [t', x_', y_', theta_', v_', delta_'];
% round reference to reduce file size
ref = round(ref, 7);
% header line to explain columns
header = {'Time', 'x_d', 'y_d', 'theta_d','v_', 'delta_'};
output = [header; num2cell(ref)];
writecell(output, 'Trajectory/lemniscate_of_gerono/lemniscate_of_gerono_ref.csv'); % introduced in Matlab 2019a

