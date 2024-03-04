run_time = 10;
T = 20;
t = linspace(0,run_time,T*run_time);
R = 1.5;
w = 2*pi*(1/T);
L = 1;
% delta_t = (1/T);
delta_t = 1;

x_d = R*cos(w*t+3*pi/2);
y_d = R*sin(w*t+3*pi/2)+R;
% First derivative of adapted lemniscate of gerono
x_d_dot = -R*w*sin(w*t+3*pi/2);
y_d_dot = R*w*cos(w*t+3*pi/2);
% Second derivative of adapted lemniscate of gerono
x_d_d_dot = -R*w*w*cos(w*t+3*pi/2);
y_d_d_dot = -R*w*w*sin(w*t+3*pi/2);
% Reference for theta_d
theta_d = atan2(y_d_dot, x_d_dot);
for i = 1:length(x_d)/4
    theta_d(1,i) = atan2(y_d_dot(1,i), x_d_dot(1,i));
end    
for i = length(x_d)/4:length(x_d)
    theta_d(1,i) = atan2(y_d_dot(1,i), x_d_dot(1,i))+2*pi;
end
% Reference for v_d
v_d = sqrt(x_d_dot.^2 + y_d_dot.^2);
% Reference for δ_d (steering angle)
delta_d = atan(L * (y_d_d_dot .* x_d_dot - x_d_d_dot .* y_d_dot) ./ (v_d.^3));
% Reference for δ_dot_d (steering rate)
delta_d_dot = zeros(1,length(t));
for i = 1:length(t)-1
    delta_d_dot(1,i) = (delta_d(1,i+1) - delta_d(1,i)) / delta_t; 
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
plot(x_d, y_d, '-', 'Color', blue, 'linewidth', 1.5);
grid on;
box on;

ax = gca();
ax.TickLabelInterpreter = 'latex';
xlabel('position $x$ in $\mathrm{m}$', 'interpreter', 'latex');
ylabel('position $y$ in $\mathrm{m}$', 'interpreter', 'latex');

% save with matlab2tikz
cleanfigure('targetResolution', 300)
% matlab2tikz('figures/xy_reference.tex','width','\fwidth', 'encoding', 'utf8')

%% >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
% save reference
% >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
% save as csv for further use and easy import in C++
ref = [t', x_d', y_d', theta_d', delta_d', v_d', delta_d_dot'];
% round reference to reduce file size
ref = round(ref, 7);
% header line to explain columns
header = {'Time', 'x_d', 'y_d', 'theta_d', 'delta_d', 'v_d', 'delta_d_dot'};
output = [header; num2cell(ref)];
filename = mfilename('fullpath');
[filepath,~,~] = fileparts(filename);
writecell(output, [filepath,'/circle_ref.csv']); % introduced in Matlab 2019a

