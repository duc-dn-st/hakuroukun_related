clear 
close all
clc

dt = 0.05;

% reference
trajectory = trajectory();
trajectory = trajectory.read_ref("uturn.csv");

% controller
controller = FeedForwardCLASS(trajectory);

% plant
model = HakuroukunCLASS();
model.wheel_slip = 0.0;
model = model.Init();

% observer
observe_mode = false;    % true, false
observer = NormalObserverCLASS(model,observe_mode);
observer = observer.Init();

% initial position
x_out(:,1)    = [0;0;0];
y_out(:,1)    = [0;0;0];
u_out = trajectory.input_ref(:,1);

x_out = model.f_d_slip(y_out(:,1), u_out, dt, model.p);

% run
for i=1:length(trajectory.t_ref)
    [controller, status, u_out] = controller.Run(controller, y_out(:,i), i);

    xM = model.f_d_slip(y_out(:,i), u_out, dt, model.p);
    
    yM = observer.Observe(xM); 
    
    if i < length(trajectory.t_ref)
        x_out(:, i+1) = xM;
        y_out(:, i+1) = yM;
    end
end
%% ========================================================================
% xy-plot
grey = ReadColor("grey");
red = ReadColor("red");
green = ReadColor("green");
blue = ReadColor("blue");

f1 = figure(1);
f1.Color = 'w';
hold on
% plot(x_out(1,:), x_out(2,:), '-', 'Color', green, 'linewidth', 1.5);
plot(y_out(1,:), y_out(2,:), '-', 'Color', blue, 'linewidth', 1.5);
plot(trajectory.state_ref(1,:), trajectory.state_ref(2,:), '--', 'Color', grey, 'linewidth', 1.5);
grid on;
box on;
hold off;
