clear
clc
close all

trajectory = trajectory();
trajectory = trajectory.read_ref("uturn.csv");

model = HakuroukunCLASS();
model.wheel_slip = 0.1;
model = model.Init();

controller = ModelPredictiveControlCLASS(model, trajectory);
controller = controller.Init();

iterations = length(controller.trajectory.state_ref) - controller.N; 
mearsure_output = [0;0;0];
dt = 0.05;

observe_mode = false;    % true, false
observer = NormalObserverCLASS(model,observe_mode);

x_out = zeros(model.nx,260);
y_out = zeros(model.nx,260);
u_out = zeros(model.nu,260);

x_out(:,1) = [0;0;0];
y_out(:,1) = [0;0;0];
N = 10;


[status, u_out(:,1)] = controller.Run(dt, y_out(:,1), 1);

for i=1:250
    
    [status, u_out(:,i)] = controller.Run(dt, y_out(:,i), i);
    
    xM = model.f_d_slip(y_out(:,i), u_out(:,i), dt, model.p);

    yM = observer.Observe(xM); 

    if i < 260
        x_out(:, i+1) = xM;
        y_out(:, i+1) = yM;
    end
end

grey = [0.2431, 0.2431, 0.2980];
red = [0.7529, 0.0000, 0.000];
green = [0.0000, 0.6902, 0.3137];
blue = [0.0000, 0.3176, 0.6196];

f1 = figure(1);
f1.Color = 'w';
hold on
plot(x_out(1,:), x_out(2,:), '-', 'Color', green, 'linewidth', 1.5);
plot(trajectory.state_ref(1,:), trajectory.state_ref(2,:), '--', 'Color', grey, 'linewidth', 1.5);
grid on;
box on;
hold off;