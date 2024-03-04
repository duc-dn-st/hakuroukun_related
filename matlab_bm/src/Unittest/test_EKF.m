clear
clc
close all

% reference
trajectory = trajectory();
trajectory = trajectory.read_ref("uturn.csv");

% plants
model = BicycleModelCLASS();
model.name = "bicycle_slip";
model.wheel_slip = 0.2;
model = model.Init();

x0 = [0.0; 0.0; trajectory.state_ref(3,1); trajectory.state_ref(4,1)];
% x0 = trajectory.state_ref(:,1);

% controller
controller = FeedForwardCLASS(trajectory);
% controller = ModelPredictiveControlCLASS(model, trajectory);
controller = controller.Init();

t_start = 0;
t_end   = 20;
dt     = 0.05;

t_out = linspace(t_start, t_end, (1/dt) * t_end);
n_t = length(t_out); 
x_out = zeros(model.nx+1,n_t);
y_out = zeros(model.nx+1,n_t);
u_out = zeros(model.nu,n_t);

% observer
observe_mode = true;    % true, false
% observer = NormalObserverCLASS(model,observe_mode);
observer = EKFObserverCLASS(model, trajectory ,observe_mode);
observer = observer.Init(x0);

[controller, status, u_out(:,1)] = controller.Run(dt, y_out(:,1), 1);

if ~status
    u_out(:,1) = zeros(model.nu,1);
end

if model.name == "bicycle"
    xM = model.f_d(y_out(:,1), u_out(:,1), dt, model.p);
elseif model.name == "bicycle_slip"
    xM = model.f_d_slip(y_out(:,1), u_out(:,1), dt, model.p);
end

[observer.measurement, yM] = observer.measurement.Observe(xM);

observer = observer.PredictionSTEP(u_out(:,1));

observer = observer.CorrectionSTEP(yM);

observe_output = observer.x_tilde_k'*observer.C_d';


% % xy-plot
% grey = [0.2431, 0.2431, 0.2980];
% red = [0.7529, 0.0000, 0.000];
% green = [0.0000, 0.6902, 0.3137];
% blue = [0.0000, 0.3176, 0.6196];
% 
% f1 = figure(1);
% f1.Color = 'w';
% hold on
% plot(trajectory.state_ref(1,:), trajectory.state_ref(2,:), '--', 'Color', grey, 'linewidth', 1.5);
% plot(simulation.y_out(1,:), simulation.y_out(2,:), '-', 'Color', green, 'linewidth', 1.5);
% grid on;
% box on;
% hold off;
