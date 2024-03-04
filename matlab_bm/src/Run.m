clear
clc
close all

% reference
trajectory = trajectory();
trajectory = trajectory.read_ref("uturn.csv");
% trajectory = trajectory.read_ref("straight.csv");
% trajectory = trajectory.read_ref("circle_ref.csv");
% trajectory = trajectory.read_ref("lemniscate_of_gerono_ref.csv");

% plants
model = HakuroukunCLASS();
model.wheel_slip = 0.0;
model = model.Init();

x0 = [0.0; 0.1; pi/10];
% x0 = trajectory.state_ref(:,1);

% controller
% controller = FeedForwardCLASS(trajectory);
controller = ModelPredictiveControlCLASS(model, trajectory);
controller = controller.Init();

% observer
measurement_mode = false;    % true, false
observer = NormalObserverCLASS(model, measurement_mode);
% observer = EKFObserverCLASS(model, trajectory, measurement_mode);
observer = observer.Init(x0);
simulation = TimeSteppingCLASS(trajectory, model, controller, observer);

% Loop
simulation = simulation.Loop(x0);
% %% =======================================================================
% % xy-plot
% grey = ReadColor("grey");
% red = ReadColor("red");
% green = ReadColor("green");
% blue = ReadColor("blue");
% 
% f1 = figure(1);
% f1.Color = 'w';
% hold on
% plot(trajectory.state_ref(1,:), trajectory.state_ref(2,:), '--', 'Color', grey, 'linewidth', 1.5);
% plot(simulation.y_out(1,:), simulation.y_out(2,:), '-', 'Color', green, 'linewidth', 1.5);
% xlim([-1 6]);
% ylim([-0.5 4]);
% 
% ax = gca();
% ax.TickLabelInterpreter = 'latex';
% xlabel('position $x$ ($\mathrm{m}$)', 'interpreter', 'latex');
% ylabel('position $y$ ($\mathrm{m}$)', 'interpreter', 'latex');
% legend('reference', 'robot state', 'real state', 'interpreter', 'latex', 'orientation','vertical',...
%                                             'location','southeast');
% grid on;
% box on;
% hold off;

% % f2 = figure(2);
% % f2.Color = 'w';
% % hold on
% % plot(trajectory.t_ref,simulation.y_out(1,:) - trajectory.state_ref(1,:), '-', 'Color', blue, 'linewidth', 1.5);
% % xlim([0 24]);
% % ylim([-0.3 0.3]);
% % ax = gca();
% % ax.TickLabelInterpreter = 'latex';
% % xlabel('time $t$ ($\mathrm{s}$)', 'interpreter', 'latex');
% % ylabel('longitudinal error ($\mathrm{m}$)', 'interpreter', 'latex');
% % grid on;
% % box on;
% % hold off;
% % 
% % f3 = figure(3);
% % f3.Color = 'w';
% % hold on;
% % plot(trajectory.t_ref,simulation.y_out(2,:) - trajectory.state_ref(2,:), '-', 'Color', blue, 'linewidth', 1.5);
% % xlim([0 24]);
% % ylim([-0.3 0.3]);
% % ax = gca();
% % ax.TickLabelInterpreter = 'latex';
% % xlabel('time $t$ ($\mathrm{s}$)', 'interpreter', 'latex');
% % ylabel('lateral error ($\mathrm{m}$)', 'interpreter', 'latex');
% % grid on;
% % box on;
% % hold off;
% 
% % f4 = figure(4);
% % f4.Color = 'w';
% % hold on;
% % plot(trajectory.t_ref,trajectory.input_ref(1,:), '--', 'Color', red, 'linewidth', 1.5);
% % plot(trajectory.t_ref,simulation.u_out(1,:), '-', 'Color', blue, 'linewidth', 1.5);
% % xlim([0 24]);
% % % ylim([-0.3 0.3]);
% % ax = gca();
% % ax.TickLabelInterpreter = 'latex';
% % xlabel('time $t$ ($\mathrm{s}$)', 'interpreter', 'latex');
% % ylabel('velocity $\mathrm{v}$ ($\mathrm{m/s}$)', 'interpreter', 'latex');
% % legend('reference', 'real value', 'interpreter', 'latex', 'orientation','vertical',...
% %                                             'location','southeast');
% % grid on;
% % box on;
% % hold off;
% % 
% % f5 = figure(5);
% % f5.Color = 'w';
% % hold on;
% % plot(trajectory.t_ref,trajectory.input_ref(2,:), '--', 'Color', red, 'linewidth', 1.5);
% % plot(trajectory.t_ref,simulation.u_out(2,:), '-', 'Color', blue, 'linewidth', 1.5);
% % xlim([0 24]);
% % % ylim([-0.3 0.3]);
% % ax = gca();
% % ax.TickLabelInterpreter = 'latex';
% % xlabel('time $t$ ($\mathrm{s}$)', 'interpreter', 'latex');
% % ylabel('angular velocity ($\mathrm{m/s}$)', 'interpreter', 'latex');
% % legend('reference', 'real value', 'interpreter', 'latex', 'orientation','vertical',...
% %                                             'location','southeast');
% % grid on;
% % box on;
% % hold off;
% % 
