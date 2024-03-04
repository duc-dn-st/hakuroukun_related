model = BicycleModelCLASS();
model = model.Init();

trajectory = trajectory();
trajectory = trajectory.read_ref("uturn.csv");

x0_extended = [0;0;0;0; model.wheel_slip];
u = trajectory.input_ref(:,1);

dt = 0.05;

x1_extended = model.f_d_extended(x0_extended, u, dt, model.p);