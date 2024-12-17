close all
clear

addpath('../car_project/')

Ts = 1/10;

car = Car(Ts);
[xs, us] = car.steady_state(120 / 3.6);
sys = car.linearize(xs, us);
[sys_lon, sys_lat] = car.decompose(sys);

% Design MPC controller
H_lat = 2; % Horizon length in seconds
H_lon = H_lat;

mpc_lat = MpcControl_lat(sys_lat, Ts, H_lat);
mpc_lon = MpcControl_lon(sys_lon, Ts, H_lon);

mpc = car.merge_lin_controllers(mpc_lon, mpc_lat);

%% Simulation
estimator = LonEstimator(sys_lon, Ts);

% reference
x0 = [0 0 0 80/3.6]'; % (x, y, theta, V)
ref1 = [0 80/3.6]'; % (y ref, V ref)
ref2 = [3 50/3.6]'; % (y ref, V ref)

params = {};
params.Tf = 15;
params.myCar.model = car;
params.myCar.x0 = x0;
params.myCar.est_fcn = @estimator.estimate;
params.myCar.est_dist0 = 0;
params.myCar.u = @mpc.get_u;
params.myCar.ref = car.ref_step(ref1, ref2, 2); % delay reference step by 5s
result = simulate(params);
visualization(car, result);