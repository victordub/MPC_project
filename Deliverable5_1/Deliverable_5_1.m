close all
clear

addpath('../car_project/')

run("tube_mpc_sets.m");

%% Define mpc
Ts = 1/10;

car = Car(Ts);
[xs, us] = car.steady_state(120 / 3.6);
sys = car.linearize(xs, us);
[sys_lon, sys_lat] = car.decompose(sys);

% Design MPC controller
H_lat = 5; % Horizon length in seconds
H_lon = H_lat;

mpc_lat = MpcControl_lat(sys_lat, Ts, H_lat);
mpc_lon = MpcControl_lon(sys_lon, Ts, H_lon);

mpc = car.merge_lin_controllers(mpc_lon, mpc_lat);

%% Run simulation

otherRef = 100 / 3.6;

ref = [0 100/3.6]';
params = {};
params.Tf = 25;
params.myCar.model = car;
params.myCar.x0 = [0 0 0 100/3.6]';
params.myCar.u = @mpc.get_u;
params.myCar.ref = ref;
params.otherCar.model = car;
params.otherCar.x0 = [15 0 0 otherRef]';
params.otherCar.u = car.u_const(otherRef);
result = simulate(params);
visualization(car, result);
