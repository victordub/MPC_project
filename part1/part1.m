close all
clear

addpath('../car_project/')

Ts = 1/10;
car = Car(Ts);
Tf = 2;

x0 = [0, 0, deg2rad(-2), 20/3.6]'; % (x, y, theta, V) Initial state
u = [deg2rad(-5), -0.3]'; % (delta, u T) Constant input

params = {}; % Setup simulation parameter struct
params.Tf = Tf;
params.myCar.model = car;
params.myCar.x0 = x0;
params.myCar.u = u;
result = simulate(params); % Simulate nonlinear model
visualization(car, result);
