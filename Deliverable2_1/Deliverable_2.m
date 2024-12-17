close all
clear

addpath('../car_project/')

Ts = 1/10;

% Numerical linearization
car = Car(Ts);
Vs = 120/3.6; % 120 km/h
[xs, us] = car.steady_state(Vs); % Compute steady−state for which f s(xs,us) = 0
sys = car.linearize(xs, us); % Linearize the nonlinear model around xs, us
