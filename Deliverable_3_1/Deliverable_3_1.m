addpath(fullfile('..', 'src'));

close all
clear all
clc

%% TODO: This file should produce all the plots for the deliverable

Ts = 1/20;
rocket = Rocket(Ts);

[xs, us] = rocket.trim(); % Compute steady−state for which 0 = f(xs,us)
sys = rocket.linearize(xs, us); % Linearize the nonlinear model about trim point
[sys_x, sys_y, sys_z, sys_roll] = rocket.decompose(sys, xs, us); % Decompose the system in 4 independent systems


% Design MPC controller
H = .8; % Horizon length in seconds
mpc_x = MpcControl_x(sys_x, Ts, H);


% Evaluate once and plot optimal open−loop trajectory,
% pad last input to get consistent size with time and state

% start from 3 meters
x_x_0 = [0 0 0 3]';

[u, T_opt, X_opt, U_opt] = mpc_x.get_u(x_x_0);
U_opt(:,end+1) = NaN;
% Account for linearization point
X_opt = X_opt + xs([2,4,7,10]);
U_opt = U_opt + us(2);
rocket.plotvis_sub(T_opt, X_opt, U_opt, sys_x, xs, us); % Plot as usual


Tf = 10;
[T, X_sub, U_sub] = rocket.simulate_f(sys_x, x_x_0, Tf, @mpc_x.get_u, 0);
rocket.plotvis_sub(T, X_sub, U_sub, sys_x, xs, us);