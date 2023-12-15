addpath(fullfile('..', 'src'));

close all
clear all
clc

%% TODO: This file should produce all the plots for the deliverable
% rocket = Rocket(1/20);
% 
% d1 = 0;
% d2 = 0;
% Pavg = 100;
% Pdiff = -2;
% 
% 
% u = [d1, d2, Pavg, Pdiff]'; % (Assign appropriately)
% 
% 
% [b_F, b_M] = rocket.getForceAndMomentFromThrust(u)
% 
% w = [0 0 0];
% phi = [0 0 0];
% v = [0 0 0];
% p = [0 0 0];
% x = [w, phi, v, p]'; % (Assign appropriately)
% 
% 
% for i = 1:10
%     disp(i)
%     x_dot = rocket.f(x, u);
%     x = x + x_dot/20;
% 
%     [w, phi, v, p] = rocket.parse_state(x)
% end
% 

% Tf = 20.0; % Simulation end time
% x0 = [deg2rad([0 0 0, 0 0 0]), 0 0 0, 0 0 0]'; % (w, phi, v, p) Initial state
% u = [deg2rad([0 0]), 62.61, 0 ]'; % (d1 d2 Pavg Pdiff) Constant input
% [T, X, U] = rocket.simulate(x0, Tf, u); % Simulate unknown, nonlinear model
% rocket.anim_rate = 1; %1.0; % Visualize at 1.0x real−time
% rocket.vis(T, X, U);

Ts = 1/20;
rocket = Rocket(Ts);

[xs, us] = rocket.trim(); % Compute steady−state for which 0 = f(xs,us)
sys = rocket.linearize(xs, us); % Linearize the nonlinear model about trim point
[sys_x, sys_y, sys_z, sys_roll] = rocket.decompose(sys, xs, us); % Decompose the system in 4 independent systems


% Design MPC controller
H = 5; % Horizon length in seconds
mpc_x = MpcControl_x(sys_x, Ts, H);
% Get control input ( x is the index of the subsystem here)
u_x = mpc_x.get_u(x_x);


% Evaluate once and plot optimal open−loop trajectory,
% pad last input to get consistent size with time and state
[u, T_opt, X_opt, U_opt] = mpc_x.get_u(x);
U_opt(:,end+1) = NaN;
% Account for linearization point
X_opt = X_opt + xs(2,4,7,10);
U_opt = U_opt + us(2);
ph = rocket.plotvis_sub(T_opt, X_opt, U_opt, sys_x, xs, us); % Plot as usual