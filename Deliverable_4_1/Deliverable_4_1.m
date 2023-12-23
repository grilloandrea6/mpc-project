addpath(fullfile('..', 'src'));

close all
clear all
clc

% Constant definition
Ts = 1/20; % Sampling time
H = 5; % Horizon length in seconds
Tf = 20; % Simulation time

rocket = Rocket(Ts);

[xs, us] = rocket.trim(); % Compute steady−state for which 0 = f(xs,us)
sys = rocket.linearize(xs, us); % Linearize the nonlinear model about trim point
[sys_x, sys_y, sys_z, sys_roll] = rocket.decompose(sys, xs, us); % Decompose the system in 4 independent systems

% Define MPC controllers for the 4 subsystem
mpc_x = MpcControl_x(sys_x, Ts, H);
mpc_y = MpcControl_y(sys_y, Ts, H);
mpc_z = MpcControl_z(sys_z, Ts, H);
mpc_roll = MpcControl_roll(sys_roll, Ts, H);

% Merge four sub−system controllers into one full−system controller
mpc = rocket.merge_lin_controllers(xs, us, mpc_x, mpc_y, mpc_z, mpc_roll);

% Evaluate once and plot optimal open−loop trajectory,
% pad last input to get consistent size with time and state
x0 = zeros(12,1);
ref4 = [2 2 2 deg2rad(40)]';
[u, T_opt, X_opt, U_opt] = mpc.get_u(x0, ref4);
U_opt(:,end+1) = nan;

ph = rocket.plotvis(T_opt, X_opt, U_opt, ref4); % Plot as usual
ph.fig.Name = 'Linear MPC in nonlinear system - Open Loop simulation'; 

% Setup reference function
ref = @(t_, x_) ref_TVC(t_);

[T, X, U, Ref] = rocket.simulate(x0, Tf, @mpc.get_u, ref);

% Visualize
rocket.anim_rate = 10; % Increase this to make the animation faster
ph = rocket.plotvis(T, X, U, Ref);
ph.fig.Name = 'Linear MPC in nonlinear system - Closed Loop simulation'; 