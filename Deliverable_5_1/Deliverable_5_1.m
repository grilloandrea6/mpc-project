addpath(fullfile('..', 'src'));

close all
clear all
clc

Ts = 1/20;
rocket = Rocket(Ts);

[xs, us] = rocket.trim(); % Compute steady−state for which 0 = f(xs,us)
sys = rocket.linearize(xs, us); % Linearize the nonlinear model about trim point
[sys_x, sys_y, sys_z, sys_roll] = rocket.decompose(sys, xs, us); % Decompose the system in 4 independent systems


% Design MPC controller
H = 1; % Horizon length in seconds

mpc_x = MpcControl_x(sys_x, Ts, H);
mpc_y = MpcControl_y(sys_y, Ts, H);
mpc_z = MpcControl_z(sys_z, Ts, H);
mpc_z_est = MpcControl_z_est(sys_z, Ts, H); 
mpc_roll = MpcControl_roll(sys_roll, Ts, H);


% Merge four sub−system controllers into one full−system controller
mpc = rocket.merge_lin_controllers(xs, us, mpc_x, mpc_y, mpc_z, mpc_roll);
mpc_est = rocket.merge_lin_controllers(xs, us, mpc_x, mpc_y, mpc_z_est, mpc_roll);

x0 = [zeros(1, 9), 1, 0, 3]' ;
ref = [1.2, 0, 3, 0]';

% Simulate
Tf = 8;

% Manipulate mass for simulation
rocket.mass = 2.13;


%% Simulation without state estimation
[T, X, U, Ref, ~] = rocket.simulate(x0, Tf, @mpc.get_u, ref);

% Visualize
rocket.anim_rate = 10; % Increase this to make the animation faster
ph = rocket.plotvis(T, X, U, Ref);
ph.fig.Name = 'Linear MPC in simulation with greater mass';
saveas(ph.fig,'img/no_estimator.png')


%% Simulation with state estimation
[T, X, U, Ref, Z_hat] = rocket.simulate_est_z(x0, Tf, @mpc_est.get_u, ref, mpc_z_est, sys_z);

% Visualize
rocket.anim_rate = 10; % Increase this to make the animation faster
ph = rocket.plotvis(T, X, U, Ref);
ph.fig.Name = 'Linear MPC in simulation with greater mass and z state estimator';
saveas(ph.fig, 'img/with_estimator.png')