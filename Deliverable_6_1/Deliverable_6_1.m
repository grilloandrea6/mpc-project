addpath(fullfile('..', 'src'));

close all
clear all
clc

Ts = 1/20; % Sampling time
H = 1;     % Horizon length in seconds
Tf = 30;   % Closed loop simulation time

rocket = Rocket(Ts);

% Nonlinear MPC controller
nmpc = NmpcControl(rocket, H);

% Linearization and subsystem splitting
[xs, us] = rocket.trim(); % Compute steady−state for which 0 = f(xs,us)
sys = rocket.linearize(xs, us); % Linearize the nonlinear model about trim point
[sys_x, sys_y, sys_z, sys_roll] = rocket.decompose(sys, xs, us); % Decompose the system in 4 independent systems

% Linear MPC controllers for the 4 subsystems
mpc_x = MpcControl_x(sys_x, Ts, H);
mpc_y = MpcControl_y(sys_y, Ts, H);
mpc_z = MpcControl_z(sys_z, Ts, H);
mpc_roll = MpcControl_roll(sys_roll, Ts, H);
% Merge four sub−system controllers into one full−system controller
mpc = rocket.merge_lin_controllers(xs, us, mpc_x, mpc_y, mpc_z, mpc_roll);

% Initial state for all the simulations
x0 = zeros(12,1);

%% Open loop nonlinear controller simulation
ref = -ones(4,1);
% Evaluate once and plot optimal open−loop trajectory,
% pad last input to get consistent size with time and state
[u, T_opt, X_opt, U_opt] = nmpc.get_u(x0, ref);
U_opt(:,end+1) = nan;
ph = rocket.plotvis(T_opt, X_opt, U_opt, ref);
ph.fig.Name = 'Open-loop nonlinear MPC.';
saveas(ph.fig,'img/openloop.png')

%% Nonlinear MPC simulation for 15 degree maximum roll
% MPC reference with default maximum roll = 15 deg
ref = @(t_, x_) ref_TVC(t_);

[T, X, U, Ref] = rocket.simulate(x0, Tf, @nmpc.get_u, ref);

% Visualize
rocket.anim_rate = 10; % Increase this to make the animation faster
ph = rocket.plotvis(T, X, U, Ref);
ph.fig.Name = 'Nonlinear MPC in closed loop - 15deg maximum roll.';
saveas(ph.fig,'img/nonlinear15.png')

%% Nonlinear MPC simulation for 50 degree maximum roll
% MPC reference with specified maximum roll = 50 deg
roll_max = deg2rad(50);
ref = @(t_, x_) ref_TVC(t_, roll_max);

[T, X, U, Ref] = rocket.simulate(x0, Tf, @nmpc.get_u, ref);

% Visualize 
rocket.anim_rate = 10; % Increase this to make the animation faster
ph = rocket.plotvis(T, X, U, Ref);
ph.fig.Name = 'Nonlinear MPC in closed loop - 50deg maximum roll.';
saveas(ph.fig,'img/nonlinear50.png')

%% Linear MPC simulation for 50 degree maximum roll

[T, X, U, Ref] = rocket.simulate(x0, Tf, @mpc.get_u, ref);

% Visualize
rocket.anim_rate = 10; % Increase this to make the animation faster
ph = rocket.plotvis(T, X, U, Ref);
ph.fig.Name = 'Linear MPC in closed loop - 50deg maximum roll.';
saveas(ph.fig,'img/linear50.png')