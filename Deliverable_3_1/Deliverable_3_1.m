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
Tf = 10;

mpc_x = MpcControl_x(sys_x, Ts, H);
mpc_y = MpcControl_y(sys_y, Ts, H);
mpc_z = MpcControl_z(sys_z, Ts, H);
mpc_roll = MpcControl_roll(sys_roll, Ts, H);


% Define initial state
% start from 3 meters for x,y,z
x_x_0 = [0 0 0 3]';
x_y_0 = [0 0 0 3]';
x_z_0 = [0 3]';
% start from 40deg for roll
x_roll_0 = [0 deg2rad(40)]';

%% controller X
[~, T_opt, X_opt, U_opt] = mpc_x.get_u(x_x_0);
U_opt(:,end+1) = NaN;
% Account for linearization point
X_opt = X_opt + xs([2,5,7,10]);
U_opt = U_opt + us(2);
% open loop plot
ph = rocket.plotvis_sub(T_opt, X_opt, U_opt, sys_x, xs, us);
ph.fig.Name = "Controller X - Open Loop";
saveas(ph.fig,'img/xopen.png')

% closed loop plot
[T, X_sub, U_sub] = rocket.simulate_f(sys_x, x_x_0, Tf, @mpc_x.get_u, 0);
ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_x, xs, us);
ph.fig.Name = "Controller X - Closed Loop";
saveas(ph.fig,'img/xclosed.png')

%% controller Y
[~, T_opt, X_opt, U_opt] = mpc_y.get_u(x_y_0);
U_opt(:,end+1) = NaN;
% Account for linearization point
X_opt = X_opt + xs([1,4,8,11]);
U_opt = U_opt + us(1);
% open loop plot
ph = rocket.plotvis_sub(T_opt, X_opt, U_opt, sys_y, xs, us); % Plot as usual
ph.fig.Name = "Controller Y - Open Loop";
saveas(ph.fig,'img/yopen.png')

% closed loop plot
[T, X_sub, U_sub] = rocket.simulate_f(sys_y, x_y_0, Tf, @mpc_y.get_u, 0);
ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_y, xs, us);
ph.fig.Name = "Controller Y - Closed Loop";
saveas(ph.fig,'img/yclosed.png')

%% controller Z
[~, T_opt, X_opt, U_opt] = mpc_z.get_u(x_z_0);
U_opt(:,end+1) = NaN;
% Account for linearization point
X_opt = X_opt + xs([9,12]);
U_opt = U_opt + us(3);
% open loop plot
ph = rocket.plotvis_sub(T_opt, X_opt, U_opt, sys_z, xs, us); % Plot as usual
ph.fig.Name = "Controller Z - Open Loop";
saveas(ph.fig,'img/zopen.png')

% closed loop plot
[T, X_sub, U_sub] = rocket.simulate_f(sys_z, x_z_0, Tf, @mpc_z.get_u, 0);
ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_z, xs, us);
ph.fig.Name = "Controller Z - Closed Loop";
saveas(ph.fig,'img/zclosed.png')


%% controller roll
[u, T_opt, X_opt, U_opt] = mpc_roll.get_u(x_roll_0);
U_opt(:,end+1) = NaN;
% Account for linearization point
X_opt = X_opt + xs([3,6]);
U_opt = U_opt + us(4);
% open loop plot
ph = rocket.plotvis_sub(T_opt, X_opt, U_opt, sys_roll, xs, us); % Plot as usual
ph.fig.Name = "Controller roll - Open Loop";
saveas(ph.fig,'img/rollopen.png')

% closed loop plot
[T, X_sub, U_sub] = rocket.simulate_f(sys_roll, x_roll_0, Tf, @mpc_roll.get_u, 0);
ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_roll, xs, us);
ph.fig.Name = "Controller roll - Closed Loop";
saveas(ph.fig,'img/rollclosed.png')
