addpath(fullfile('..', 'src'));

close all
clear all
clc

%% TODO: This file should produce all the plots for the deliverable

Ts = 1/20;
rocket = Rocket(Ts);
H = 1.5; % Horizon length in seconds
nmpc = NmpcControl(rocket, H);

% MPC reference with default maximum roll = 15 deg

% MPC reference with specified maximum roll = 50 deg
% roll_max = deg2rad(50);
% ref = @(t_, x_) ref_TVC(t_, roll_max);

x0 = zeros(12,1);
ref = -ones(4,1);
% Evaluate once and plot optimal open−loop trajectory,
% pad last input to get consistent size with time and state
[u, T_opt, X_opt, U_opt] = nmpc.get_u(x0, ref);
U_opt(:,end+1) = nan;
ph = rocket.plotvis(T_opt, X_opt, U_opt, ref);
ph.fig.Name = 'Open-loop nonlinear MPC.';


Tf = 30 ;
ref = @(t_, x_) ref_TVC(t_);

[T, X, U, Ref] = rocket.simulate(x0, Tf, @nmpc.get_u, ref);

% Visualize
rocket.anim_rate = 10; % Increase this to make the animation faster
ph = rocket.plotvis(T, X, U, Ref);
ph.fig.Name = 'Nonlinear MPC in closed loop.'; % Set a figure title
