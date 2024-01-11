addpath(fullfile('..', 'src'));

close all
clear all
clc

Ts = 1/40; % Sampling time
H = 1;    % Horizon length in seconds

rocket = Rocket(Ts);

Tf = 2.5; 
rocket.mass = 1.75;

x0 = zeros(12,1);
ref = [0.5, 0, 1, deg2rad(65)]';

%% Partially compensating controller - 5 steps delay 0.125s - 1 step compensation 0.025s
rocket.delay = 5;
expected_delay = 1;

nmpc = NmpcControl(rocket, H, expected_delay);

[T, X, U, Ref] = rocket.simulate(x0, Tf, @nmpc.get_u, ref);

% Visualize
rocket.anim_rate = 10; % Increase this to make the animation faster
ph = rocket.plotvis(T, X, U, Ref);
ph.fig.Name = 'Partially compensating controller - 5 steps delay 0.125s - 1 step compensation 0.025s'; 
saveas(ph.fig,'img/5_1.png')

%% Fully compensating controller - 3 steps delay and compensation 0.075s
rocket.delay = 3;
expected_delay = 3;

nmpc = NmpcControl(rocket, H, expected_delay);

[T, X, U, Ref] = rocket.simulate(x0, Tf, @nmpc.get_u, ref);

% Visualize
rocket.anim_rate = 10; % Increase this to make the animation faster
ph = rocket.plotvis(T, X, U, Ref);
ph.fig.Name = 'Fully compensating controller - 3 steps delay and compensation 0.075s'; 
saveas(ph.fig,'img/3_3.png')
