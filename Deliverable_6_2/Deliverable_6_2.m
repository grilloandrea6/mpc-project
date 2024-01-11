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

%% No compensation - unstable system - 4 steps delay 0.1s
rocket.delay = 4;
expected_delay = 0;

nmpc = NmpcControl(rocket, H, expected_delay);

[T, X, U, Ref] = rocket.simulate(x0, Tf, @nmpc.get_u, ref);

% Visualize
rocket.anim_rate = 10; % Increase this to make the animation faster
ph = rocket.plotvis(T, X, U, Ref);
ph.fig.Name = 'No compensation - unstable system - 4 steps delay 0.1s'; 
saveas(ph.fig,'img/4_0.png')

%% No compensation - unstable system - 5 steps delay 0.125s
rocket.delay = 5;
expected_delay = 0;

nmpc = NmpcControl(rocket, H, expected_delay);

[T, X, U, Ref] = rocket.simulate(x0, Tf, @nmpc.get_u, ref);

% Visualize
rocket.anim_rate = 10; % Increase this to make the animation faster
ph = rocket.plotvis(T, X, U, Ref);
ph.fig.Name = 'No compensation - unstable system - 5 steps delay 0.125s'; 
saveas(ph.fig,'img/5_0.png')

%% Partially compensating controller - 8 steps delay 0.2s - 6 step compensation 0.15s
rocket.delay = 8;
expected_delay = 6;

nmpc = NmpcControl(rocket, H, expected_delay);

[T, X, U, Ref] = rocket.simulate(x0, Tf, @nmpc.get_u, ref);

% Visualize
rocket.anim_rate = 10; % Increase this to make the animation faster
ph = rocket.plotvis(T, X, U, Ref);
ph.fig.Name = 'Partially compensating controller - 8 steps delay 0.2s - 6 step compensation 0.15s'; 
saveas(ph.fig,'img/8_6.png')

%% Fully compensating controller - 8 steps delay and compensation 0.2s
rocket.delay = 8;
expected_delay = 8;

nmpc = NmpcControl(rocket, H, expected_delay);

[T, X, U, Ref] = rocket.simulate(x0, Tf, @nmpc.get_u, ref);

% Visualize
rocket.anim_rate = 10; % Increase this to make the animation faster
ph = rocket.plotvis(T, X, U, Ref);
ph.fig.Name = 'Fully compensating controller - 8 steps delay and compensation 0.2s'; 
saveas(ph.fig,'img/8_8.png')
