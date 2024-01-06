addpath(fullfile('..', 'src'));

close all
clear all
clc

Ts = 1/40;

rocket = Rocket(Ts);

Tf = 2.5;
rocket.mass = 1.75;
rocket.delay = 5;
expected_delay = 5;

H = 4; % Horizon length in seconds
nmpc = NmpcControl(rocket, H, expected_delay);
    
x0 = zeros(12,1);
ref = [0.5, 0, 1, deg2rad(65)]';


[T, X, U, Ref] = rocket.simulate(x0, Tf, @nmpc.get_u, ref);

% Visualize
rocket.anim_rate = 10; % Increase this to make the animation faster
ph = rocket.plotvis(T, X, U, Ref);
ph.fig.Name = 'TODO TODO TODO'; 
