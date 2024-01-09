addpath(fullfile('..', 'src'));

close all
clear all
clc

Ts = 1/40;

rocket = Rocket(Ts);

Tf = 10; 
rocket.mass = 1.75;

% senza compensazione
% per ritardo 2 con massa ridotta funziona bene
% per ritardo 3 sminchia ma non troppo, al limite della divergenza
% con quattro assolutamente instabile


% con compensazione
% delay = 3 expected 3 working, poor performance but still stable
% delay 4 expected 4 not stable


% partially -> delay 3 exp 1 perfetto 
% fully compensating -> 3 3 
%% No compensation - 3 steps delay / 0.075s
rocket.delay = 3;
expected_delay = 0;

H = 1; % Horizon length in seconds
nmpc = NmpcControl(rocket, H, expected_delay);
    
x0 = zeros(12,1);
ref = [0.5, 0, 1, deg2rad(65)]';


[T, X, U, Ref] = rocket.simulate(x0, Tf, @nmpc.get_u, ref);

% Visualize
rocket.anim_rate = 10; % Increase this to make the animation faster
ph = rocket.plotvis(T, X, U, Ref);
ph.fig.Name = 'No compensation - 3 steps delay / 0.075s'; 
saveas(ph.fig,'img/3_0.png')

%% No compensation - 4 steps delay / 0.1s
rocket.delay = 4;
expected_delay = 0;

H = 1; % Horizon length in seconds
nmpc = NmpcControl(rocket, H, expected_delay);
    
x0 = zeros(12,1);
ref = [0.5, 0, 1, deg2rad(65)]';


[T, X, U, Ref] = rocket.simulate(x0, Tf, @nmpc.get_u, ref);

% Visualize
rocket.anim_rate = 10; % Increase this to make the animation faster
ph = rocket.plotvis(T, X, U, Ref);
ph.fig.Name = 'No compensation - 4 steps delay / 0.1s'; 
saveas(ph.fig,'img/4_0.png')

%% Partially compensating controller - 3 steps delay 0.075s - 1 step compensation 0.025s
rocket.delay = 3;
expected_delay = 1;

H = 1; % Horizon length in seconds
nmpc = NmpcControl(rocket, H, expected_delay);
    
x0 = zeros(12,1);
ref = [0.5, 0, 1, deg2rad(65)]';


[T, X, U, Ref] = rocket.simulate(x0, Tf, @nmpc.get_u, ref);

% Visualize
rocket.anim_rate = 10; % Increase this to make the animation faster
ph = rocket.plotvis(T, X, U, Ref);
ph.fig.Name = 'Partially compensating controller - 3 steps delay 0.075s - 1 step compensation 0.025s'; 
saveas(ph.fig,'img/3_1.png')

%% Fully compensating controller - 3 steps delay and compensation 0.075s
% 4 1 una merda
rocket.delay = 3;
expected_delay = 3;

H = 1; % Horizon length in seconds
nmpc = NmpcControl(rocket, H, expected_delay);
    
x0 = zeros(12,1);
ref = [0.5, 0, 1, deg2rad(65)]';


[T, X, U, Ref] = rocket.simulate(x0, Tf, @nmpc.get_u, ref);

% Visualize
rocket.anim_rate = 10; % Increase this to make the animation faster
ph = rocket.plotvis(T, X, U, Ref);
ph.fig.Name = 'Fully compensating controller - 3 steps delay and compensation 0.075s'; 
saveas(ph.fig,'img/3_3.png')
