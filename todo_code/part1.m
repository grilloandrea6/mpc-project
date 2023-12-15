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
