# mpc-project
Model Predictive Control - project

5.1 
just added the estimator: 
A_bar = [mpc.A mpc.B; zeros(1,nx) 1];
B_bar = [mpc.B; zeros(1,nu)];
C_bar = [mpc.C 0];
% need to check what is mpc.D, if it appears somewhere

% can compute L by using the LQR, 
% system : error_i+1= (A'+C'*L) error, Optimal L given Q, R
% more intuitive to tune Q, R rather than tune eigenvalues
R = .1;
L  = dlqr(A_bar', C_bar', eye(nx+1), R);
L= -L';

% alternative is to use poleplacement
%L = -place(A_pred',C_pred',[0.5 0.6 0.7]); % controller for x pred
%L = L';


% need to tune parameters and add plots


5.2
dynamic mass:
just added the lines given in the description of the exercise

the rocket diverges in z because as the fuel is consumed the rocket gets lighter and constraint of minimum thrust is of 50% which is above the necessary value to constrast gravity.
