# mpc-project
Model Predictive Control - project

Deliverable 2.1 
 the matrices of the system can be decoupled because the variables of the directions are independent between each other. As we can see from matrices A, B, C, D, all the values of component that relate variable x and variable y are set to 0. This is valid for every direction.

 phisical\mechanical intuition: 
 the position x depends from delta2
 position y depends on delta1
 position z is dependent on Pavg
 roll dependant on P_diff
 therefore, the state variables can be controlled indipendently 

5.1 

steady state target. 
added the presence of the disturbance in the model
xs = A*xs+ B*us +B* d_est
ref = C*xs

controller: 
added the presence of the disturbance in the model: 
x = A*x+ B*us +B* d_est

added the estimator: 
A_bar = [mpc.A mpc.B; zeros(1,nx) 1];
B_bar = [mpc.B; zeros(1,nu)];
C_bar = [mpc.C 0];


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

if you let the simulation run for more the simulation stops because the problem becomes unfeasible, the rocket runs out of fuel and can't provide the minimum thurst set by the constraints
