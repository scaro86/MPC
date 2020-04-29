% Init
clear all
close all
addpath(genpath(cd));
load('system/parameters_scenarios.mat');

param = compute_controller_base_parameters;
%% Exercise 5: execute simulation with LQR
% clear persisten variables of function controller_lqr
clear controller_lqr; 
% execute simulation starting from T0_1 using lqr controller with scenario 1
x0_1 = [3; 1; 0];
T0_1 = param.T_sp + x0_1;
[T, p] = simulate_truck(T0_1, @controller_lqr, scen1);
% check if the reference is approached reasonably fast
dist = norm(param.T_sp - T(:,30));
if dist > 0.2*norm(x0_1)
    disp('your control is too slow')
else
    disp('your control is perfect')
end
%% Exercise 6: Compute the infinite horizon cost under the LQR control law
K = -dlqr(param.A,param.B,param.Q,param.R);
costlqr = 0;
x = x0_1;
for i = 1:1000
    costlqr = costlqr + x0_1'*(param.Q + K'*param.R*K)*x;
    x = (param.A-param.B*K)*x;
end
fprintf('Cost of the optimal LQR controller: %.2f\n',costlqr);

%% Exercise 7 : closed loop simulation of LQR controller
x0_2 = [-1 -0.3 -4.5]';
T0_2 = param.T_sp + x0_2;
[T, p] = simulate_truck(T0_2, @controller_lqr, scen1);

%% Exercise 8 : Computation of X_lqr set
