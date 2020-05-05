% Init
clear all
close all
addpath(genpath(cd));
load('system/parameters_scenarios.mat');
param = compute_controller_base_parameters;

%% Exercise 5: execute simulation with LQR, x0_1
% clear persistent variables of function controller_lqr
clear controller_lqr; 
% execute simulation starting from T0_1 using lqr controller with scenario 1
x0_1 = [3; 1; 0];
T0_1 = param.T_sp + x0_1;
figure(1)
[T_LQR_1, p_LQR_1] = simulate_truck(T0_1, @controller_lqr, scen1);
% check if the reference is approached reasonably fast
dist = norm(param.T_sp - T_LQR_1(:,30));
if dist > 0.2*norm(x0_1)
    disp('your control is too slow')
else
    disp('your control is perfect')
end
%% Exercise 6: Compute the infinite horizon cost under the LQR control law
costlqr = compute_infinite_cost_LQR(x0_1);
fprintf('Cost of the optimal LQR controller: %.2f\n',costlqr);
%% Exercise 7 : closed loop simulation of LQR controller
x0_2 = [-1 -0.3 -4.5]';
T0_2 = param.T_sp + x0_2;
[T, p] = simulate_truck(T0_2, @controller_lqr, scen1);
% Big constraint violation from k=2 to k=53
%% Exercise 8 : Computation of X_lqr set
[A_x, b_x] = compute_X_LQR;

%% Exercise 9: execute simulation with MPC_1
param.calc_done = "false";
figure(3)
[T_MPC_11, p_MPC_11] = simulate_truck(T0_1, @controller_mpc_1, scen1);
param.calc_done = "false";
figure(4)
[T_MPC_12, p_MPC_12] = simulate_truck(T0_2, @controller_mpc_1, scen1);
%% Exercise 12: execute simulation with MPC_2
param.calc_done = "false";
figure(5)
[T_MPC_21, p_MPC_21] = simulate_truck(T0_1, @controller_mpc_2, scen1);