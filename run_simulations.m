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
figure(1);
sgtitle('Simulation with LQR control, T01');
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
figure(2)
sgtitle('Simulation with LQR control, T02');
[T_LQR_2, p_LQR_2] = simulate_truck(T0_2, @controller_lqr, scen1);
% Big constraint violation from k=2 to k=53
%% Exercise 8 : Computation of X_LQR set
[A_x, b_x] = compute_X_LQR;
%% Exercise 9: execute simulation with MPC_1
figure(4)
sgtitle('Simulation with MPC1 control, T01');
[T_MPC_11, p_MPC_11] = simulate_truck(T0_1, @controller_mpc_1, scen1);
figure(5)
sgtitle('Simulation with MPC1 control, T02');
[T_MPC_12, p_MPC_12] = simulate_truck(T0_2, @controller_mpc_1, scen1);
%% Exercise 12: execute simulation with MPC_2
figure(6)
sgtitle('Simulation with MPC2 control, T01');
[T_MPC_21, p_MPC_21] = simulate_truck(T0_1, @controller_mpc_2, scen1);
figure(16)
sgtitle('Simulation with MPC2 control, T02');
[T_MPC_22, p_MPC_22] = simulate_truck(T0_2, @controller_mpc_2, scen1);
%% Exercise 15: execute simulation with MPC_3
figure(7)
sgtitle('Simulation with MPC3 control, T01');
[T_MPC_31, p_MPC_31] = simulate_truck(T0_1, @controller_mpc_3, scen1);
figure(8)
sgtitle('Simulation with MPC3 control, T02');
[T_MPC_32, p_MPC_32] = simulate_truck(T0_2, @controller_mpc_3, scen1);
%% Exercise 17: execute simulation with MPC_3, T0_3
T0_3 = [12;12;12];
figure(9)
sgtitle('Simulation with MPC3 control, T03');
[T_MPC_33, p_MPC_33] = simulate_truck(T0_3, @controller_mpc_3, scen1);
%% Exercise 18: execute simulation with MPC_4, T0_3
figure(10)
sgtitle('Simulation with MPC4 control, T03');
[T_MPC_43, p_MPC_43] = simulate_truck(T0_3, @controller_mpc_4, scen1);
%% Exercise 19: execute simulation with MPC_4, T0_2
figure(11)
sgtitle('Simulation with MPC4 control, T02');
[T_MPC_42, p_MPC_42] = simulate_truck(T0_2, @controller_mpc_4, scen1);
%% Exercise 20: augmented state dynamics
% see compute_controller_base_parameters
%% Exercise 21: observer gain matrix 
eval = eig(param.A_aug+param.L*param.C_aug);
%% Exercise 22: offset-free MPC
figure(12)
sgtitle('Simulation with MPC5 control, T01');
[T_MPC_51, p_MPC_51] = simulate_truck(T0_1, @controller_mpc_5, scen2);
figure(13)
sgtitle('Simulation with MPC3 control, T01');
[T_MPC_31b, p_MPC_31b] = simulate_truck(T0_1, @controller_mpc_3, scen2);
%% Exercise 23: execute simulation with MPC_forces vs MPC_1
figure(20)
[T_MPC_12, p_MPC_12, t_sim] = simulate_truck(T0_2, @controller_mpc_1, scen1);
figure(21)
[T_MPC_f2, p_MPC_f2] = simulate_truck(T0_2, @controller_mpc_1_forces, scen1); % to initialise
[T_MPC_f2, p_MPC_f2, t_sim_forces] = simulate_truck(T0_2, @controller_mpc_1_forces, scen1);