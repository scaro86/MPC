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