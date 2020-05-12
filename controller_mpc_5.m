% BRIEF:
%   Controller function template. This function can be freely modified but
%   input and output dimension MUST NOT be changed.
% INPUT:
%   T: Measured system temperatures, dimension (3,1)
% OUTPUT:
%   p: Cooling power, dimension (2,1)
function p = controller_mpc_5(T)
% controller variables
persistent param yalmip_optimizer d_hat

% initialize controller, if not done already
if isempty(param)
    [param, yalmip_optimizer] = mpc_5_optimizer();
    d_hat = param.d;
end
%get parameters 
Nsim = 10;
A = param.A;
Bd = param.Bd;
B = param.B;
C = param.C;
B_aug = param.B_aug;
A_aug = param.A_aug;
C_aug = param.C_aug;
d = param.d;
L = param.L;
%Simulate autonomous system to estimate d_hat
T_hat = T;
x = T;
aux = [T_hat; d_hat];
for i = 1:Nsim-1
    x(:,i+1) = A*x(:,i)+ Bd*d;
    aux = A_aug*aux+ L*C_aug*(aux - [x(:,i);d]);
end
d_hat = aux(4:6);
% calculate steady state
T_sp = param.T_sp; % for T1 and T2 we track the same steady state as before
[T_sp, p_sp] = steady(A, B, Bd, T_sp, d_hat); %calculate T_sp(3) and p_sp
% get x0
x0 = T - T_sp;
% get optimal u
[u, errorcode] = yalmip_optimizer([x0, d_hat]);
p = u + p_sp;
% Analyze error flags
if (errorcode ~= 0)
      warning('MPC infeasible');
end
end

function [param, yalmip_opt] = mpc_5_optimizer()
% initializes the controller on first call and returns parameters and
% Yalmip optimizer object
param = compute_controller_base_parameters; % get basic controller parameters
Ucons = param.Ucons;
Xcons = param.Xcons;
A = param.A;
B = param.B;
C = param.C;
Bd = param.Bd;
Q = param.Q;
R = param.R;
[A_x, b_x] = compute_X_LQR;
%% implement your MPC using Yalmip 
N=30;
nx = size(A,1);
nu = size(B,2);
nd = size(Bd,2);
% define symbolic decision values
U = sdpvar(repmat(nu,1,N),repmat(1,1,N),'full');
X = sdpvar(repmat(nx,1,N+1),repmat(1,1,N+1),'full');
D = sdpvar(repmat(nd,1,N+1),repmat(1,1,N+1),'full');
x0 = sdpvar(3,1);
d0 = sdpvar(3,1);
%define constraints and objective function
objective = 0;
constraints = [X{1}==x0, D{1} == d0, A_x*X{31}<=b_x];
for k = 1:N
  constraints = [constraints, X{k+1}==A*X{k}+B*U{k}+Bd*D{k}];
  constraints = [constraints, D{k+1} == D{k}];
  constraints = [constraints, Xcons(:,1)<=X{k+1}<=Xcons(:,2)];
  constraints = [constraints, Ucons(:,1)<=U{k}<=Ucons(:,2)];
  objective = objective + X{k}'*Q*X{k}+U{k}'*R*U{k};
end
% determine terminal cost P (solution to discrete-time Riccati equation)
P = dare(A,B,Q,R);
lf = X{31}'*P*X{31};
objective = objective + lf;

ops = sdpsettings('verbose',0,'solver','quadprog');
yalmip_opt = optimizer(constraints,objective,ops,[x0, d0],U{1});
end