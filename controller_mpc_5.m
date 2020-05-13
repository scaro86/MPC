% BRIEF:
%   Controller function template. This function can be freely modified but
%   input and output dimension MUST NOT be changed.
% INPUT:
%   T: Measured system temperatures, dimension (3,1)
% OUTPUT:
%   p: Cooling power, dimension (2,1)
function p = controller_mpc_5(T)
% controller variables
persistent param yalmip_optimizer d_hat T_hat T_sp p_sp
% initialize controller, if not done already
if isempty(param)
    first = 1;
    [param, yalmip_optimizer] = mpc_5_optimizer();
    d_hat = param.d;
    T_hat = T;
    T_sp = param.T_sp;
    p_sp = param.p_sp;
end
%get parameters 
B_aug = param.B_aug;
A_aug = param.A_aug;
C_aug = param.C_aug;
L = param.L;
%Estimate d_hat starting in the second iteration
est = [T_hat; d_hat(:,end)];
x0_est = T_hat-T_sp;
u_est = yalmip_optimizer([x0_est, d_hat(:,end)]);
p_est = u_est+p_sp;
if (sum(isnan(p_est))~=0)
    p_est = [-2500; -2000];
end
est = A_aug*est+B_aug*p_est+L*C_aug*(est-[T; zeros(3,1)]);
T_hat = est(1:3);
d_hat = [d_hat, est(4:6)];
d1 = d_hat(1,:); 
d2 = d_hat(2,:); 
d3 = d_hat(3,:);
subplot(3,1,1)
plot(d1);
subplot(3,1,2)
plot(d2);
subplot(3,1,3)
plot(d3);
err = abs(d_hat(:,end)-d_hat(:,end-1));
if (err <= 1)
    err = abs(d_hat-d_hat(:,end));
    err1 = max(err(1,:));
    err2 = max(err(2,:));
    err3 = max(err(3,:));
end
% calculate steady state
T_sp = param.T_sp; % for T1 and T2 we track the same steady state as before
[T_sp, p_sp] = steady(param.A, param.B, param.Bd, T_sp, d_hat(:,end)); %calculate T_sp(3) and p_sp
% get x0
x0 = T - T_sp;
% get optimal u
[u, errorcode] = yalmip_optimizer([x0, d_hat(:,end)]);
p = u + p_sp;
p_est =p;
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