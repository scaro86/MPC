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
%Estimate d_hat
est = [T_hat; d_hat(:,end)];
x0_est = T_hat-T_sp;
u_est = yalmip_optimizer([x0_est, d_hat(:,end)]); %estimate control input
p_est = u_est+p_sp;
while (sum(isnan(p_est))~=0) % if MPC not feasible, use LQR
    K = -dlqr(param.A,param.B,param.Q,param.R);
    u_est = K*(T_hat - T_sp);
    p_est = u_est + p_sp;
end
est = A_aug*est+B_aug*p_est+L*C_aug*(est-[T; param.d]);
T_hat = est(1:3);
d_hat = [d_hat, est(4:6)];
% plot disturbance estimation
subplot(3,1,1)
plot(d_hat(1,:));
subplot(3,1,2)
plot(d_hat(2,:));
subplot(3,1,3)
plot(d_hat(3,:));
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
Bd = param.Bd;
Q = param.Q;
R = param.R;
[A_x, b_x] = compute_X_LQR;
%% implement your MPC using Yalmip 
N=50;
nx = size(A,1);
nu = size(B,2);
% define symbolic decision values
U = sdpvar(repmat(nu,1,N),repmat(1,1,N),'full');
X = sdpvar(repmat(nx,1,N+1),repmat(1,1,N+1),'full');
x0 = sdpvar(3,1);
d0 = sdpvar(3,1);
%define constraints and objective function
objective = 0;
constraints = [X{1}==x0, A_x*X{N+1}<=b_x];
for k = 1:N
  constraints = [constraints, X{k+1}==A*X{k}+B*U{k}+Bd*d0];
  constraints = [constraints, Xcons(:,1)<=X{k+1}<=Xcons(:,2)];
  constraints = [constraints, Ucons(:,1)<=U{k}<=Ucons(:,2)];
  objective = objective + X{k}'*Q*X{k}+U{k}'*R*U{k};
end
% determine terminal cost P (solution to discrete-time Riccati equation)
P = dare(A,B,Q,R);
lf = X{N+1}'*P*X{N+1};
objective = objective + lf;

ops = sdpsettings('verbose',0,'solver','quadprog');
yalmip_opt = optimizer(constraints,objective,ops,[x0, d0],U{1});
end