% BRIEF:
%   Controller function template. This function can be freely modified but
%   input and output dimension MUST NOT be changed.
% INPUT:
%   T: Measured system temperatures, dimension (3,1)
% OUTPUT:
%   p: Cooling power, dimension (2,1)
function p = controller_mpc_3(T)
persistent param yalmip_optimizer
% get parameters and do optimization based on sdpvar x0
if isempty(param)
    [yalmip_optimizer, param] = mpc_3_optimizer();
end
x0 = T - param.T_sp;
% get optimal u
[u, errorcode] = yalmip_optimizer(x0);
p = u + param.p_sp;
% Analyze error flags
if (errorcode ~= 0)
      warning('MPC infeasible');
end
end

function [yalmip_opt, param] = mpc_3_optimizer(param)
% get parameters
param = compute_controller_base_parameters;
Ucons = param.Ucons;
Xcons = param.Xcons;
A = param.A;
B = param.B;
Q = param.Q;
R = param.R;
[A_x, b_x] = compute_X_LQR;
%% evaluate control actuation
N = 30;
nx = size(A,1);
nu = size(B,2);
% define symbolic decision values
U = sdpvar(repmat(nu,1,N),repmat(1,1,N),'full');
X = sdpvar(repmat(nx,1,N+1),repmat(1,1,N+1),'full');
x0 = sdpvar(3,1);
%define constraints and objective function
objective = 0;
constraints = [X{1}==x0, A_x*X{31}<=b_x];
for k = 1:N
  constraints = [constraints, X{k+1}==A*X{k}+B*U{k}];
  constraints = [constraints, Xcons(:,1)<=X{k+1}<=Xcons(:,2)];
  constraints = [constraints, Ucons(:,1)<=U{k}<=Ucons(:,2)];
  objective = objective + X{k}'*Q*X{k}+U{k}'*R*U{k};
end
% determine terminal cost P (solution to discrete-time Riccati equation)
P = dare(A,B,Q,R);
lf = X{31}'*P*X{31};
objective = objective + lf;

ops = sdpsettings('verbose',0,'solver','quadprog');
yalmip_opt = optimizer(constraints,objective,ops,x0,U{1});
end