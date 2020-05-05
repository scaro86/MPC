% BRIEF:
%   Controller function template. This function can be freely modified but
%   input and output dimension MUST NOT be changed.
% INPUT:
%   T: Measured system temperatures, dimension (3,1)
% OUTPUT:
%   p: Cooling power, dimension (2,1)
function p = controller_mpc_4(T)
persistent param yalmip_optimizer
% get parameters and do optimization based on sdpvar x0
if isempty(param)
    [yalmip_optimizer, param] = mpc_4_optimizer();
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

function [yalmip_opt, param] = mpc_4_optimizer(param)
% get parameters
param = compute_controller_base_parameters;
Ucons = param.Ucons;
Xcons = param.Xcons;
A = param.A;
B = param.B;
Q = param.Q;
R = param.R;
[A_x, b_x] = compute_X_LQR;
% define penalty parameters
S = 1e-3*eye(6);
Sn = 1e-3*eye(length(b_x));
v = 1e-5*ones(1,6);
vn = 1e-5*ones(1,length(b_x));
%% evaluate control actuation
N = 30;
nx = size(A,1);
nu = size(B,2);
% define symbolic decision values
U = sdpvar(repmat(nu,1,N),repmat(1,1,N),'full');
X = sdpvar(repmat(nx,1,N+1),repmat(1,1,N+1),'full');
eps = sdpvar(repmat(2*nx,1,N),repmat(1,1,N),'full');
epsN = sdpvar(length(b_x),1);
x0 = sdpvar(3,1);
%define constraints and objective function
objective = 0;
constraints = [X{1}==x0, A_x*X{31}<=b_x+epsN];
for k = 1:N
  constraints = [constraints, X{k+1}==A*X{k}+B*U{k}];
  constraints = [constraints, Xcons(:,1)-eps{k}(1:3)<=X{k+1}<=Xcons(:,2)+eps{k}(4:6)];
  constraints = [constraints, Ucons(:,1)<=U{k}<=Ucons(:,2)];
  constraints = [constraints, zeros(6,1)<=eps{k}];
  objective = objective + X{k}'*Q*X{k}+U{k}'*R*U{k}+v*eps{k}+eps{k}'*S*eps{k};
end
% determine terminal cost P (solution to discrete-time Riccati equation)
P = dare(A,B,Q,R);
lf = X{31}'*P*X{31};
objective = objective + lf + vn*epsN+epsN'*Sn*epsN; 

ops = sdpsettings('verbose',0,'solver','quadprog');
yalmip_opt = optimizer(constraints,objective,ops,x0,U{1});
end