% BRIEF:
%   Controller function template. This function can be freely modified but
%   input and output dimension MUST NOT be changed.
% INPUT:
%   T: Measured system temperatures, dimension (3,1)
% OUTPUT:
%   p: Cooling power, dimension (2,1)
function p = controller_mpc_1(T)
persistent param yalmip_optimizer
% get parameters
if isempty(param)
    param = compute_controller_base_parameters;
end
x0 = T - param.T_sp;
Ucons = param.Ucons;
Xcons = param.Xcons;

%% evaluate control actuation
N = 30;
nx = size(param.A,1);
nu = size(param.B,2);
% define symbolic decision values
U = sdpvar(repmat(nu,1,N),repmat(1,1,N),'full');
X = sdpvar(repmat(nx,1,N+1),repmat(1,1,N+1),'full');
%define constraints and objective function
objective = 0;
constraints = [X{1}==x0];
for k = 1:N
  constraints = [constraints, X{k+1}==param.A*X{k}+param.B*U{k}];
  constraints = [constraints, Xcons(:,1)<=X{k+1}<=Xcons(:,2)];
  constraints = [constraints, Ucons(:,1)<=U{k}<=Ucons(:,2)];
  objective = objective + X{k}'*param.Q*X{k}+U{k}'*param.R*U{k};
end
lf = compute_infinite_cost_LQR(X{N+1});
objective = objective + lf;

ops = sdpsettings('verbose',0);
yalmip_opt = optimize(constraints,objective,ops);
% Analyze error flags
if yalmip_opt.problem == 0
 % Extract and display value
 u = value(U{1});
 p = u + param.p_sp;
else
 display('Hmm, something went wrong!');
 yalmip_opt.info
 yalmiperror(yalmip_opt.problem)
end
end