% BRIEF:
%   Controller function template. This function can be freely modified but
%   input and output dimension MUST NOT be changed.
% INPUT:
%   T: Measured system temperatures, dimension (3,1)
% OUTPUT:
%   p: Cooling power, dimension (2,1)
function p = controller_mpc_5(T)
% controller variables
persistent param yalmip_optimizer xh dh

% initialize controller, if not done already
if isempty(param)
    [yalmip_optimizer, param] = mpc_5_optimizer();
    x(:,1) = x0;
    xh(:,1) = x0;
    dh(:,1) = [0;0;0];
end
%get the estimation 
Nsim = 10;
A = param.A;
B = param.B;
Bd = param.Bd;
C = param.C;
A_aug = param.A_aug;
B_aug = param.B_aug;
C_aug = param.C_aug;
d = param.d;
L = param.L;


%Simulate autonomous system
aux = [xh(:,1); dh(:,1)];
for i = 1:Nsim-1
    x(:,i+1) = A*x(:,i) + d;
    aux = A_aug*aux + L*(C_aug*aux - x(:,i));
    xh(:,i+1) = aux(1:2); 
    dh(:,i+1) = aux(3:4);
end

%Computation of steady states 
H=eye(3);
r=zeros(3,1);
left = [A-eye(3) B;...
    H*C zeros(3,2];
right = [-Bb*dh;...
        r];
estim = left\right;
xs = estim[1:3,:];
us = estim[4:end,:];

x0 = T-T_sp;% à revoir parce qu'on veut une estimation là
% get optimal u
[u, errorcode] = yalmip_optimizer([x0,d_hat]);
p = u + p_sp;
% Analyze error flags
if (errorcode ~= 0)
      warning('MPC infeasible');
end
end

function [param, yalmip_optimizer] = mpc_5_optimizer()
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
nx = size(A,1);
nu = size(B,2);
nd = size(Bd,1);
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
yalmip_opt = optimizer(constraints,objective,ops,[x0,d0],U{1});
end