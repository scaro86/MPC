% I use this script to test my code in functions. 
% Please delete script before handing in assignment.
% If you'd like to test something, use this script. Feel free to delete 
% anything that is in this script. Don't keep anything important in here, 
% so the next person can delete your code and test something else.

x0 = [3; 1; 0];
Ucons = param.Ucons;
Xcons = param.Xcons;
A = param.A;
B = param.B;
Q = param.Q;
R = param.R;

%% evaluate control actuation
N = 30;
nx = size(A,1);
nu = size(B,2);
% define symbolic decision values
U = sdpvar(repmat(nu,1,N),repmat(1,1,N),'full');
X = sdpvar(repmat(nx,1,N+1),repmat(1,1,N+1),'full');
x0_sdp = sdpvar(3,1);
%define constraints and objective function
objective = 0;
constraints = [X{1}==x0_sdp];
for k = 1:N
  constraints = [constraints, X{k+1}==A*X{k}+B*U{k}];
  constraints = [constraints, Xcons(:,1)<=X{k+1}<=Xcons(:,2)];
  constraints = [constraints, Ucons(:,1)<=U{k}<=Ucons(:,2)];
  objective = objective + X{k}'*Q*X{k}+U{k}'*R*U{k};
end
lf = compute_infinite_cost_LQR(X{N+1});
objective = objective + lf;

ops = sdpsettings('verbose',0);
yalmip_opt = optimizer(constraints,objective,ops,x0_sdp,U{1});
[u, errorcode] = yalmip_opt(x0);
% Analyze error flags
if (errorcode ~= 0)
      warning('MPC infeasible');
end
% % Analyze error flags
% if yalmip_opt.problem == 0
%  % Extract and display value
%  u = value(U{1});
%  p = u + param.p_sp;
% else
%  display('Hmm, something went wrong!');
%  yalmip_opt.info
%  yalmiperror(yalmip_opt.problem)
% end