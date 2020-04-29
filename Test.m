% I use this script to test my code in functions. 
% Please delete script before handing in assignment.
% If you'd like to test something, use this script. Feel free to delete 
% anything that is in this script. Don't keep anything important in here, 
% so the next person can delete your code and test something else.


% Define variables
x = sdpvar(10,1);

% Define constraints 
Constraints = [sum(x) <= 10, x(1) == 0, 0.5 <= x(2) <= -1.5];
for i = 1 : 7
  Constraints = [Constraints, x(i) + x(i+1) <= x(i) + x(i)];
end

% Define an objective
Objective = x'*x+norm(x,1);

% Set some options for YALMIP and solver
options = sdpsettings('verbose',1,'solver','quadprog','quadprog.maxiter',100);

% Solve the problem
sol = optimize(Constraints,Objective,options);

% Analyze error flags
if sol.problem == 0
 % Extract and display value
 solution = value(x)
else
 display('Hmm, something went wrong!');
 sol.info
 yalmiperror(sol.problem)
end

%%
if isempty(param)
    param = compute_controller_base_parameters; % get basic controller parameters
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
fprintf('JMPC_dummy = %f',value(objective));
yalmip_opt = optimize(constraints,objective,ops);
% Analyze error flags
if yalmip_opt.problem == 0
 % Extract and display value
 u = value(U{1})
else
 display('Hmm, something went wrong!');
 yalmip_opt.info
 yalmiperror(yalmip_opt.problem)
end