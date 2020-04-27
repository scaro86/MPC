% BRIEF:
%   Controller function template. Input and output dimension MUST NOT be
%   modified.
% INPUT:
%   T: Measured system temperatures, dimension (3,1)
% OUTPUT:
%   p: Cooling power, dimension (2,1)
function p = controller_lqr(T)
% controller variables
persistent param;

% initialize controller, if not done already
if isempty(param)
    param = init();
end

% compute control action
  [K,~,] = dlqr(param.A,param.B,param.Q,param.R);
  x = T - param.T_sp;
  u = K*x;
  p = u + param.p_sp;
end

function param = init()
param = compute_controller_base_parameters;
% add additional parameters if necessary, e.g.
  %param.F = ...,
end

