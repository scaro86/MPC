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
  Q = param.Q;
  R = param.R;
  A = param.A;
  B = param.B;
  x = param.x;
  [K,S,e] = dlqr(A,B,Q,R);
  p = K*x;
  param.x = A*x + B*p;
end

function param = init()
param = compute_controller_base_parameters;
% add additional parameters if necessary, e.g.
  %param.F = ...,
  if isfield(param,'x') == false
      param.x = [3; 1; 0];
  end
end