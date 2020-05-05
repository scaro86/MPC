% BRRIEF:
%   Template for explicit invariant set computation. You MUST NOT change
%   the output.
% OUTPUT:
%   A_x, b_x: Describes polytopic X_LQR = {x| A_x * x <= b_x}
function [A_x, b_x] = compute_X_LQR
    % get basic controller parameters
    param = compute_controller_base_parameters;
    %% Here you need to implement the X_LQR computation and assign the result.
    % computes a control invariant set for LTI system x^+ = A*x+B*u
    K = -dlqr(param.A,param.B,param.Q,param.R);
    %Xp = Polyhedron('A',[eye(3); -eye(3); K; -K], 'b', [param.Xcons(:,2);-param.Xcons(:,1); param.Ucons(:,2);-param.Ucons(:,1)]);
    A_x = [eye(3); -eye(3); K; -K];
    b_x = [param.Xcons(:,2);-param.Xcons(:,1); param.Ucons(:,2);-param.Ucons(:,1)];
end

