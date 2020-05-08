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
    systemLQR = LTISystem('A', param.A+param.B*K);
    Xp = Polyhedron('A',[eye(3); -eye(3); K; -K], 'b', [param.Xcons(:,2);-param.Xcons(:,1); param.Ucons(:,2);-param.Ucons(:,1)]);
    systemLQR.x.with('setConstraint');
    systemLQR.x.setConstraint = Xp;
    X_LQR = systemLQR.invariantSet();
    A_x =X_LQR.A ;
    b_x =X_LQR.b ;
    %figure(3)
    %X_LQR.plot(), alpha(0.25)
end

