function costlqr = compute_infinite_cost_LQR(x0)
persistent param

if isempty(param)
    param = compute_controller_base_parameters; % get basic controller parameters
end

K = -dlqr(param.A,param.B,param.Q,param.R);
costlqr = 0;
x = x0;
for i = 1:1000
    costlqr = costlqr + x'*(param.Q + K'*param.R*K)*x;
    x = (param.A+param.B*K)*x;
end
end