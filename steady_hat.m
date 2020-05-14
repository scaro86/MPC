function [T_sp, p_sp] = steady_hat(A, B, Bd, C, T_sp, d_hat)
r = [1,1]';
steady = [A-eye(3), B;...
    C, zeros(size(B))]\[-Bd*d_hat;r];
T_sp(3) = steady(3);
p_sp = steady(4:5);
end