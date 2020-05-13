function [T_sp, p_sp] = steady(A, B, Bd, T_sp, d)
syms T3 p1 p2
eqn1 = (1-A(3,3))*T3-B(3,1)*p1-B(3,2)*p2 == ...
        (A(3,1)*T_sp(1)+A(3,2)*T_sp(2)+Bd(3,:)*d);
eqn2 = -B(1,1)*p1-A(1,3)*T3-B(1,2)*p2 == ...
        ((A(1,1)-1)*T_sp(1)+A(1,2)*T_sp(2)+Bd(1,:)*d);
eqn3 = -B(2,2)*p2-A(2,3)*T3-B(2,1)*p1 == ...
        (A(2,1)*T_sp(1)+(A(2,2)-1)*T_sp(2)+Bd(2,:)*d);
sol = vpasolve([eqn1, eqn2, eqn3], [T3, p1, p2]);
T_sp(3,1) = double(sol.T3);
p_sp(1,1) = double(sol.p1);
p_sp(2,1) = double(sol.p2);

%     eqn1 = A(3,1)*T_sp(1)+A(3,2)*T_sp(2)+Bd(3,:)*d;
%     eqn2 = (A(1,1)-1)*T_sp(1)+A(1,2)*T_sp(2)+Bd(1,:)*d;
%     eqn3 = A(2,1)*T_sp(1)+(A(2,2)-1)*T_sp(2)+Bd(2,:)*d;
%     dB = B(2,1)*B(1,2)/B(1,1)/B(2,2);
%     eqn4 = 1/(1-dB)*(dB/B(2,1)*eqn3-eqn2/B(1,1));
%     m1 = 1/(1-dB)*(dB/B(2,1)*A(2,3)-A(1,3)/B(1,1));
%     eqn5 = -1/B(2,2)*(eqn3+B(2,1)*eqn4);
%     m2 = -1/B(2,2)*(A(2,3)+B(2,1)*m1);
%     eqn6 = eqn1+B(3,1)*eqn4+B(3,2)*eqn5;
%     m3 = 1-A(3,3)-B(3,1)*m1-B(3,2)*m2;
%     
%     T_sp(3,1) = eqn6/m3;
%     p_sp(1,1) = eqn4+m1*T_sp(3);
%     p_sp(2,1) = eqn5+m2*T_sp(3);
end

