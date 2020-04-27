function param = compute_controller_base_parameters
    % load truck parameters
    load('system/parameters_truck');
    % parameters
    m1 = truck.m1; m2 = truck.m2; m3 = truck.m3;
    a12 = truck.a12; a23 = truck.a23; 
    a1o = truck.a1o; a2o = truck.a2o; a3o = truck.a3o;
    To = truck.To; w=truck.w;
    
    % (1)
    Ac = [-(a12+a1o)/m1, a12/m1, 0; ...
        a12/m2, -(a12+a23+a2o)/m2, a23/m2; ...
        0, a23/m3, -(a23+a3o)/m3];
    Bc = [1/m1, 0; 0, 1/m2; 0, 0];
    Bdc = [1/m1, 0, 0; 0, 1/m2, 0; 0, 0, 1/m3];
    d = [w(1)+a1o*To; w(2)+a2o*To; w(3)+a3o*To];
    
    % (2) discretization
    Ts = 60;
    A = expm(Ac*Ts);
    B = inv(Ac)*(A-eye(3))*Bc;
    Bd = inv(Ac)*(A-eye(3))*Bdc;
    
    % (3) set point computation
    T_sp(1,1) = -21; T_sp(2,1) = 0.3;
    
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
    
    clear T3 p1 p2 eqn1 eqn2 eqn3
    
%     eqn1 = A(3,1)*T_sp(1)+A(3,2)*T_sp(2)+Bd(3,:)*d;
%     eqn2 = (A(1,1)-1)*T_sp(1)+A(1,2)*T_sp(2)+Bd(1,:)d;
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

    % (4) system constraints
    Pcons = truck.InputConstraints;
    Tcons = truck.StateConstraints;
    
    % (4) constraints for delta formulation
    Ucons = Pcons-[p_sp(1)*ones(1,2); p_sp(2)*ones(1,2)];
    Xcons = Tcons-[T_sp(1)*ones(1,2); T_sp(2)*ones(1,2); T_sp(3)*ones(1,2)];
    
    % (5) LQR cost function
    Q = 100*[1 1 1; 1 1 1; 1 1 1];
    R = 10*eye(2);
    
    % put everything together
    param.A = A;
    param.B = B;
    param.Q = Q;
    param.R = R;
    param.T_sp = T_sp;
    param.p_sp = p_sp;
    param.Ucons = Ucons;
    param.Xcons = Xcons;
    param.Tcons = Tcons;
    param.Pcons = Pcons;
end

