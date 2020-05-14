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
    [T_sp, p_sp] = steady(A, B, Bd, T_sp, d);
    
    % (4) system constraints
    Pcons = truck.InputConstraints;
    Tcons = truck.StateConstraints;
    
    % (4) constraints for delta formulation
    Ucons = Pcons-[p_sp(1)*ones(1,2); p_sp(2)*ones(1,2)];
    Xcons = Tcons-[T_sp(1)*ones(1,2); T_sp(2)*ones(1,2); T_sp(3)*ones(1,2)];
    
    % (5) LQR cost function
    Q = 1e-1*eye(3);
    R = 2*1e-6*eye(2);
    
    % (20) augmented system 
    A_aug =[A Bd;...
        zeros(3) eye(3)];
    B_aug = [B;...
        zeros(3,2)];
    C_aug = [eye(3) zeros(3)];
    %(21)
%     L = -(place(A_aug', C_aug', [0,0.35,0.1,0.2,0.4,0.5]))';
%     L = -(place(A_aug', C_aug', [0,0.3,0.1,0,0.4,0.5]))';
    L = -(place(A_aug', C_aug', [0.3,0.2,0.2,0.2,0.3,0.3]))';
    eig(A_aug + L*C_aug)
    
    % put everything together
    param.A = A;
    param.B = B;
    param.C = eye(3);
    param.Bd = Bd;
    param.A_aug = A_aug;
    param.B_aug = B_aug;
    param.C_aug = C_aug;
    param.L = L;
    param.Q = Q;
    param.R = R;
    param.d = d;
    param.T_sp = T_sp;
    param.p_sp = p_sp;
    param.Ucons = Ucons;
    param.Xcons = Xcons;
    param.Tcons = Tcons;
    param.Pcons = Pcons;
end

