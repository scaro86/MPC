function L = getL
param = compute_controller_base_parameters;

for i=1:50
    eigval(i,:) = round(rand(1,6),1);
    for j = 1:6
        if eigval(i,j)==1
            eigval(i,j) = 0;
        end
    end
    L{i} = -(place(param.A_aug', param.C_aug', eigval(i,:)))';
end
end
