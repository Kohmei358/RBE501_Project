function J_a = jacoba(S,M,q)    
% Your code here
    noofJoints = size(S,2);
    for i = 1:noofJoints
        T{i} = twist2ht(S(:, i),q(i));
        Tsubs = eye(4);
        for j = 1:i
            Tsubs = Tsubs * T{j};
        end
        Jacob{i} = adjoint(S(:, i),Tsubs);
        J_s(:,i) = Jacob{i};
    end
    Tf = fkine(S,M,q,'space');
    R1 = Tf(1:3,1:3); 
    
    % Calculating the adjoint of T inverse
    Tf_i = inv(Tf);
    R = Tf_i(1:3,1:3);
    P = Tf_i(1:3,4);
    skewP = [0  -P(3) P(2); P(3) 0 -P(1); -P(2) P(1) 0];
    z = zeros(3);
    adjTf_i = [R z; skewP*R R];
    J_b = adjTf_i * J_s;
    
    % CaLculating the analytical jacobian
    J_vb = J_b(4:6,:);
    J_a = R1 * J_vb;
end