function J = jacob0(S,q)
    noofJoints = size(q,2);
    for i = 1:noofJoints
        T{i} = twist2ht(S(:, i),q(i));
    end
    for i = 1:noofJoints
        Tsubs = eye(4);
        for j = 1:i
            Tsubs = Tsubs * T{j};
        end
        Jacob{i} = adjoint(S(:, i),Tsubs);
    end
    for i = 1:noofJoints
        J(:,i) = Jacob{i};
    end
    J;
end