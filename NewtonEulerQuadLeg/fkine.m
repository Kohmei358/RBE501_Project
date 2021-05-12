function T = fkine(S,M,q,frame)
    % Your code here
    noofJoints = size(S,2);
    if strcmp(frame,'space')
        Tfinal = eye(4);
        for i = 1:noofJoints
            T{i} = twist2ht(S(:, i),q(i));
            Tfinal = Tfinal * T{i};
        end
        Tfinal = Tfinal * M;
        T = Tfinal;
    end
    if strcmp(frame,'body')
        Tfinal = eye(4);
        for i = 1:noofJoints
            
            T{i} = twist2ht(S(:, i),q(i));
            Tfinal = Tfinal * T{i};
        end
        Tfinal = M * Tfinal;
        T = Tfinal;
    end
end