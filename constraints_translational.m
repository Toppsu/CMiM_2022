function C = constraints_translational(sys, q)
%CONSTRAINTS_TRANSLATIONAL Compute constraints for the translational joints
C = zeros(2 * length(sys.joints.translational), 1);
c_id = 0;
x_free = 0;
y_free = 0;

for j = sys.joints.translational
    qi = q(j.body_i_qidx);
    qj = q(j.body_j_qidx);
    Ai = rot(qi(3));
    Aj = rot(qj(3));

        if j.free_trans == 1
        x_free = 1;
        end
    
        if j.free_trans == 2
        y_free = 1;
        end
    
    x_i_P = qi(1) + Ai(1) * j.s_i(1) + Ai(3) * j.s_i(2);
    y_i_P = qi(2) + Ai(2) * j.s_i(1) + Ai(4) * j.s_i(2);

    x_j_P = qj(1) + Aj(1) * j.s_j(1) + Aj(3) * j.s_j(2);
    y_j_P = qj(2) + Aj(2) * j.s_j(1) + Aj(4) * j.s_j(2);

    x_i_Q = qi(1) + Ai(1) * j.s_i(1) + Ai(3) * j.s_i(2) + x_free;
    y_i_Q = qi(2) + Ai(2) * j.s_i(1) + Ai(4) * j.s_i(2) + y_free;

    d = zeros(2,1);

    %d(1:2) = qj(1:2) + Aj * j.s_j ...
    %    - qi(1:2) - Ai * j.s_i;

%     d(1) = qj(1) + Aj(1) * j.s_j(1) + Aj(3) * j.s_j(2) - qi(1) - Aj(1) * j.s_i(1) + Aj(3) * j.s_i(2);
%     d(2) = qj(2) + Aj(2) * j.s_j(1) + Aj(4) * j.s_j(2) - qi(2) - Ai(2) * j.s_i(1) + Aj(4) * j.s_i(2);
    
    d(1) = x_j_P - x_i_P;
    d(2) = y_j_P - y_i_P;

    fii_i_0 = qi(3);
    fii_j_0 = qj(3);

    n_i = [-(y_i_P - y_i_Q)
        x_i_P - x_i_Q];
    
    %d
    %n_i
    %n_i' * d
    %n_i' .* d

    C(c_id + 1) = n_i' * d;
    C(c_id + 2) = qi(3) - qj(3) - (fii_i_0 - fii_j_0);
    c_id = c_id + 2;
end

end