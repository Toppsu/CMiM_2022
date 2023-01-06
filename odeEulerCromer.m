function [T, Q, Qd] = odeEulerCromer(accfun, q0, qd0, t_step, t_final)

n_steps = ceil(t_final / t_step) + 1;
T = linspace(0, t_final, n_steps);

Q = zeros(length(q0), length(T));
Qd = zeros(length(qd0), length(T));

% Initial condition
Q(:, 1) = q0;
Qd(:, 1) = qd0;
% Step equations forward in time
for n = 1:n_steps - 1
    qdd = accfun(Q(:, n), Qd(:, n), T(n));
    Qd(:, n+1) = Qd(:, n) + t_step*qdd;
    Q(:, n+1) = Q(:, n) + t_step*Qd(:, n+1);
end