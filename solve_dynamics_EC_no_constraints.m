function [T, Q, Qd] = solve_dynamics_EC_no_constraints(sys)
%SOLVE_DYNAMICS_EC_NO_CONSTRAINTS Solve the multibody system sys on
%dynamics using Euler-Cromer integration scheme and assuming that no
%constraints are added to the system

q0 = initial_coordinates(sys);
qd0 = zeros(size(q0));

M = mass_matrix(sys);
f = forces(sys);

accfun = @(q, qd, t) M \ f;

[T, Q, Qd] = odeEulerCromer(accfun, q0, qd0, ...
    sys.solver.t_step, sys.solver.t_final);
