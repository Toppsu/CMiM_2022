%% PREPROCESSOR
% Globally we have a complete multibody system
% It must contain bodies, joints, analysis settings
sys = make_system();

% Bodies

% What do we need to describe our body?
% location, orientation, name 

sys = add_body(sys, "ground");
sys = add_body(sys, "block", [-0.1, 0.05], -deg2rad(30), 0.1, 1e-3);

sys = add_joint_simple(sys, "ground", "x");
sys = add_joint_simple(sys, "ground", "y");
sys = add_joint_simple(sys, "ground", "fi");

sys = add_joint_simple(sys, "block", "x", -0.1);
sys = add_joint_simple(sys, "block", "fi", -deg2rad(30));

%% SOLVE system of ordinary differential equations

[T, Q, Qd] = solve_dynamics_EC(sys);

%% POSTPROCESSING
% checking with analytical solution
% plot(T, Q(5, :), T, 0.05 + 0.5 .* T .^ 2 * sys.gravity(2), '--', 'LineWidth', 2)
plot(T, Qd(6, :))