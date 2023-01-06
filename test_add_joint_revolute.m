%% Check adding joint between two bodies
sys = make_system();
sys = add_body(sys, "ground");
sys = add_body(sys, "crank");

sys = add_joint_revolute(sys, "ground", "crank", [0; 0], [0.3; -11]);
assert(length(sys.joints.revolute) == 1)
j = sys.joints.revolute(1);
assert(all(j.body_i_qidx == [1, 2, 3]))
assert(all(j.body_j_qidx == [4, 5, 6]))
assert(norm(j.s_i) == 0)
assert(all(j.s_j == [0.3; -11]))