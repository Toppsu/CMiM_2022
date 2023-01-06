%% Check adding ground body
sys = make_system();
sys = add_body(sys, "ground");
assert(length(sys.bodies) == 1)
b = sys.bodies(1);
assert(b.name == "ground")
assert(norm(b.location) == 0)
assert(length(b.location) == 2)
assert(b.orientation == 0)