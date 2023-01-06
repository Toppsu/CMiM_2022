%% Add two bodies to the system and verify they exists
sys = make_system();
sys = add_body(sys, "ground");
sys = add_body(sys, "slider");

try
   check_body_exists(sys, "ground")
catch exception
   assert(false, "Body ground not found")
end

try
   check_body_exists(sys, "slider")
catch exception
   assert(false, "Body slider not found")
end

try
   check_body_exists(sys, "slidar"),
   assert(false, "Body slider not found")
catch exception
   assert(exception.message == "Body slidar not found in the system.")
end

try
   check_body_exists(sys, "slid"),
   assert(false, "Body slider not found")
catch exception
   assert(exception.message == "Body slid not found in the system.")
end