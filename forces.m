function f = forces(sys)
%FORCES Get force vector from the system
f = zeros(3 * length(sys.bodies), 1);
for ii = 1:length(sys.bodies)
    f(body_idx(ii)) = [sys.bodies(ii).mass * sys.gravity; 
        0];
end

