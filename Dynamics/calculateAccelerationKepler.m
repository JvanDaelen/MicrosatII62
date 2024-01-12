function acceleration = calculateAccelerationKepler(state)
    % acceleration = 3x1 vector [a_x; a_y; a_z]
    mu_earth = 398600.435507 * 10 ^ 9; % gravitational parameter [m^3/s^2]
    
    acceleration = - (mu_earth / norm(state(1:3))^3) * state(1:3);
end