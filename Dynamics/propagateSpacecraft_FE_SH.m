function [newstate, acceleration] = propagateSpacecraft_FE_SH(state, controlForce, mass, dt, earthRotation)
    % state/newstate is 6x1 array [x; y; z; u; v; w] in meters
    % controlForce is 3x1 array [F_x, F_y, F_z] in Newtons
    % mass is mass of spacecraft in kilograms
    % dt is timestep in seconds
    % earthRotation is angle between ECI and ECEF fram in radians

    acceleration = calculateAccelerationSH(state, earthRotation) + controlForce / mass;
    statedot = [state(4); state(5); state(6); ...
        acceleration(1); acceleration(2); acceleration(3)];
    newstate = state + statedot * dt;
end