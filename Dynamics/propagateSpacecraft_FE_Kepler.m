function [newstate, acceleration] = propagateSpacecraft_FE_Kepler(state, dt)
    % state/newstate is 6x1 array [x; y; z; u; v; w]
    % dt is timestep in seconds

    acceleration = calculateAccelerationKepler(state);
    statedot = [state(4); state(5); state(6); ...
        acceleration(1); acceleration(2); acceleration(3)];
    newstate = state + statedot * dt;
end