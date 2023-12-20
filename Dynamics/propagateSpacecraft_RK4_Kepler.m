function [newstate, acceleration] = propagateSpacecraft_RK4_Kepler(state, controlForce, mass, dt)
    % state/newstate is 6x1 array [x; y; z; u; v; w] in meters
    % controlForce is 3x1 array [F_x, F_y, F_z] in Newtons
    % mass is mass of spacecraft in kilograms
    % dt is timestep in seconds

    %%%%%%%%%%%%%%%%%% RK4 %%%%%%%%%%%%%%%%%%%
    % y_n+1 = y_n + h/6 * (k1 + 2k2 + 2k3 + k4)
    % t_n+1 = t_n + h

    % k1 = f(t_n, y_n)
    % k2 = f(t_n + h/2, y_n + h*k1/2)
    % k3 = f(t_n + h/2, y_n + h*k2/2)
    % k4 = f(t_n + h, y_n + h*k3)
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    acceleration = calculateAccelerationKepler(state)  + controlForce / mass;
    statedot = [state(4); state(5); state(6); ...
        acceleration(1); acceleration(2); acceleration(3)];
    k1 = statedot;
    
    intermediateState = state + k1 * dt/2;
    acceleration = calculateAccelerationKepler(intermediateState) + controlForce / mass;
    statedot = [intermediateState(4); intermediateState(5); ...
        intermediateState(6); acceleration(1); acceleration(2); ...
        acceleration(3)];
    k2 = statedot;

    intermediateState = state + k2 * dt/2;
    acceleration = calculateAccelerationKepler(intermediateState) + controlForce / mass;
    statedot = [intermediateState(4); intermediateState(5); ...
        intermediateState(6); acceleration(1); acceleration(2); ...
        acceleration(3)];
    k3 = statedot;

    intermediateState = state + k3 * dt;
    acceleration = calculateAccelerationKepler(intermediateState) + controlForce / mass;
    statedot = [intermediateState(4); intermediateState(5); ...
        intermediateState(6); acceleration(1); acceleration(2); ...
        acceleration(3)];
    k4 = statedot;

    newstate = state + dt/6 * (k1 + 2 * k2 + 2 * k3 + k4);
    acceleration = calculateAccelerationKepler(state) + controlForce / mass;
end