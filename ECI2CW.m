function vectorCW = ECI2CW(absolute_state_target, vector)
    % Doesn't take curvature of the path into account, so if the vector is
    % a relative state, it will become in accurate at large distances
    
    velocity = absolute_state_target(4:6);
    [azimuth, elevation, ~] = cart2sph(absolute_state_target(1), ...
                                       absolute_state_target(2), ...
                                       absolute_state_target(3));
    rotationMatrixZ = [[cos(-azimuth), -sin(-azimuth), 0]; ...
                       [sin(-azimuth), cos(-azimuth), 0]; ...
                       [0, 0, 1]];
    rotationMatrixY = [[cos(elevation), 0, sin(elevation)]; ...
                       [0, 1, 0]; ...
                       [-sin(elevation), 0, cos(elevation)]];
    
    intermediateVelocity = rotationMatrixY * rotationMatrixZ * velocity;
    crossYV = cross([0; 1; 0], intermediateVelocity);
    dotYV = dot([0;1;0], intermediateVelocity);

    if crossYV(1) > 0
        rollAngle = atan2(norm(crossYV), dotYV);
    elseif crossYV(1) < 0
        rollAngle = -atan2(norm(crossYV), dotYV);
    elseif dotYV == norm(intermediateVelocity)
        rollAngle = 0;
    else
        rollAngle = pi;
    end

    rotationMatrixX = [[1, 0, 0]; ...
                       [0, cos(-rollAngle), -sin(-rollAngle)]; ...
                       [0, sin(-rollAngle), cos(-rollAngle)]];
    
    vector_size = size(vector);
    if vector_size(1) == 3
        vectorCW = rotationMatrixX * rotationMatrixY * rotationMatrixZ *...
                   vector;
    elseif vector_size(1) == 6
        positionCW = rotationMatrixX * rotationMatrixY * rotationMatrixZ *...
                   vector(1:3);
        velocityCW = rotationMatrixX * rotationMatrixY * rotationMatrixZ *...
                   vector(4:6);
        vectorCW = [positionCW; velocityCW];
    else
    	error("Invalid vector size in ECI2CW.")
    end
end