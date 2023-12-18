function acceleration = calculateAccelerationSH(state, earthRotation)
    % acceleration = 3x1 vector [a_x; a_y; a_z]
    positionECEF = ECI2ECEF(state(1:3), earthRotation);
    
    % using aerospace toolbox add-on for spherical harmonics
    % (included in TUDelft MATLAB license)
    [a_x, a_y, a_z] = gravitysphericalharmonic(transpose(positionECEF));
    accelerationECEF = [a_x; a_y; a_z];
    
    acceleration = ECEF2ECI(accelerationECEF, earthRotation);
end