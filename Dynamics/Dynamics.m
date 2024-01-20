function [newChaserState, newTargetState, newEarthRotation] ...
    = Dynamics(integrator, accelerationModel, chaserState, targetState, ...
    controlForce, mass, dt, earthRotation)
    % states are absolute
    if strcmp(integrator, 'FE') && strcmp(accelerationModel, 'kepler')
        [newChaserState, ~] = propagateSpacecraft_FE_Kepler(chaserState, ...
            controlForce, mass, dt);
        [newTargetState, ~] = propagateSpacecraft_FE_Kepler(targetState, ...
            [0;0;0], mass, dt);
        newEarthRotation = earthRotation;
    
    elseif strcmp(integrator, 'FE') && strcmp(accelerationModel, 'SH')
        [newChaserState, ~] = propagateSpacecraft_FE_SH(chaserState, ...
            controlForce, mass, dt, earthRotation);
        [newTargetState, ~] = propagateSpacecraft_FE_SH(targetState, ...
            [0;0;0], mass, dt, earthRotation);
        newEarthRotation = propagateEarth(earthRotation);
    
    elseif strcmp(integrator, 'RK4') && strcmp(accelerationModel, 'kepler')
        [newChaserState, ~] = propagateSpacecraft_RK4_Kepler(chaserState, ...
            controlForce, mass, dt);
        [newTargetState, ~] = propagateSpacecraft_RK4_Kepler(targetState, ...
            [0;0;0], mass, dt);
        newEarthRotation = earthRotation;
    
    elseif strcmp(integrator, 'RK4') && strcmp(accelerationModel, 'SH')
        [newChaserState, ~] = propagateSpacecraft_RK4_SH(chaserState, ...
            controlForce, mass, dt, earthRotation);
        [newTargetState, ~] = propagateSpacecraft_RK4_SH(targetState, ...
            [0;0;0], mass, dt, earthRotation);
        newEarthRotation = propagateEarth(earthRotation);
    
    else
        error("Invalid selection of integrator or acceleration model.")
    end
end

