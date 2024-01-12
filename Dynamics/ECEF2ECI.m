function coordinatesECI = ECEF2ECI(coordinatesECEF, earthRotation)
    % ECEF is rotated positively around the z-axis w.r.t. ECI, so the 
    % coordinates must be transformed positively around the z-axis
    % (we're viewing the position from the perspective of the ECI frame)

    % rotation matrix for positive rotation around the z-axis
    rotationMatrix = [cos(earthRotation) -sin(earthRotation) 0; ...
                      sin(earthRotation) cos(earthRotation) 0; ...
                      0 0 1];
    coordinatesECI = rotationMatrix * coordinatesECEF;
end
