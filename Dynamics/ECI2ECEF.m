function coordinatesECEF = ECI2ECEF(coordinatesECI, earthRotation)
    % ECEF is rotated positively around the z-axis w.r.t. ECI, so the 
    % coordinates must be transformed negatively around the z-axis
    % (we're viewing the coordinates from the perspective of the ECEF frame)

    % rotation matrix for negative rotation around the z-axis
    rotationMatrix = [cos(earthRotation) sin(earthRotation) 0; ...
                      -sin(earthRotation) cos(earthRotation) 0; ...
                      0 0 1];
    coordinatesECEF = rotationMatrix * coordinatesECI;
end