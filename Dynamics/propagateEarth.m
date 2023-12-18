function newEarthRotation = propagateEarth(earthRotation, dt)
    newEarthRotation = rem(earthRotation + (2 * pi / (86400)) * dt, 2 * pi);
end

