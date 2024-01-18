function vectorECI = CW2ECI(absolute_state_target, vector)
    % Doesn't take curvature of the path into account, so if the vector is
    % a relative state, it will become in accurate at large distances
    CWx_unit_vector = absolute_state_target(1:3) / norm(absolute_state_target(1:3));
    CWy_unit_vector = absolute_state_target(4:6) / norm(absolute_state_target(4:6));
    CWz_unit_vector = cross(CWx_unit_vector, CWy_unit_vector);
    vector_size = size(vector);
    if vector_size(1) == 3
        vectorECI = vector(1) * CWx_unit_vector + ...
                         vector(2) * CWy_unit_vector + ...
                         vector(3) * CWz_unit_vector;
    elseif vector_size(1) == 6
        position = vector(1) * CWx_unit_vector + ...
                   vector(2) * CWy_unit_vector + ...
                   vector(3) * CWz_unit_vector;
        velocity = vector(4) * CWx_unit_vector + ...
                   vector(5) * CWy_unit_vector + ...
                   vector(6) * CWz_unit_vector;
        vectorECI = [position; velocity];
    else
    	error("Invalid vector size in CW2ECI.")
    end
end

