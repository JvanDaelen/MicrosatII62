function [control_force, control_memory_variable] = Control( ...
    control_memory_variable, ...
    relative_state_chaser, ...
    desired_relative_state, ...
    controller, ...
    time_step, ...
    mass, ...
    mean_motion ...
    )
    % MSAT Controll function
    % controller can be LQR, PID
    if strcmp('LQR', controller)
        [control_force, ~] = LQR_3D(relative_state_chaser, desired_relative_state, time_step, mass, mean_motion);
    elseif strcmp('PID', controller)
        if isempty(control_memory_variable)
            % Initialize the integral initial values for PID
            control_memory_variable(1) = {[0 0 0]};
        end
        [control_force, ~] = SSPID(relative_state_chaser, desired_relative_state, mean_motion, mass, time_step, control_memory_variable);
    end

    % Convert control_force from row to column vector
    control_force = control_force';
end