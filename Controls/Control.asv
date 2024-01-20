function [control_force, control_memory_variable] = Control( ...
    control_memory_variable, ...
    relative_state_chaser, ...
    desired_relative_state, ...
    controller, ...
    time_step, ...
    mass, ...
    mean_motion, ...
    control_mode ...
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
        if strcmp('pos', control_mode)
            disp("pos conrtrol")
            [control_force, ~] = SSPID(relative_state_chaser, desired_relative_state, mean_motion, mass, time_step, control_memory_variable);
        elseif strcmp('vel', control_mode)
            disp("velocity conrtrol")
            [control_force, ~] = velSSPID(relative_state_chaser, desired_relative_state, mean_motion, mass, time_step, control_memory_variable);
        else
            error("Invalid control mode provided")
        end
    end
    
    % Convert control_force from row to column vector
    control_force = control_force';
end