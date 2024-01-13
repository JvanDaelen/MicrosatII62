save_state = cell(1);
[~, ~, save_state] = DEMO_MSAT_controller(save_state);



function [force_vector, torque_vector, save_state] = DEMO_MSAT_controller(save_state)
    absolute_position = [0 0 0 0 0 0];
    absolute_attitude = [0 0 0 0 0 0];
    relative_position = [0 0 0 0 0 0];
    relative_attitude = [0 0 0 0 0 0];
    desired_position = [1 1 1 0 0 0];
    desired_attitude = [0 0 0 0 0 0];
    controller = 'LQR';
    dt = 0.005;
    m = 1;

    for i = 0:5
        [force_vector, torque_vector, save_state] = MSAT_controller( ...
                absolute_position, ...
                absolute_attitude, ...
                relative_position, ...
                relative_attitude, ...
                desired_position, ...
                desired_attitude, ...
                controller, ...
                save_state, ...
                dt, ...
                m ...
                );
        disp(absolute_position)
        disp(force_vector)
        disp(dt * force_vector / m)
        absolute_position(1:3) = absolute_position + absolute_position(4:6);
        acc = dt*force_vector/m;
        disp([0 0 0 acc])
        absolute_position(4:6) = absolute_position + [0 0 0 acc];
    end

    disp(force_vector)
    disp(torque_vector)
end


function [force_vector, torque_vector, save_state] = MSAT_controller( ...
    absolute_position, ...
    absolute_attitude, ...
    relative_position, ...
    relative_attitude, ...
    desired_position, ...
    desired_attitude, ...
    controller, ...
    save_state, ...
    dt, ...
    m ...
    )
    
    absolute_position_error = desired_position - absolute_position;

    if strcmp('LQR', controller)
        [force_vector, torque_vector] = LQR_3D(absolute_position_error, dt, m);
    elseif strcmp('PID', controller)
        force_vector = absolute_error;
        torque_vector = [0 0 0];
    end
end