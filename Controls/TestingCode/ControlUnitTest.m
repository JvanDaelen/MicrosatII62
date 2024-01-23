clear all

relative_state_chaser = [0; 0; 0; 0; 0; 0];
desired_relative_state = [0; 0; 0; 1; 1; 1];

time_step = 0.3;

mass = 1;
mean_motion = 1e-5;

controller = "PID";

control_memory_variable = cell(0);
close_system("SS_PID_Simulink.slx", 0)
close_system("VEL_SS_PID_Simulink.slx", 0)

control_memory_variable = cell(0); % Variable to store controller initialization
control_force_history = [0;0;0];
time_history = [0];
relative_state_chaser_history = relative_state_chaser;

t = 0;
for i = 1:20
    [control_force, control_memory_variable] = Control( ...
    control_memory_variable, ...
    relative_state_chaser, ...
    desired_relative_state, ...
    controller, ...
    time_step, ...
    mass, ...
    mean_motion, ...
    "vel"...
    );

    F = control_force;

    old_vel = relative_state_chaser(4:6);
    new_vel = relative_state_chaser(4:6) + F/mass * time_step;
    
    position_change = (old_vel + new_vel)/2 * time_step;
    
    relative_state_chaser(1:3) = relative_state_chaser(1:3) + position_change;
    relative_state_chaser(4:6) = new_vel;
    
    disp(relative_state_chaser)

    % Save variables for plotting
    control_force_history = horzcat(control_force_history, control_force);
    relative_state_chaser_history = horzcat(relative_state_chaser_history, relative_state_chaser);
    time_history = horzcat(time_history, t);
    t = t + time_step;
end

PlotResults(relative_state_chaser_history, control_force_history, "ControlUnitTest", time_history)

% close_system("SS_PID_Simulink.slx", 0)
% close_system("VEL_SS_PID_Simulink.slx", 0)