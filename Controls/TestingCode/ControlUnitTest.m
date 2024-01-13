clear all

relative_state_chaser = [3; 0; 0; 0; 0; 0];
desired_relative_state = [1; 0; 0; 0; 0; 0];

time_step = 0.3;

mass = 1;
mean_motion = 1e-5;

controller = "PID";

control_memory_variable = cell(0);
close_system("SS_PID_Simulink.slx", 0)

for i = 1:100
    [control_force, control_memory_variable] = Control( ...
    control_memory_variable, ...
    relative_state_chaser, ...
    desired_relative_state, ...
    controller, ...
    time_step, ...
    mass, ...
    mean_motion ...
    );

    F = control_force;

    old_vel = relative_state_chaser(4:6);
    new_vel = relative_state_chaser(4:6) + F/mass * time_step;
    
    position_change = (old_vel + new_vel)/2 * time_step;
    
    relative_state_chaser(1:3) = relative_state_chaser(1:3) + position_change;
    relative_state_chaser(4:6) = new_vel;
    
    disp(relative_state_chaser)
end