clear all

relative_state_chaser = [3; 0; 0; 0; 0; 0];
desired_relative_state = [1; 0; 0; 0; 0; 0];

time_step = 0.3;

mass = 1;
n = 1e-5;

for i = 1:1000
    [F, T] = LQR_3D(relative_state_chaser, desired_relative_state, time_step, mass, n);

    old_vel = relative_state_chaser(4:6);
    new_vel = relative_state_chaser(4:6) + F/mass * time_step;
    
    position_change = (old_vel + new_vel)/2 * time_step;
    
    relative_state_chaser(1:3) = relative_state_chaser(1:3) + position_change;
    relative_state_chaser(4:6) = new_vel;
    
    disp(relative_state_chaser)
end