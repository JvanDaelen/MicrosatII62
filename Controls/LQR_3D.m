function [F, T] = LQR_3D(relative_state_chaser, desired_relative_state, time_step, mass, n)
    state_error_vector = relative_state_chaser - desired_relative_state;
    
    % System Dynamics
    A = [0     0   0    1    0   0; 
        0     0   0    0    1   0;
        0     0   0    0    0   1;
        3*n^2 0   0    0    2*n 0;
        0     0   0    -2*n 0   0;
        0     0   -n^2 0    0   0];
    B = [0    0   0; 
        0    0   0;
        0    0   0;
        1/mass  0   0;
        0    1/mass 0;
        0    0   1/mass];
    C = eye(6);
    D = [0 0 0;
        0 0 0;
        0 0 0;
        0 0 0;
        0 0 0;
        0 0 0];
    
    % Control Law
    Q = [.1 0 0 0 0 0;  % Penalize x error
         0 .1 0 0 0 0;  % Penalize y error
         0 0 .1 0 0 0;  % Penalize z error
         0 0 0 1 0 0;  % Penalize x velocity
         0 0 0 0 1 0;  % Penalize y velocity
         0 0 0 0 0 1]; % Penalize z velocity
    R = [2 0 0;      % Penalize Fx
         0 2 0;      % Penalize Fy
         0 0 2];     % Penalize Fz
    K = lqr(A,B,Q,R);
    % 
    % Closed loop system
    sys_ss = ss((A - B*K), B, C, D);
    
    % Run response to initial condition
    [y,~,~] = initial(sys_ss, state_error_vector, [0 time_step]);
    
    
    % Total impulse
    acceleration_array = diff(y(:,4:6));
    F = mass*acceleration_array;
    T = [0 0 0];
end