function [relative_state_chaser, absolute_state_chaser] = navigation_no_gps(relative_state_chaser, absolute_state_chaser, absolute_state_target, control_force)
    %% Define Initial Conditions
    
    dt = 1e-5;
    
    A = [0 0 0 dt 0 0;
        0 1 0 0 dt 0;
        0 0 0 0 0 dt;
        0 0 0 0 0 0;
        0 0 0 0 1 0;
        0 0 0 0 0 0]; % State Transition Matrix

    B = [0 0 0;
     0 dt^2/2 0;
     0 0 0;
     0 0 0;
     0 0 0;
     0 0 0]; % Control Matrix

    P = diag([1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4]); % Process Covariance Matrix

    Q = diag([0, 0, 0, 5e-5, 5e-5, 5e-5]); % Process Noise Covariance Matrix

    R = [2 0 0 0 0 0;
        0 2 0 0 0 0;
        0 0 2 0 0 0;
        0 0 0 1 0 0;
        0 0 0 0 1 0;
        0 0 0 0 0 1]; % Measurement Covariance Matrix - Error in measurement

    H = eye(6)*1e-5; % Measurement Noise Matrix
    
    %% Implementing the Kalman Filter

    x_hat = relative_state_chaser;

    % Time update (propagation)
    x_hat_minus = (A * x_hat) + B * control_force; 
    P_minus = (A * P * A') + Q;

    % Measurement update (correction)
    y = relative_state_chaser - (H * x_hat_minus);
    S = (H * P_minus * H') + R;
    K = P_minus * H'/S;
    x_hat = x_hat_minus + (K * y);
    P = (eye(size(P)) - (K * H)) * P_minus;
    
    relative_state_chaser = x_hat;

    absolute_state_chaser = relative_state_chaser + absolute_state_target;

    disp("Relative State Nav")
    disp(relative_state_chaser);

end