function [relative_state_chaser, absolute_state_chaser, P] = navigation(relative_state_chaser, absolute_state_chaser, absolute_state_target, control_force, P)
    %% Kalman Filter Initialisation

    dt = 1e-3;

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
        0 dt^3 0;
        0 0 0]; % Control Matrix

    Q = diag([0, 0, 0, 5e-5, 5e-5, 5e-5]); % Process Noise Covariance Matrix

    R = [6e-2 0 0 0 0 0;
        0 6e-2 0 0 0 0;
        0 0 6e-2 0 0 0;
        0 0 0 1 0 0;
        0 0 0 0 1 0;
        0 0 0 0 0 1]; % Measurement Covariance Matrix - Error in measurement

    H = eye(6); % Measurement Noise Matrix

    %% Implementation of the Kalman Filter
    
    relative_state_chaser_measured = LIDARSensor(relative_state_chaser);
    
    x_hat = relative_state_chaser;
    
    % Time update (propagation)
    x_hat_minus = (A * x_hat) + B * control_force; 
    P_minus = (A * P * A') + Q;
            
    % Measurement update (correction)
    y = relative_state_chaser_measured - (H * x_hat_minus);
    S = (H * P_minus * H') + R;
    K = P_minus * H'/S;
    x_hat = x_hat_minus + (K * y);
    P = (eye(size(P)) - (K * H)) * P_minus;
    
    relative_state_chaser = x_hat;
    
    absolute_state_chaser = relative_state_chaser + absolute_state_target;
end
