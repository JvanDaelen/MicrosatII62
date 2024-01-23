function [relative_state_chaser, absolute_state_chaser] = navigation(relative_state_chaser, absolute_state_chaser, absolute_state_target, control_force)
    % System dynamics matrices
    % A = eye(6); %+ randn(6) * 0.1;  % State transition matrix (with some randomness)
    A = [1 1 1 1 1 1; 0 0 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0];
    B = [zeros(3); zeros(3)];      % Control input matrix (assuming control_force is [F_x, F_y, F_z])
    H = eye(6);                   % Measurement matrix

    % Process and measurement noise covariances
    Q = diag([0.01, 0.01, 0.01, 0.001, 0.001, 0.001]);  % Process noise covariance
    R = diag([0.1, 0.1, 0.1, 0.01, 0.01, 0.01]);        % Measurement noise covariance

    % Prediction Step
    x_hat_minus = A * absolute_state_chaser;          % Predicted absolute state
    P_minus = A * Q * A' + Q;                          % Predicted error covariance

    % Update Step (using the relative state as a measurement)
    K = P_minus * H' / (H * P_minus * H' + R);         % Kalman gain
    x_hat = x_hat_minus + K * (relative_state_chaser - H * x_hat_minus);  % Updated absolute state estimate
    P = (eye(6) - K * H) * P_minus;                     % Updated error covariance

    % Apply control force to the chaser
    x_hat = x_hat + B * control_force;

    % Compute the updated relative state
    relative_state_chaser = x_hat - absolute_state_target;

    % Output the updated relative state and absolute state of the chaser
    relative_state_chaser = relative_state_chaser(:);   % Ensure column vector
    disp('relative_state_chaser=: ')
    disp(relative_state_chaser);
    absolute_state_chaser = x_hat(:);
    disp('absolute_state_chaser=: ')
    disp(absolute_state_chaser);
end
