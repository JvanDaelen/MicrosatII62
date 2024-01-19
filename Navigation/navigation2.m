function [relative_state_chaser, absolute_state_chaser] = navigation2(relative_state_chaser, absolute_state_chaser, absolute_state_target, control_force, mass, time_step)
    % System dynamics matrices
    A = zeros(6);  % State transition matrix (with some randomness)
    B = [zeros(3); eye(3)];      % Control input matrix (assuming control_force is [F_x, F_y, F_z])
    H = eye(6);                   % Measurement matrix

    % Process and measurement noise covariances
    Q = diag([0.01, 0.01, 0.01, 0.001, 0.001, 0.001]);  % Process noise covariance
    R = diag([0.1, 0.1, 0.1, 0.01, 0.01, 0.01]);        % Measurement noise covariance

    % Prediction Step
    A = expm(A * time_step);  % Discrete-time approximation for continuous-time dynamics
    x_hat_minus = A * absolute_state_chaser;  % Predicted absolute state
    P_minus = A * Q * A' + Q;                  % Predicted error covariance

    % Update Step (using the relative state as a measurement)
    K = P_minus * H' / (H * P_minus * H' + R);  % Kalman gain
    x_hat = x_hat_minus + K * (relative_state_chaser - H * x_hat_minus);  % Updated absolute state estimate
    P = (eye(6) - K * H) * P_minus;  % Updated error covariance

    % Apply control force to the chaser
    x_hat = x_hat + (B / mass) * control_force * time_step;  % Adjusted for mass and time step

    % Output the updated relative state and absolute state of the chaser
    absolute_state_chaser = x_hat(:);
    % disp('absolute_state_chaser=: ')
    % disp(absolute_state_chaser);
    % Compute the updated relative state
    relative_state_chaser = absolute_state_chaser - absolute_state_target;
    relative_state_chaser = relative_state_chaser(:);   % Ensure column vector
    % disp('relative_state_chaser=: ')
    % disp(relative_state_chaser);
end