function [relative_state_chaser, absolute_state_chaser, P, alpha, beta, ...
    gamma, station_keeping_points, desired_relative_state, mode] = ... 
    navigation_vbn(relative_state_chaser, absolute_state_chaser, ...
    absolute_state_target, control_force, P, alpha, beta, gamma, station_keeping_points)
    %% Define Initial State of Chaser S/C

    % Distance of chaser from target s/c after entering VBN mode
    range_initial = sqrt( ...
                            relative_state_chaser(1,:).^2 + ...
                            relative_state_chaser(2,:).^2 + ...
                            relative_state_chaser(3,:).^2 ...
                            );

    %% Define the camera parameters
    focal_length = 0.2; % m, Focal length of the Camera

    D = 0.05; % m, Distance between the LEDs

    d_initial = D*focal_length/range_initial; % Initial Size of the image on the Camera Sensor

    Az = atand(d_initial/focal_length); % Initial Azimuth Angle
    El = atand(d_initial/focal_length); % Initial Elevation Angle

    %% Determining the positions of LEDs on the spacecrafts
    % Uncomment the following lines in the section to determine the
    % position of the LEDs on the spacecrafts
    % xnc = led_positions_chaser(d_initial, Az, El, alpha, beta, gamma, relative_state_chaser);
    % xnt = led_positions_target(d_initial);

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

    range = range_initial;

    %% Implementation of the Kalman Filter

    relative_pos_chaser = [(cosd(El).*range).*cosd(Az); (cosd(El).*range).*sind(Az); range.*sind(El)];
    
    relative_state_chaser_measured = [relative_pos_chaser; ...
    relative_state_chaser(4,:); relative_state_chaser(5,:); relative_state_chaser(6,:)];
    
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

    % Check for conditions using a loop
    for i = 4:12-1
        condition = relative_state_chaser(2, 1) >= station_keeping_points(i, 2) & relative_state_chaser(2, 1) <= station_keeping_points(i+1, 2);
        if condition
            desired_relative_state = [station_keeping_points(i+1, 1:3), 0, 0, 0];
            mode = "Final";
            break; % exit the loop if a condition is met

        else
            desired_relative_state = [0, 0, 0, 0, 0, 0];
            mode = "Docked";
        end
    end
    
end
