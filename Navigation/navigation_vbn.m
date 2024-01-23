function [relative_state_chaser, absolute_state_chaser] = navigation_vbn(relative_state_chaser, absolute_state_chaser, absolute_state_target, control_force)
    %% Define Initial State of Chaser S/C

    % Distance of chaser from target s/c after entering VBN mode
    range_initial = sqrt( ...
                            relative_state_chaser(1,:).^2 + ...
                            relative_state_chaser(2,:).^2 + ...
                            relative_state_chaser(3,:).^2 ...
                            );

    % Attitude of the target spacecraft
    alpha = 0;
    beta = 0;
    gamma = 0;

    %% Define the camera parameters
    focal_length = 0.2; % m, Focal length of the Camera
    ymax = 0.1; % m, Maximum height of the Camera Sensor
    zmax = 0.1; % m, Maximum width of the Camera Sensor

    AzMax = atand(ymax/focal_length); % deg, Maximum Azimuth angle for the Camera Sensor in the XY plane
    ElMax = atand(zmax/focal_length); % deg, Maximum Elevation angle for the Camera Sensor in the ZX plane

    D = 0.05; % m, Distance between the LEDs

    d_initial = D*focal_length/range_initial; % Initial Size of the image on the Camera Sensor

    center_camera_sensor = [0; ymax/2; zmax/2]; % Position of LED-5 on the Camera Sensor for the 2 spacecrafts to be aligned

    %% Define LED pattern layout on the Target S/C    
    led1_pos = [0; d_initial; 0]; % LED-1
    led2_pos = [0; 0; d_initial]; % LED-2
    led3_pos = [0; -d_initial; 0]; % LED-3
    led4_pos = [0; 0; -d_initial]; % LED-4
    led5_pos = [-d_initial; 0; 0]; % LED-5

    %% Kalman Filter Initialisation

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
     0 dt^3 0;
     0 0 0]; % Control Matrix

    P = diag([1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4]); % Process Covariance Matrix

    Q = diag([0, 0, 0, 5e-5, 5e-5, 5e-5]); % Process Noise Covariance Matrix

    R = [6e-2 0 0 0 0 0;
        0 6e-2 0 0 0 0;
        0 0 6e-2 0 0 0;
        0 0 0 1 0 0;
        0 0 0 0 1 0;
        0 0 0 0 0 1]; % Measurement Covariance Matrix - Error in measurement

    H = eye(6); % Measurement Noise Matrix

    xk = d_initial;

    range = range_initial;

    %% Implementation of the Kalman Filter

    Az = atand(xk/focal_length); % Initial Azimuth Angle
    El = atand(xk/focal_length); % Initial Elevation Angle
    
    relative_pos_chaser = [(cos(El).*range).*cos(Az); (cos(El).*range).*sin(Az); range.*sin(El)];
    
    range = sqrt( ...
        relative_pos_chaser(1,:).^2 + ...
        relative_pos_chaser(2,:).^2 + ...
        relative_pos_chaser(3,:).^2 ...
        );
    
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

    disp("Relative State")
    disp(relative_state_chaser)
    
    absolute_state_chaser = relative_state_chaser + absolute_state_target;
    
    range = sqrt( ...
        relative_state_chaser(1,:).^2 + ...
        relative_state_chaser(2,:).^2 + ...
        relative_state_chaser(3,:).^2 ...
        );
    
    %disp("Range")
    %disp(range)
    
end
