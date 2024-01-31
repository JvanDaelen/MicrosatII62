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
    ymax = 0.1; % m, Maximum height of the Camera Sensor
    zmax = 0.1; % m, Maximum width of the Camera Sensor

    AzMax = atand(ymax/focal_length); % deg, Maximum Azimuth angle for the Camera Sensor in the XY plane
    ElMax = atand(zmax/focal_length); % deg, Maximum Elevation angle for the Camera Sensor in the ZX plane

    D = 0.05; % m, Distance between the LEDs

    d_initial = D*focal_length/range_initial; % Initial Size of the image on the Camera Sensor

    Az = atand(d_initial/focal_length); % Initial Azimuth Angle
    El = atand(d_initial/focal_length); % Initial Elevation Angle

    center_camera_sensor = [0; ymax/2; zmax/2]; % Position of LED-5 on the Camera Sensor for the 2 spacecrafts to be aligned

    %% Define the ideal scaled down LED pattern layout on the Target S/C    
    led1_pos_target = [0; -d_initial; 0]; % LED-1
    led2_pos_target = [0; 0; -d_initial]; % LED-2
    led3_pos_target = [0; d_initial; 0]; % LED-3
    led4_pos_target = [0; 0; d_initial]; % LED-4
    led5_pos_target = [d_initial; 0; 0]; % LED-5
    
    % LED Pattern Matrix on the Target Frame
    xnt_ideal = [led1_pos_target, led2_pos_target, led3_pos_target, led4_pos_target, led5_pos_target];

    %% LED pattern on the Chaser S/C

    % Position of the LED on the camera sensor
    xy1 = d_initial*((cosd(Az+gamma)*cosd(alpha))-(sind(El+beta)*sind(Az+gamma)*sind(alpha)));
    
    xz1 = -d_initial*cosd(El+beta)*sind(alpha);
    
    xy2 = d_initial*((cosd(Az+gamma)*sind(alpha))+(sind(El+beta)*sind(Az+gamma)*cosd(alpha)));
    
    xz2 = d_initial*cosd(El+beta)*cosd(alpha);
    
    xy3 = -d_initial*((cosd(Az+gamma)*cosd(alpha))-(sind(El+beta)*sind(Az+gamma)*sind(alpha)));
    
    xz3 = d_initial*cosd(El+beta)*sind(alpha);
    
    xy4 = -d_initial*((cosd(Az+gamma)*sind(alpha))+(sind(El+beta)*sind(Az+gamma)*cosd(alpha)));
    
    xz4 = -d_initial*cosd(El+beta)*cosd(alpha);
    
    xy5 = d_initial*(cosd(El+beta)*sind(Az+gamma));
    
    xz5 = -d_initial*sind(El+beta);

    led1_pos_chaser = [xy1; relative_state_chaser(2,:); xz1];

    led2_pos_chaser = [xy2; relative_state_chaser(2,:); xz2];

    led3_pos_chaser = [xy3; relative_state_chaser(2,:); xz3];

    led4_pos_chaser = [xy4; relative_state_chaser(2,:); xz4];

    led5_pos_chaser = [xy5; relative_state_chaser(2,:); xz5];

    % LED Pattern Matrix on the Chaser Frame 
    xnc = [led1_pos_chaser, led2_pos_chaser, led3_pos_chaser, led4_pos_chaser, led5_pos_chaser];

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
