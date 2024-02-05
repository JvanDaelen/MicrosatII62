% Master Simulation for Microsat
close all

%% Settings for dynamics simulation
run_ID = 'Final_Run_NoNav_PID';
integrator = 'FE'; % 'FE' for forward Euler or 'RK4' for 4th order fixed
                   % time_step Runge-Kutta
acceleration_model = 'kepler'; % 'kepler' for kepler orbit model or 'SH'
                               % for spherical harmonics model

%% Physical Constants
mass = 1; %kg
mu = 3.986004418e+14 ;%m3s-2

%% Inputs
orbital_height = 500e+3; %m

% Kepler elements
a = orbital_height + 6371e+3; % semi-major-axis in m
ecc = 0; % eccentricity
incl = 0; % deg inclination
RAAN = 0; % deg RAAN
argp = 0; % deg AOP
nu = 0; % deg True Anomoly

relative_state_chaser = [ % in CW reference frame centered at target
    0; % x in m, zenith direction
    -500; % y in m, allong track, direction of motion positive
    0; % z in m, completes right hand coordinate system
    0; % u, m/s
    0; % v, m/s
    0  % w, m/s
    ]; % in m CW reference frame centered at target [x, y, z, u, v, w]

%% Simulation constants
simulation_time_step_large = 10; %s
simulation_time_step_small = 0.3; %s
time_out_time = 10000; %s
time_out_time = inf; %s

RotatingEarth = false;
controller = "LQR";
control_mode = "vel";

%% Initial conditions states
if ecc == 0 && incl == 0
    [r_ijk, v_ijk] = keplerian2ijk(a, ecc, incl, RAAN, argp, nu, 'truelon', nu); % ECI/IJK reference frame 
else
    [r_ijk, v_ijk] = keplerian2ijk(a, ecc, incl, RAAN, argp, nu); % ECI/IJK reference frame 
end
absolute_state_target = vertcat(r_ijk, v_ijk); % [x, y, z, u, v, w] [m, m, m, m/s, m/s, m/s]

absolute_state_chaser = absolute_state_target + CW2ECI(absolute_state_target, relative_state_chaser); % ECI reference frame [x, y, z, u, v, w] [m, m, m, m/s, m/s, m/s]

control_force = [0; 0; 0]; % [Fx, Fy, Fz] in N

earth_rotation = 0; %rad

%% Mission profile
mode = "Homing";
station_keeping_points = [
    -100 -1000 0;
    0 -500 0;
    0 -300 0;
    0 -150 0;
    0 -50 0;
    0 -15 0;
    0 -10 0;
    0 -7.5 0;
    0 -5 0;
    0 -2.5 0;
    0 -1 0;
    0 0 0
    ]; % [x, y, z] in m in CR RF

%% Global Variables
control_memory_variable = cell(0); % Variable to store controller initialization
control_force_history = control_force;
time_history = [0];
relative_state_chaser_history = relative_state_chaser;
desired_state_history = relative_state_chaser;
P = diag([1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4]);

% Attitude of the target s/c
alpha = 0;
beta = 0;
gamma = 0; 

running = true;
final_time_steps = false;
t = 0;
while running
    
    % Current Timestep Size
    if mode == "Homing"
        time_step = simulation_time_step_large;
    elseif mode == "Closing"
        time_step = simulation_time_step_large;
    elseif mode == "Final"
        control_mode = "pos";
        control_memory_variable = cell(0);
        time_step = simulation_time_step_small;
        if ~final_time_steps
            time_out_time = t + 1000 * simulation_time_step_small;
            final_time_steps = true;
            disp("switch to final steps")
        end
    end
    disp("Mode: " + mode)

    if t >= time_out_time
        running = false;
    end
    % Global variables
    mean_motion = sqrt(mu/vecnorm(absolute_state_chaser(1:3))^3);

    if mode == "Final"
        disp("Switching to VBN")
        [relative_state_chaser, absolute_state_chaser, P, alpha, beta, ...
            gamma, station_keeping_points, desired_relative_state, mode] = navigation_vbn( ...
             relative_state_chaser, ...
             absolute_state_chaser, ...
             absolute_state_target, ...
             control_force, ...
             P, ...
             alpha, ...
             beta, ...
             gamma, ...
             station_keeping_points ...
             );

        desired_relative_state = desired_relative_state'; % Transpose to make it a column vector

        % Control
        [control_force, control_memory_variable] = Control( ...
            control_memory_variable, ...
            relative_state_chaser, ...
            desired_relative_state, ...
            controller, ...
            time_step, ...
            mass, ...
            mean_motion, ...
            control_mode ...
            );

        % control_force(1,1) = -control_force(1,1);
    
        disp("beginning time: " + t)
        disp("rel state: " + relative_state_chaser)
        disp("desired rel state: " + desired_relative_state)
        disp("control force: " + control_force)
        
        
        % Convert control force from CW reference frame to ECI
        control_force_ECI = CW2ECI(absolute_state_target, control_force);
        %disp("control force ECI: " + control_force_ECI)
        disp("finished time: " + t)
    
        % Dynamics
        [absolute_state_chaser, absolute_state_target, earth_rotation] = ...
        Dynamics(...
            integrator, ...
            acceleration_model, ...
            absolute_state_chaser, ...
            absolute_state_target, ...
            control_force_ECI, ...
            mass, ...
            time_step, ...
            earth_rotation);
        
        if mode == "Docked"
            running = false;
            disp("DOCKED")
        end
        
        % Save variables for plotting
        desired_state_history = horzcat(desired_state_history, desired_relative_state);
        control_force_history = horzcat(control_force_history, control_force);
        relative_state_chaser_history = horzcat(relative_state_chaser_history, relative_state_chaser);
        time_history = horzcat(time_history, t);
        t = t + time_step;

    else 
        relative_state_chaser = ECI2CW(absolute_state_target, absolute_state_chaser - absolute_state_target);

        % Guidance
        [desired_relative_state, desired_acceleration, mode] = guidance( ...
            relative_state_chaser, ...
            station_keeping_points, ...
            mean_motion ...
            );
        desired_relative_state = desired_relative_state'; % Transpose to make it a column vector
        if desired_relative_state(4) > 1
            desired_relative_state(4) =  1;
        elseif desired_relative_state(4) < -1
            desired_relative_state(4) = -1;
        end

        % Control
        [control_force, control_memory_variable] = Control( ...
            control_memory_variable, ...
            relative_state_chaser, ...
            desired_relative_state, ...
            controller, ...
            time_step, ...
            mass, ...
            mean_motion, ...
            control_mode ...
            );

        % control_force(1,1) = -control_force(1,1);
    
        disp("beginning time: " + t)
        disp("rel state: " + relative_state_chaser)
        disp("desired rel state: " + desired_relative_state)
        disp("control force: " + control_force)
        
        
        % Convert control force from CW reference frame to ECI
        control_force_ECI = CW2ECI(absolute_state_target, control_force);
        %disp("control force ECI: " + control_force_ECI)
        disp("finished time: " + t)
    
        % Dynamics
        [absolute_state_chaser, absolute_state_target, earth_rotation] = ...
        Dynamics(...
            integrator, ...
            acceleration_model, ...
            absolute_state_chaser, ...
            absolute_state_target, ...
            control_force_ECI, ...
            mass, ...
            time_step, ...
            earth_rotation);
        
        if mode == "Docked"
            running = false;
        end
        
        % Save variables for plotting
        desired_state_history = horzcat(desired_state_history, desired_relative_state);
        control_force_history = horzcat(control_force_history, control_force);
        relative_state_chaser_history = horzcat(relative_state_chaser_history, relative_state_chaser);
        time_history = horzcat(time_history, t);
        t = t + time_step;

    end
    
    % Save variables for plotting
    desired_state_history = horzcat(desired_state_history, desired_relative_state);
    control_force_history = horzcat(control_force_history, control_force);
    relative_state_chaser_history = horzcat(relative_state_chaser_history, relative_state_chaser);
    time_history = horzcat(time_history, t);
    t = t + time_step;
end

PlotResults(relative_state_chaser_history, control_force_history, run_ID, time_history, desired_state_history, mass)
