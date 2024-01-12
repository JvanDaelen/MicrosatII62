% Master Simulation for Microsat
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
    -1e+3; % y in m, allong track, direction of motion positive
    0; % z in m, completes right hand coordinate system
    0; % u, m/s
    0; % v, m/s
    0  % w, m/s
    ]; % in m CW reference frame centered at target [x, y, z, u, v, w]

%% Simulation constants
simulation_time_step_large = 0.3; %s
simulation_time_step_small = 0.3; %s

RotatingEarth = False;

%% Initial conditions states
[r_ijk, v_ijk] = keplerian2ijk(a, ecc, incl, RAAN, argp, nu); % ECI/IJK reference frame 
absolute_state_target = vertcat(r_ijk, v_ijk); % [x, y, z, u, v, w] [m, m, m, m/s, m/s, m/s]

absolute_state_chaser = absolute_state_target + CW2ECI6DOF(relative_state_chaser); % ECI reference frame [x, y, z, u, v, w] [m, m, m, m/s, m/s, m/s]

control_force = [0; 0; 0]; % [Fx, Fy, Fz] in N

earth_rotation = 0; %rad

%% Mission profile
mode = "Homing";
station_keeping_points = [
    0 0 0;
    1 1 1
    ]; % [x, y, z] in m in CR RF

%% Global Variables
control_memory_variable = cell(1); % Variable to store controller initialization
control_force_history = control_force;
relative_state_chaser_history = relative_state_chaser;

running = True;
while running
    % Current Timestep Size
    if mode == "Homing"
        time_step = simulation_time_step_large;
    elseif mode == "Closing"
        time_step = simulation_time_step_large;
    elseif mode == "Final"
        time_step = simulation_time_step_small;
    end
    % Global variables
    mean_motion = sqrt(mu/vecnorm(absolute_state_chaser(1:3))^3);
    
    % Navigation
    [relative_state_chaser, absolute_state_chaser] = Navigation( ...
        relative_state_chaser, ...
        absolute_state_chaser, ...
        absolute_state_target, ...
        control_force ...
        );

    % Guidance
    [desired_relative_state, desired_acceleration, mode] = Guidance( ...
        relative_state_chaser, ...
        station_keeping_points, ...
        mean_motion ...
        );

    % Control
    [control_force, control_memory_variable] = Control( ...
        control_memory_variable, ...
        relative_state_chaser, ...
        desired_relative_state, ...
        controller, ...
        time_step, ...
        mass, ...
        mean_motion ...
        );
    
    % Convert control force from CW reference frame to ECI
    control_force_ECI = CW2ECI3DOF(control_force);

    % Dynamics
    absolute_state_chaser = propagateSpacecraft_FE_Kepler( ...
        absolute_state_chaser, ...
        control_force_ECI, ...
        mass, ...
        time_step ...
        );
    absolute_state_target = propagateSpacecraft_FE_Kepler( ...
        absolute_state_target, ...
        control_force_ECI, ...
        mass, ...
        time_step ...
        );
    if RotatingEarth
        earth_rotation = propagateEarth(earth_rotation);
    end
    
    if mode == "Docked"
        running = False;
    end
    
    % Save variables for plotting
    control_force_history = horzcat(control_force_history, control_force);
    relative_state_chaser_history = horzcat(relative_state_chaser_history, relative_state_chaser);
end


%% Plotting results
% figure_name = "Trajectory"
% figure('Name', figure_name)
% plot(state_timeseries_reference.Time, state_timeseries_reference.Data(:, 1:3))
% grid
% hold on
% plot(t_reconstruct, state_memory(:,1:3), '--')
% xlabel("time [s]")
% ylabel("position [m]")
% legend({ ...
%     'x', ...
%     'y', ...
%     'z', ...
%     'x runs', ...
%     'y runs', ...
%     'z runs' ...
%     },'Location','northeast')
% saveas(gcf, figure_name)