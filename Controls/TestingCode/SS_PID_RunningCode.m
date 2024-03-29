% PID controller using state space
close all

%% Input
% Simulation constants
orbital_altitude = 500 ;%km
mu = 3.986004418e+14 ;%m3s-2
a = 6378e+3 + orbital_altitude*1e+3; % Semi-major axis
n = sqrt(mu/a^3);
mass = 1; %kg
AOCS_loop_timestep = .3; % seconds per full AOCS loop
desired_relative_state = [0 0 0 0 0 0];
relative_state_initial = [5 0 0 0 0 0];

% Simulink settings
number_of_runs = 1;


[F, T] = MSAT_PID_SS(relative_state_initial, desired_relative_state, n, mass, AOCS_loop_timestep, number_of_runs);

function [F, T] = MSAT_PID_SS(relative_state_initial, desired_relative_state, n, mass, AOCS_loop_timestep, number_of_runs)
    %% Initialization of PID variables and constants
    relative_state = relative_state_initial;
    steps_per_run = 10; % nr of steps per chunk
    
    % PID parameters
    Pterm = .08; % .8
    Iterm = .0; % .08
    Dterm = 1; % 2
    Nterm = 4;
    
    % PID initial values
    integral_term = [0 0 0];
    differential_term = differential_term_vector(relative_state, desired_relative_state, Nterm, Dterm);
    
    % SS parameters
    A = [0     0   0    1    0   0; 
        0     0   0    0    1   0;
        0     0   0    0    0   1;
        3*n^2 0   0    0    2*n 0;
        0     0   0    -2*n 0   0;
        0     0   -n^2 0    0   0]; % pass through the velocity term and discard the position term
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
    
    % Memory variables
    state_memory = relative_state_initial;
    control_force_memory = [0 0 0];
    
    disp(differential_term)
    %% Run reference run
    run_time = AOCS_loop_timestep*number_of_runs;
    step_size = AOCS_loop_timestep/steps_per_run; %s
    simOut = sim("SS_PID_Simulink.slx", 'SrcWorkspace', 'current');
    yout = simOut.yout;

    % Extract Final result of reference run
    state_timeseries_reference = yout.getElement('state').Values;
    control_force_reference = yout.getElement('Control Forces').Values;
    
    %% Run simulink
    % Derived simulink settings
    run_time = AOCS_loop_timestep;
    simulation_final_time =  AOCS_loop_timestep*number_of_runs; %s
    step_size = AOCS_loop_timestep/steps_per_run; %s
    
    for i = 1:1:number_of_runs
        disp("currently running run:")
        disp(i)
        differential_term = differential_term_vector(relative_state, desired_relative_state, Nterm, Dterm);
        simOut = sim("SS_PID_Simulink.slx", 'SrcWorkspace', 'current');
        yout = simOut.yout;
        
        % Extract Final result
        state_timeseries = yout.getElement('state').Values;
        control_force_timeseries = yout.getElement('Control Forces').Values;
        final_integral_terms = yout.getElement('Error Integral').Values.data(end,1:3);
        
        % Convert to outputs for the dynamics simulator
        control_force = trapz(control_force_timeseries.Time, control_force_timeseries.data) / run_time;
        disp(control_force)
        
        % Update relative state and PID initials for next run
        relative_state = state_timeseries.data(end,:);
        integral_term = integral_term + final_integral_terms * Iterm;  % 
        
        % Save data in memory
        state_memory = vertcat(state_memory, state_timeseries.data(2:end,:));
        control_force_memory = vertcat(control_force_memory, control_force_timeseries.data(1:end-1,:));
    end
    
    %% Plot Results
    t_reconstruct = 0:step_size:simulation_final_time;
    PlotTrajectory(state_timeseries_reference, t_reconstruct, state_memory);
    PlotVelocity(state_timeseries_reference, t_reconstruct, state_memory);
    PlotControlForce(control_force_reference, t_reconstruct, control_force_memory);

    %% Generate outputs
    F = [0 0 0];
    T = [0 0 0];
end

function PlotControlForce(control_force_reference, t_reconstruct, control_force_memory)
figure('Name', "Control")
plot(control_force_reference.Time, vertcat([0 0 0], control_force_reference.Data(1:end-1,:)))
grid
hold on
plot(t_reconstruct, control_force_memory, '--')
xlabel("time [s]")
ylabel("control force [N]")
legend({ ...
    'x', ...
    'y', ...
    'z', ...
    'x chunks', ...
    'y chunks', ...
    'z chunks' ...
    },'Location','northeast')
end

function PlotVelocity(state_timeseries_reference, t_reconstruct, state_memory)
figure('Name', "Velocity")
plot(state_timeseries_reference.Time, state_timeseries_reference.Data(:, 4:6))
grid
hold on
plot(t_reconstruct, state_memory(:,4:6), '--')
xlabel("time [s]")
ylabel("velocity [m/s]")
legend({ ...
    'x', ...
    'y', ...
    'z', ...
    'x runs', ...
    'y runs', ...
    'z runs' ...
    },'Location','northeast')
end

function PlotTrajectory(state_timeseries_reference, t_reconstruct, state_memory)
figure('Name', "Trajectory")
plot(state_timeseries_reference.Time, state_timeseries_reference.Data(:, 1:3))
grid
hold on
plot(t_reconstruct, state_memory(:,1:3), '--')
xlabel("time [s]")
ylabel("position [m]")
legend({ ...
    'x', ...
    'y', ...
    'z', ...
    'x runs', ...
    'y runs', ...
    'z runs' ...
    },'Location','northeast')
end

function differential_term = differential_term_vector(relative_state, desired_state, Nterm, Dterm)
error = desired_state - relative_state;
disp(error(1))
disp(relative_state(4))
differential_term = [
    differential_term_element(error(1), relative_state(4), Nterm, Dterm)
    differential_term_element(error(2), relative_state(5), Nterm, Dterm)
    differential_term_element(error(3), relative_state(6), Nterm, Dterm)
    ];
end

function D0 = differential_term_element(pos_error, vel, N, D)
    D0 = (pos_error + vel/N)*D;
end










