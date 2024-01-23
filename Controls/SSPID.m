function [F, T, control_memory_variable] = SSPID(relative_state_initial, desired_relative_state, n, mass, main_time_step, control_memory_variable)
%SSPID Simulink based discretized PID controller

% Initialization of PID variables and constants
relative_state = relative_state_initial;
steps_per_run = 10; % nr of steps per chunk
% Pterm = .08; % .8
% Iterm = .0; % .08
% Dterm = 1; % 2
% Nterm = 4;

Pterm = .8; % .8
Iterm = .08; % .08
Dterm = 1; % 2
Nterm = 4;

% SS matrices
A = [0     0   0    1    0   0; 
    0     0   0    0    1   0;
    0     0   0    0    0   1;
    3*n^2 0   0    0    2*n 0;
    0     0   0    -2*n 0   0;
    0     0   -n^2 0    0   0];
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

% Derived simulink settings
run_time = main_time_step; % s Length of simulink simulatiom
step_size = main_time_step/steps_per_run; % s Step sized used for simulink simulation

% PID initial values
integral_term = control_memory_variable{1};
differential_term = differential_term_vector(relative_state, desired_relative_state, Nterm, Dterm);

% Run simulink simulation
simOut = sim("SS_PID_Simulink.slx", 'SrcWorkspace', 'current');
yout = simOut.yout;

% Extract Final result
control_force_timeseries = yout.getElement('Control Forces').Values;
final_integral_terms = yout.getElement('Error Integral').Values.data(end,1:3);

% Update PID initial integral value for next run
control_memory_variable(1) = {integral_term + final_integral_terms * Iterm}; 

% Generate outputs
F = trapz(control_force_timeseries.Time, control_force_timeseries.data) / run_time;
T = [0 0 0];
end

function differential_term = differential_term_vector(relative_state, desired_state, Nterm, Dterm)
error = desired_state - relative_state;
differential_term = [
    differential_term_element(error(1), relative_state(4), Nterm, Dterm)
    differential_term_element(error(2), relative_state(5), Nterm, Dterm)
    differential_term_element(error(3), relative_state(6), Nterm, Dterm)
    ];
end

function D0 = differential_term_element(pos_error, vel, N, D)
    D0 = (pos_error + vel/N)*D;
end