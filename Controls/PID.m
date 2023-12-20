% PID controller

close all

% Simulation constants
orbital_period = 90*60; %s
mass = 1; %kg

n_chunks = 50;
chunk_resolution = 10; % nr of steps per chunk
full_simulation_timestep = .3; %s for full guidance loop
simulation_final_time =  full_simulation_timestep*n_chunks; %s

Tstep = full_simulation_timestep/chunk_resolution; %s
chunk_time = full_simulation_timestep;


Tfinal = simulation_final_time; %s

% Input
relative_position = [.2 0 0 .1 0 0];
relative_position_const = relative_position;
desired_position = [0 0 0 0 0 0];


P = 10;
I = 5;
D = 1;
N = 20;

% Initialize timeseries variables
error_timeseries = timeseries((1:5)',[0 10 20 30 40],"Name","error");
plant_input = timeseries((1:5)',[0 10 20 30 40],"Name",'PlantInput');
TControlStart = -1;

integral_term = 0;
differential_term = D * N / (1 + N*relative_position(4));
tracking_signal = 0;

simOut = sim("PID_Simulink.slx");
yout = simOut.yout;

position_long = yout.getElement('State').Values;
control_force_long = yout.getElement('Control Force').Values;
integral_term_long = simOut.PID_I_term;
differential_term_long = simOut.PID_D_term;

t_reconstruct = 0:Tstep:Tfinal;
Tfinal = Tfinal/n_chunks; %s


% Initialize record variables
position_record = [relative_position(1:3)];
velocity_record = [relative_position(4)];
control_force_record = [0 0 0];

% Initialize timeseries variables
error_timeseries = timeseries((1:5)',[0 10 20 30 40],"Name","error");
plant_input = timeseries((1:5)',[0 10 20 30 40],"Name",'PlantInput');

%%% Run first simulation chunk
% Set simulation final time
Tfinal = simulation_final_time/n_chunks;
TControlStart = -1;

% Initiate PID initialization terms
% differential_term_record = D * N / (1 + N*relative_position(4));
% integral_term_chunk = 0;

% Run PID simulink model
simOut = sim("PID_Simulink.slx");
yout = simOut.yout;

% Extract sim results timeseries
position = yout.getElement('State').Values; % timeseries [x,y,z,u,v,w]
control_force = yout.getElement('Control Force').Values; % timeseries [Fx, Fy, Fz]
% integral_term = simOut.PID_I_term(end);
% differential_term = simOut.PID_D_term(end);
error_timeseries = simOut.error;
plant_input = simOut.PlantInput;

% Save results to the records
position_record = vertcat(position_record, position.data(end-chunk_resolution+1:end, 1:3));
velocity_record = vertcat(velocity_record, position.data(end-chunk_resolution+1:end, 4));
control_force_record = vertcat(control_force_record, control_force.data(end-chunk_resolution+1:end, 1:3));
% differential_term_record = vertcat(differential_term_record, simOut.PID_D_term(end-chunk_resolution+1:end));

% Update the relative position variable
relative_position(1:4) = position.data(end, 1:4);

% Display results
disp(position)


%%% Run 2nd chunk
for i = 2:1:n_chunks
    disp('Running chunk')
    disp(i)

    % Set simulation final time
    Tfinal = i*simulation_final_time/n_chunks;
    TControlStart = (i-1)*chunk_time;
    
    % Initiate PID initialization terms
    % differential_term_record = D * N / (1 + N*relative_position(4));
    % integral_term_chunk = 0;
    
    % Run PID simulink model
    simOut = sim("PID_Simulink.slx");
    yout = simOut.yout;
    
    % Extract sim results timeseries
    position = yout.getElement('State').Values; % timeseries [x,y,z,u,v,w]
    control_force = yout.getElement('Control Force').Values; % timeseries [Fx, Fy, Fz]
    error_timeseries = simOut.error;
    plant_input = simOut.PlantInput;
    
    % Save results to the records
    position_record = vertcat(position_record, position.data(end-chunk_resolution+1:end, 1:3));
    velocity_record = vertcat(velocity_record, position.data(end-chunk_resolution+1:end, 4));
    control_force_record = vertcat(control_force_record, control_force.data(end-chunk_resolution+1:end, 1:3));
    % differential_term_record = vertcat(differential_term_record, simOut.PID_D_term(2:end));
    
    % Update the relative position variable
    relative_position(1:4) = position.data(end, 1:4);
end




figure('Name', "Trajectory")
plot(position_long.Time, position_long.Data(:, 1:3))
grid
hold on
plot(t_reconstruct, position_record, '--')
xlabel("time [s]")
ylabel("position [m]")
legend({ ...
        'x', ...
        'y', ...
        'z', ...
        'x chunks', ...
        'y chunks', ...
        'z chunks' ...
            },'Location','northeast')

figure('Name', "Velocity")
plot(position_long.Time, position_long.Data(:, 4))
grid
hold on
plot(t_reconstruct, velocity_record, '--')
xlabel("time [s]")
ylabel("velocity [m/s]")
legend({ ...
        'x', ...
        'x chunks'
            },'Location','northeast')


figure('Name', "Control")
plot(control_force_long)
grid
hold on
plot(t_reconstruct, control_force_record, '--')
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
