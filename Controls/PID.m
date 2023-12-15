% PID controller

close all

% Simulation constants
orbital_period = 90*60; %s
mass = 1; %kg

n_chunks = 100;
Tfinal = 0.1*n_chunks; %s
Tstep = 0.05; %s

% Input
relative_position = [5 10 7 0 0 0];
desired_position = [0 2 -3 0 0 0];
integral_term = 0;

simOut = sim("PID_Simulink.slx");
yout = simOut.yout;

position_long = yout.getElement('Position').Values;
control_force_long = yout.getElement('Control Force').Values;
integral_term_long = simOut.PID_I_term;


Tfinal = Tfinal/n_chunks; %s

t = 0:Tfinal:Tfinal*n_chunks;

position_record = [relative_position(1:3)];
control_force_record = [0 0 0];


for i = 1:1:n_chunks
    simOut = sim("PID_Simulink.slx");
    yout = simOut.yout;
    
    position = yout.getElement('Position').Values;
    control_force = yout.getElement('Control Force').Values;
    integral_term = simOut.PID_I_term(end-1) + integral_term

    position_record(i + 1, 1:3) = position.data(end, :);
    relative_position(1:3) = position.data(end, :);
end


figure('Name', "Trajectory")
plot(position_long)
grid
hold on
plot(t, position_record, '--')
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

figure('Name', "Control")
plot(control_force_long)
grid
hold on
xlabel("time [s]")
ylabel("control force [N]")
legend({ ...
        'x', ...
        'y', ...
        'z' ...
            },'Location','northeast')

% Run response to initial condition
% t = 0:0.005:50;
% dt = 0.005;
% sys_ss = ss_conversion(controlloop);
% [y,t,x] = initial(sys_ss, x0, [0 dt]);  % Initial conditions: [state (error), integral (int of error), derivative (der of error)]
% 
% plot(t,y)