% PID controller using state space
close all

% Simulation constants
total_time = 5;
u = 0;
x = [1; 0];

% PID parameters
P = .8; % .8
I = .0; % .08
D = 1; % 2
N = 4;

% PID initial values
I0 = 0;
D0 = ( u - x(1) + x(2)/N ) / D;

% Run reference run
run_time = total_time;
simOut = sim("PID_UnitTest.slx", 'SrcWorkspace', 'current');
yout = simOut.yout;

% Extract Final result of reference run
reference_state = yout.getElement('State').Values.data(:,1);
reference_force = yout.getElement('Force').Values.data;

% Memory variables
state_memory = x(1);
force_memory = 0;

for i = 1:1:5
    run_time = 1;
    simOut = sim("PID_UnitTest.slx", 'SrcWorkspace', 'current');
    yout = simOut.yout;
    
    % Extract Final result of reference run
    step_state = yout.getElement('State').Values.data;
    step_force = yout.getElement('Force').Values.data;
    
    % Memory variables
    if i == 1
        state_memory = step_state(:,1);
    else
        state_memory = vertcat(state_memory, step_state(2:end,1));
    end
    force_memory = vertcat(force_memory, step_force(2:end));
    
    x = step_state(end,:);
    I0 = I0 + yout.getElement('Integral').Values.data(end) * I;
    D0 = ( u - x(1) + x(2)/N ) / D;
end


x = [1; 0];
% PID initial values
I0 = 0;
D0 = ( u - x(1) + x(2)/N ) / D;
avg_state_memory = x(1);
avg_force_memory = 0;
avg_pos_memory = x(1);

for i = 1:1:5
    run_time = 1;
    simOut = sim("PID_UnitTest.slx", 'SrcWorkspace', 'current');
    yout = simOut.yout;
    
    % Extract Final result of reference run
    step_state = yout.getElement('State').Values.data;
    step_force = yout.getElement('Force').Values.data;
    
    % Memory variables
    if i == 1
        avg_state_memory = step_state(:,1);
    else
        avg_state_memory = vertcat(avg_state_memory, step_state(2:end,1));
    end
    avg_force_memory = vertcat(avg_force_memory, step_force(2:end));
    
    disp(mean(step_force))
    vel_change = mean(step_force) * 1;
    disp(vel_change)
    pos_change = x(2) + 0.5 * mean(step_force) * 1;
    x(2) = vel_change + x(2);
    x(1) = pos_change + x(1);
    
    % x = step_state(end,:);
    avg_pos_memory(i+1) = x(1);
    I0 = I0 + yout.getElement('Integral').Values.data(end) * I;
    D0 = ( u - x(1) + x(2)/N ) / D;
end


figure_name = "Forces";
figure('Name', figure_name)

% Postion variable to plot
t = 0:0.01:5;

plot(t, reference_force, 'b-')
hold on
grid on
plot(t, force_memory, 'r-')

average_reference = [0];
average_force = [0];
avg_pos_mem = [state_memory(1)];
avg_vel_mem = [0];
for i = 0:1:4
    mean_ref = mean(reference_force(i*100 + 1 : (i+1)*100 + 1));
    average_reference = horzcat(average_reference, mean_ref);
    average_reference = horzcat(average_reference, mean_ref);

    mean_for = mean(force_memory(i*100 + 1 : (i+1)*100 + 1));
    average_force = horzcat(average_force, mean_for);
    average_force = horzcat(average_force, mean_for);

end

plot([0 0 1 1 2 2 3 3 4 4 5], average_reference, 'b--')
plot([0 0 1 1 2 2 3 3 4 4 5], average_force, 'r--')

legend({ ...
    'continues', ...
    'discretized', ...
    'continues avg', ...
    'discretized avg', ...
        },'Location','northeast')

xlabel("time [s]")
ylabel("control force [N]")

saveas(gcf, "PID_Unit_For.png")


figure_name = "Trajectory";
figure('Name', figure_name)

% Postion variable to plot
t = 0:0.01:5;

plot(t, reference_state, 'b-')
hold on
grid on
plot(t, state_memory, 'r-')
plot(0:1:5, avg_pos_memory, 'r--')


legend({ ...
    'continues', ...
    'discretized', ...
    'discretized cons'
        },'Location','northeast')

xlabel("time [s]")
ylabel("pos  [m]")

saveas(gcf, "PID_Unit_Pos.png")

