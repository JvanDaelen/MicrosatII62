close all

relative_state_chaser = [0; 0; 0; 0; 0; 0];
desired_relative_state = [1; 1; 1; .5; .5; .5];

time_step = 0.25;

mass = 1;
mean_motion = 1e-5;
n = mean_motion;

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



control_memory_variable = cell(0);
close_system("SS_PID_Simulink.slx", 0)
close_system("VEL_SS_PID_Simulink.slx", 0)

control_memory_variable = cell(0); % Variable to store controller initialization
control_force_history = [0;0;0];
time_history = [0];


state_mem = relative_state_chaser;
desired_mem = relative_state_chaser;

t = 0;
t_mem = [t];

PID_poscontrol_memory_variable = cell(0); % Variable to store controller initialization
PID_pos_state = relative_state_chaser;
PID_pos_state_mem = desired_relative_state(4);
for i = 1:20/time_step
    if t >= 10
        desired_relative_state = [2;2;2;0;0;0];
    end
    
    % PID pos mode
    [F, PID_poscontrol_memory_variable] = Control( ...
        PID_poscontrol_memory_variable, ...
        PID_pos_state, ...
        desired_relative_state, ...
        "LQR", ...
        time_step, ...
        mass, ...
        mean_motion, ...
        "pos" ...
        );
    PID_pos_state = PID_pos_state + ( A * PID_pos_state + B * F ) * time_step;
    disp(t)
    disp(PID_pos_state(1))
    disp(PID_pos_state(4))
    PID_pos_state_mem = horzcat(PID_pos_state_mem, PID_pos_state(4));
    
    
    desired_mem = horzcat(desired_mem, desired_relative_state);
    t_mem = horzcat(t_mem, t);
    t = t + time_step;
end

figure_name = "Trajectory";
figure('Name', figure_name)

% Postion variable to plot
plot(t_mem, desired_mem(4,:), 'r--')
hold on
grid on
plot(t_mem, PID_pos_state_mem, 'b-')

legend({ ...
    'target', ...
    'state', ...
        },'Location','northeast')

xlabel("time [s]")
ylabel("vel [m/s]")

% saveas(gcf, "PID_TestResult_Pos.png")


% figure_name = "Forces";
% figure('Name', figure_name)
% 
% % Postion variable to plot
% t = 0:0.01:5;
% 
% plot(t, reference_force, 'b-')
% hold on
% grid on
% plot(t, force_memory, 'r-')
% 
% 
% 
% legend({ ...
%     'continues', ...
%     'discretized', ...
%     'continues avg', ...
%     'discretized avg', ...
%         },'Location','northeast')
% 
% xlabel("time [s]")
% ylabel("pos  [m]")
% 
% saveas(gcf, "PID_Unit_For.png")

