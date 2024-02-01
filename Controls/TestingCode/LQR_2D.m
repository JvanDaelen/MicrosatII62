%% 2-D LQR implementation

close all

% System charasteristics
m = 1; % kg

% Initial Conditions
x0 = [1;  % position
      0]; % velocity 

% System Dynamics
A = [0   1;
     0   0]; % pass through the velocity term and discard the position term
B = [0; 
     1/m];
C = [1 0;
     0 1];
D = [0;
     0];


max_w = 5;
% Equal penalization Control Law
Q = [1 0;  % Penalize pos error
     0 1]; % Penalize velocity
R = [1];   % Penalize F
[t_e, y_e, F_e] = runLQR(A, B, Q, R, C, D, x0, m);

% Position penalization Control Law
Q = [max_w 0;  % Penalize pos error
     0 1]; % Penalize velocity
R = [1];   % Penalize F
[t_p, y_p, F_p] = runLQR(A, B, Q, R, C, D, x0, m);

% Velocity penalization Control Law
Q = [1 0;  % Penalize pos error
     0 max_w]; % Penalize velocity
R = [1];   % Penalize F
[t_v, y_v, F_v] = runLQR(A, B, Q, R, C, D, x0, m);

% Force penalization Control Law
Q = [1 0;  % Penalize pos error
     0 1]; % Penalize velocity
R = [max_w];   % Penalize F
[t_f, y_f, F_f] = runLQR(A, B, Q, R, C, D, x0, m);


% Forces on the object
figure('Name', "Position over time")
hold on
grid
hold on
plot(t_e, y_e(:,1))
plot(t_p, y_p(:,1))
plot(t_v, y_v(:,1))
plot(t_f, y_f(:,1))

xlabel("time [s]")
ylabel("position [m]")
legend({ ...
    'Equal weights', ...
    'Pos weight = 5', ...
    'Vel weight = 5', ...
    'For weight = 5', ...
        },'Location','northeast')

% Forces on the object
figure('Name', "Force over Time")
hold on
grid
hold on
plot(t(1:500), F_e(1:500))
plot(t(1:500), F_p(1:500))
plot(t(1:500), F_v(1:500))
plot(t(1:500), F_f(1:500))

xlabel("time [s]")
ylabel("force [N]")
legend({ ...
    'Equal weights', ...
    'Pos weight = 5', ...
    'Vel weight = 5', ...
    'For weight = 5', ...
        },'Location','southeast')


function [t, y, Fx] = runLQR(A, B, Q, R, C, D, x0, m)
K = lqr(A,B,Q,R);
% 
% Closed loop system
sys = ss((A - B*K), B, C, D);

% Run response to initial condition
t = 0:0.005:15;
[y,t,x] = initial(sys, x0, t);

% Plotting
% x_track = x(:,1);
% y_track = x(:,2);
% plot(x_track,y_track)

% Total impulse
dt = (t(2)-t(1));
Fx = [0; m*diff(y(:,2))/dt]; % N
end