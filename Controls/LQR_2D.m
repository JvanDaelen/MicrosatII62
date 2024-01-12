%% 2-D LQR implementation

close all

% System charasteristics
m = 1; % kg

% Initial Conditions
x0 = [3;  % position x [m]
      2;  % position y [m]
      1;  % velocity x [m/s]
      0]; % velocity y [m/s]

% X dot vector:
    % velocity x
    % velocity y
    % acceleration x
    % acceleration y


% System Dynamics
A = [0    0   1   0; 
     0    0   0   1;
     0    0   0   0;
     0    0   0   0]; % pass through the velocity term and discard the position term
B = [0    0; 
     0    0;
     1/m  0;
     0    1/m];
C = [1 0 0 0
     0 1 0 0
     0 0 1 0
     0 0 0 1];
D = [0 0;
     0 0;
     0 0;
     0 0];

% Control Law
Q = [1 0 0 0;  % Penalize x error
     0 1 0 0;  % Penalize y error
     0 0 5 0;  % Penalize x velocity
     0 0 0 5]; % Penalize y velocity
R = [5 0;      % Penalize Fx
     0 5];     % Penalize Fy
K = lqr(A,B,Q,R);
% 
% Closed loop system
sys = ss((A - B*K), B, C, D);

% Run response to initial condition
t = 0:0.005:30;
[y,t,x] = initial(sys, x0, t);

% Plotting
x_track = x(:,1);
y_track = x(:,2);
plot(x_track,y_track)

% Total impulse
dt = (t(2)-t(1));
Fx = [0; m*diff(y(:,3))/dt]; % N
Fy = [0; m*diff(y(:,4))/dt]; % N


% Forces on the object
figure('Name', "Position over time")
hold on
grid
hold on
plot(t, y(:,1))
plot(t, y(:,2))

xlabel("time [s]")
ylabel("position [m]")
legend({ ...
    'Position x', ...
    'Position y' ...
        },'Location','northeast')

% Forces on the object
figure('Name', "Force over Time")
hold on
grid
hold on
plot(t, Fx)
plot(t, Fy)

xlabel("time [s]")
ylabel("force [N]")
legend({ ...
    'F_x', ...
    'F_y' ...
        },'Location','northeast')

