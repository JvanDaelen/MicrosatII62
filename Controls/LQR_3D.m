%% 2-D LQR implementation

close all

% System charasteristics
m = 5; % kg

% Initial Conditions
x0 = [3;  % position x [m]
      0;  % position y [m]
      0;  % position z [m]
      1;  % velocity x [m/s]
      0;  % velocity y [m/s]
      1]; % velocity z [m/s]

% X dot vector:
    % velocity x
    % velocity y
    % velocity y
    % acceleration x
    % acceleration y
    % acceleration y


% System Dynamics
A = [0    0   0   1  0  0; 
     0    0   0   0  1  0;
     0    0   0   0  0  1;
     0    0   0   0  0  0;
     0    0   0   0  0  0;
     0    0   0   0  0  0]; % pass through the velocity term and discard the position term
B = [0    0   0; 
     0    0   0;
     0    0   0;
     1/m  0   0;
     0    1/m 0;
     0    0   1/m];
C = eye(6);
    % [1 0 0 0
    %  0 1 0 0
    %  0 0 1 0
    %  0 0 0 1];
D = [0 0 0;
     0 0 0;
     0 0 0;
     0 0 0;
     0 0 0;
     0 0 0];

% Control Law
Q = [.1 0 0 0 0 0;  % Penalize x error
     0 .1 0 0 0 0;  % Penalize y error
     0 0 .1 0 0 0;  % Penalize z error
     0 0 0 1 0 0;  % Penalize x velocity
     0 0 0 0 1 0;  % Penalize y velocity
     0 0 0 0 0 1]; % Penalize z velocity
R = [2 0 0;      % Penalize Fx
     0 2 0;      % Penalize Fy
     0 0 2];     % Penalize Fz
K = lqr(A,B,Q,R);
% 
% Closed loop system
sys = ss((A - B*K), B, C, D);

% Run response to initial condition
t = 0:0.005:50;
[y,t,x] = initial(sys, x0, t);


% Total impulse
dt = (t(2)-t(1));
Fx = [0; m*diff(y(:,4))/dt]; % N
Fuel_x = sum(abs(Fx)) * dt; % Ns
Fy = [0; m*diff(y(:,5))/dt]; % N
Fuel_y = sum(abs(Fy)) * dt; % Ns
Fz = [0; m*diff(y(:,6))/dt]; % N
Fuel_z = sum(abs(Fz)) * dt; % Ns
Fuel_total = Fuel_x + Fuel_y + Fuel_z; % Ns
disp(Fuel_total)


% Forces on the object
figure('Name', "Position over time")
hold on
grid
hold on
plot(t, y(:,1))
plot(t, y(:,2))
plot(t, y(:,3))

xlabel("time [s]")
ylabel("position [m]")
legend({ ...
    'Position x', ...
    'Position y' ...
    'Position z' ...
        },'Location','northeast')

% Forces on the object
figure('Name', "Force over Time")
hold on
grid
hold on
plot(t, Fx)
plot(t, Fy)
plot(t, Fz)

xlabel("time [s]")
ylabel("force [N]")
legend({ ...
    'F_x', ...
    'F_y' ...
    'F_z' ...
        },'Location','northeast')

% Plotting trajecotry
figure('Name', "Trajectory")
plot3(y(:,1),y(:,2),y(:,3))
% comet3(y(:,1),y(:,2),y(:,3),0.1)
grid
hold on
% plot3(0*y(:,1),y(:,2),y(:,3), 'color', 'k','LineWidth',1); % shadow on the Y-Z plane
% plot3(y(:,1),0*y(:,2),y(:,3), 'color', 'k','LineWidth',1); % shadow on the X-Z plane
plot3(y(:,1),y(:,2),0*y(:,3), 'color', 'k','LineWidth',1); % shadow on the X-Y plane

xlim([-10 10])
ylim([-10 10])
zlim([-10 10])

xlabel("position x [m]")
ylabel("position y [m]")
zlabel("position z [m]")

legend({ ...
    'Trajectory', ...
    'Projection on X-Y plane', ...
        },'Location','northeast')

