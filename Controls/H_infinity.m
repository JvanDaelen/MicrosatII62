% H infinity controller try

% System charasteristics
m = 1000; % kg

% Initial Conditions
x0 = [3;  % position x [m]
      2;  % position y [m]
      1;  % velocity x [m/s]
      0]; % velocity y [m/s]

% State matrices
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

sys_ss = ss(A,B,C,D);
size(sys_ss); % 4 outputs, 2 inputs, 4 states
sys_ss.StateName = {'x_pos';'y_pos';...
          'x_vel';'y_vel'};
sys_ss.InputName = {'f_x';'f_y'};
sys_ss.OutputName = {'x_vel';'x_acc';'y_vel'; 'y_acc'};

ncont = 1; % one control signal, f_x
nmeas = 1; % one measurement signal, x_pos

[K1,~,gamma] = hinfsyn(sys_ss,nmeas,ncont);

size(K1); % 4 outputs, 2 inputs, 4 states

ActNom = tf(1, [1/60 1]);
ActNom.InputName = 'u';
ActNom.OutputName = 'f_x';

% Control objective: low accelerations
% Measurement of acceleration
% Disturbance: different position x_pos
% No sensor noise for now

Wpos = ss(0.07); Wpos.u = 'd1'; Wpos.y= 'x_pos';
Wact = ss(0.07); Wact.u = 'u'; Wact.y = 'e1';
Wx_acc = ss(0.07); Wx_acc.u = 'x_acc'; Wx_acc.y = 'e2';

HandlingTarget = 0.04 * tf([1/8 1],[1/80 1]);

x_acc_meas = sumblk('y1 = x_acc');

ICinputs = {'d1','u'}; %not of correct dimension
ICoutputs = {'e1'; 'e2';' y1'}; %not of correct dimension
sysic = connect(sys_ss(2,:),ActNom,Wpos,Wact,Wx_acc,x_acc_meas,ICinputs,ICoutputs);

% Closed-loop models
%K.u = {'x_acc'};  K.y = 'u';
%CL = connect(sys_ss,ActNom,K,'f_x',{'x_acc'});

% Offset x positio
%t = 0:0.0025:1;
%roaddist = zeros(size(t));
%roaddist(1:101) = 0.025*(1-cos(8*pi*t(1:101)));

% Simulate
%p1 = lsim(sys_ss,roaddist,t); % open loop
%y1 = lsim(CL(1,:),roaddist,t); % comfort






