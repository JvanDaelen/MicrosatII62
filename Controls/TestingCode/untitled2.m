% H inf try 2.0

% H_infinity example

% Physical parameters
mb = 300;    % kg
mw = 1;     % kg
bs = 1;   % N/m/s
ks = 1 ; % N/m
kt = 1; % N/m

% State matrices
A = [ 0 1 0 0; [-ks -bs ks bs]/mb ; ...
      0 0 0 1; [ks bs -ks-kt -bs]/mw];
B = [ 0 0; 0 1e3/mb ; 0 0 ; [kt -1e3]/mw];
C = [1 0 0 0; 1 0 -1 0; A(2,:)];
D = [0 0; 0 0; B(2,:)];

qcar = ss(A,B,C,D);
qcar.StateName = {'body travel (m)';'body vel (m/s)';...
          'wheel travel (m)';'wheel vel (m/s)'};
qcar.InputName = {'r';'fs'};
qcar.OutputName = {'xb';'sd';'ab'};

% Nominal Actuator Model
ActNom = tf(1, [1/60 1]);
ActNom.InputName = 'u';
ActNom.OutputName = 'fs';

% Weights
Wroad = ss(0.07);  Wroad.u = 'd';   Wroad.y = 'r';
Wact = 0.8*tf([1 50],[1 500]);  Wact.u = 'u';  Wact.y = 'e1'; % high pass filter

HandlingTarget = 0.04 * tf([1/8 1],[1/80 1]);
ComfortTarget = 0.4 * tf([1/0.45 1],[1/150 1]);

% Three design points
beta = reshape([0.01 0.5 0.99],[1 1 3]);
Wsd = beta / HandlingTarget;
Wsd.u = 'sd'; Wsd.y = 'e3';
Wab = (1-beta) / ComfortTarget;
Wab.u = 'ab';  Wab.y = 'e2';

sdmeas  = sumblk('y1 = sd');
abmeas = sumblk('y2 = ab');
ICinputs = {'d';'u'};
ICoutputs = {'e1';'e2';'e3';'y1';'y2'};
qcar(1:3,:)
qcaric = connect(qcar(2:3,:),ActNom,Wroad,Wact,Wab,Wsd,...
                 sdmeas,abmeas,ICinputs,ICoutputs);

ncont = 1; % one control signal, u
nmeas = 2; % two measurement signals, sd and ab
K = ss(zeros(ncont,nmeas,3));
gamma = zeros(3,1);
for i=1:3
   [K(:,:,i),~,gamma(i)] = hinfsyn(qcaric(:,:,i),nmeas,ncont);
end

% Closed-loop models
K.u = {'sd','ab'};  K.y = 'u';
CL = connect(qcar,ActNom,K,'r',{'xb';'sd';'ab'});

% Road disturbance
t = 0:0.0025:1;
roaddist = zeros(size(t));
roaddist(1:101) = 0.025*(1-cos(8*pi*t(1:101)));

% Simulate
p1 = lsim(qcar(:,1),roaddist,t); % open loop
y1 = lsim(CL(1:3,1,1),roaddist,t); % comfort
y2 = lsim(CL(1:3,1,2),roaddist,t); % balanced
y3 = lsim(CL(1:3,1,3),roaddist,t); % handling

% Plot results
plot(t,p1(:,1),'b',t,y1(:,1),'r.',t,y2(:,1),'m.',t,y3(:,1),'k.',t,roaddist,'g')
title('Body travel'), ylabel('x_b (m)')