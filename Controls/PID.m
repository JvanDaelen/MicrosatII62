% PID controller

% Input
x0 = [1 0 0];

% PID gains
Kp = 1;
Ki = 0.1;
Kd = 0.1;
Tf = 0; % no noise filter, extra dimension in ss
%Ts = 0.1;

% PID transfer function
C = pid(Kp, Ki, Kd, Tf);

% System
num = [4];
den = [1 2 10];
sys = tf(num, den);

% Feedback
controlloop = feedback(C*sys,1);

% Run response to initial condition
% t = 0:0.005:50;
dt = 0.005;
sys_ss = ss_conversion(controlloop);
[y,t,x] = initial(sys_ss, x0, [0 dt]);  % Initial conditions: [state (error), integral (int of error), derivative (der of error)]

plot(t,y)

% ss conversion
function sys_ss = ss_conversion(cl) % cl: cntrolloop
    cl_num = cell2mat(cl.Numerator);
    cl_den = cell2mat(cl.Denominator);
    [A,B,C,D] = tf2ss(cl_num, cl_den);
    sys_ss = ss(A,B,C,D);
end