% PID controller

% Input function

% PID variables
Kp = 1;
Ki = 1;
Kd = 0;
Tf = 100;
%Ts = 0.1;

% PID function
C = pid(Kp, Ki, Kd, Tf);

% System
num = [4];
den = [1 2 10];
sys = tf(num, den);

% Feedback
controlloop = feedback(C*sys,1);
cl_num = cell2mat(controlloop.Numerator);
cl_den = cell2mat(controlloop.Denominator);
[A,B,C,D] = tf2ss(cl_num, cl_den);
sys_ss = ss(A,B,C,D);
impulse(controlloop)

% Run response to initial condition
t = 0:0.005:50;
[y,t,x] = initial(sys_ss, [1,1,0], t); % Initial conditions: [state (error), integral (int of error), derivative (der of error)]

plot(t,y)
