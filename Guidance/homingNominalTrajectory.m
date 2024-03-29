function [x_ref,y_ref,dx_ref,dy_ref,fy] = homingNominalTrajectory(n,pos_ref,vel_ref,pos_t)

% The homingNominalTrajectory function uses the CW equations to compute the
% homing nominal (reference) trajectory (position and velocity at each 
% point). This function is necessary to compute the constant Y acceleration 
% (fy) that will be needed during the altitude raise and the velocity that 
% the chaser should have when the SK1 point is reached.
%
% Inputs:
%   n: mean motion of the target's orbit [rad/s]
%   pos_ref: relative position of SK0 w.r.t. the target [m]
%   vel_ref: initial relative velocity of the chaser w.r.t. the target [m/s]
%   pos_t: relative position of targeted SK1 point w.r.t. the target [m]
%
% Outputs:
%   x_ref: relative X positions of the chaser along the nominal trajectory [m]
%   y_ref: relative Y positions of the chaser along the nominal trajectory [m]
%   dx_ref: relative X velocity of the chaser along the nominal trajectory [m]
%   dy_ref: relative Y velocity of the chaser along the nominal trajectory [m]
%   fy: constant Y acceleration [m/s2]

% Variables (unknowns): 
% initial Y relative velocity (dy0) and constant acceleration in Y (fy)
syms dy0 fy

% Initial conditions
x0 = pos_ref(1);
y0 = pos_ref(2);
dx0 = vel_ref(1);
fx = 0;
t_end = 2*pi/n;
x_t = pos_t(1);
y_t = pos_t(2);

% Equations
eq1 = x_t == x0*(4-3*cos(n*t_end)) + dx0/n*sin(n*t_end) + 2*dy0/n*(1-cos(n*t_end)) + fx/n^2*(1-cos(n*t_end)) + 2*fy/n^2*(n*t_end-sin(n*t_end));
eq2 = y_t == y0 - dy0/n*(3*n*t_end-4*sin(n*t_end)) - 6*x0*(n*t_end-sin(n*t_end)) - 2*dx0/n*(1-cos(n*t_end)) - 2*fx/n^2*(n*t_end-sin(n*t_end)) + 2*fy/n^2*(2-3/4*n^2*t_end^2-2*cos(n*t_end));

% Resolution of the system of equations
S = vpasolve([eq1,eq2],[dy0,fy]);
dy0 = double(S.dy0);
fy = double(S.fy);

% Trajectory computation
time = 0:50:t_end;
x_ref = zeros(length(time),1); dx_ref = zeros(length(time),1);
y_ref = zeros(length(time),1); dy_ref = zeros(length(time),1);

for i = 1:length(time)
    t = time(i);
    x_ref(i) = x0*(4-3*cos(n*t)) + dx0/n*sin(n*t) + 2*dy0/n*(1-cos(n*t)) + fx/n^2*(1-cos(n*t)) + 2*fy/n^2*(n*t-sin(n*t));
    y_ref(i) = y0 - dy0/n*(3*n*t-4*sin(n*t)) - 6*x0*(n*t-sin(n*t)) - 2*dx0/n*(1-cos(n*t)) - 2*fx/n^2*(n*t-sin(n*t)) + 2*fy/n^2*(2-3/4*n^2*t^2-2*cos(n*t));
    
    dx_ref(i) = 3*n*x0*sin(n*t) + dx0*cos(n*t) + 2*dy0*sin(n*t) + fx/n*sin(n*t) + 2*fy/n*(1-cos(n*t));
    dy_ref(i) = -dy0*(3-4*cos(n*t)) - 6*n*x0*(1-cos(n*t)) - 2*dx0*sin(n*t) - 2*fx/n*(1-cos(n*t)) + 2*fy/n*(-3/2*n*t+2*sin(n*t));
end
end