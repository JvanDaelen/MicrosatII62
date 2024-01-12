function [x,y,z,dx,dy,dz] = homingTrajectory(n,pos_0,dx_ref,dy_ref,pos_t,c,fy)

% The homingTrajectory function uses the CW equations to recompute the
% homing trajectory (position and velocity at each point) given that the
% initial relative position is slightly deviated from the nominal
% trajectory.
%
% Inputs:
%   n: mean motion of the target's orbit [rad/s]
%   pos_0: current relative position of the chaser w.r.t. the target [m]
%   dx_ref: relative X velocity of the chaser along the nominal trajectory [m]
%   dy_ref: relative Y velocity of the chaser along the nominal trajectory [m]
%   pos_t: relative position of targeted SK points w.r.t. the target [m]
%   c: counter
%   fy: constant Y acceleration [m/s2]
%
% Outputs:
%   x: relative X position of the chaser along the recomputed trajectory [m]
%   y: relative Y position of the chaser along the recomputed trajectory [m]
%   z: relative Z position of the chaser along the recomputed trajectory [m]
%   dx: relative X velocity of the chaser along the recomputed trajectory [m]
%   dy: relative Y velocity of the chaser along the recomputed trajectory [m]
%   dz: relative Z velocity of the chaser along the recomputed trajectory [m]

% First solve for dx0, dy0, and t_end given [x_tar,y_tar] and [x0,y0]
% Then compute [x,y] for several t and plot it in an xy graph
% Do the plots for different values of [x0,y0]

fx = 0; fz = 0; % Unperturbed relative motion [N]

% Initial position of the S/C
x0 = pos_0(1);
y0 = pos_0(2);
z0 = pos_0(3);

% Position of the targeted SK point
x_t = pos_t(c,1);
y_t = pos_t(c,2);
z_t = pos_t(c,3);

% Solve equations symbolically
syms dx0 dy0 dz0 t_end
dx_t = dx_ref(end);
dy_t = dy_ref(end);

eq1 = x_t == x0*(4-3*cos(n*t_end)) + dx0/n*sin(n*t_end) + 2*dy0/n*(1-cos(n*t_end)) + fx/n^2*(1-cos(n*t_end)) + 2*fy^n^2*(n*t_end-sin(n*t_end));
eq2 = y_t == y0 - dy0/n*(3*n*t_end-4*sin(n*t_end)) - 6*x0*(n*t_end-sin(n*t_end)) - 2*dx0/n*(1-cos(n*t_end)) - 2*fx/n^2*(n*t_end-sin(n*t_end)) + 2*fy/n^2*(2-3/4*n^2*t_end^2-2*cos(n*t_end));
eq3 = dx_t == 3*n*x0*sin(n*t_end) + dx0*cos(n*t_end) + 2*dy0*sin(n*t_end) + fx/n*sin(n*t_end) + 2*fy/n*(1-cos(n*t_end));
eq4 = dy_t == -dy0*(3-4*cos(n*t_end)) - 6*n*x0*(1-cos(n*t_end)) - 2*dx0*sin(n*t_end) - 2*fx/n*(1-cos(n*t_end)) + 2*fy/n*(-3/2*n*t_end+2*sin(n*t_end));
eq5 = z_t == z0*cos(n*t_end) + dz0/n*sin(n*t_end) + fz/n*(1-cos(n*t_end));

S = vpasolve([eq1,eq2,eq3,eq5],[dx0,dy0,dz0,t_end]);
if S.t_end < 0
    S = vpasolve([eq1,eq2,eq4,eq5],[dx0,dy0,dz0,t_end]);
end

dx0 = double(S.dx0);
dy0 = double(S.dy0);
dz0 = double(S.dz0);
t_end = double(S.t_end);

% Propagate trajectory in time
time = 0:50:t_end;
x = zeros(length(time),1); dx = zeros(length(time),1);
y = zeros(length(time),1); dy = zeros(length(time),1);
z = zeros(length(time),1); dz = zeros(length(time),1);

for i = 1:length(time)
    t = time(i);
    x(i) = x0*(4-3*cos(n*t)) + dx0/n*sin(n*t) + 2*dy0/n*(1-cos(n*t)) + fx/n^2*(1-cos(n*t)) + 2*fy^n^2*(n*t-sin(n*t));
    y(i) = y0 - dy0/n*(3*n*t-4*sin(n*t)) - 6*x0*(n*t-sin(n*t)) - 2*dx0/n*(1-cos(n*t)) - 2*fx/n^2*(n*t-sin(n*t)) + 2*fy/n^2*(2-3/4*n^2*t^2-2*cos(n*t));
    z(i) = z0*cos(n*t) + dz0/n*sin(n*t) + fz/n*(1-cos(n*t));
    
    dx(i) = 3*n*x0*sin(n*t) + dx0*cos(n*t) + 2*dy0*sin(n*t) + fx/n*sin(n*t) + 2*fy/n*(1-cos(n*t));
    dy(i) = -dy0*(3-4*cos(n*t)) - 6*n*x0*(1-cos(n*t)) - 2*dx0*sin(n*t) - 2*fx/n*(1-cos(n*t)) + 2*fy/n*(-3/2*n*t+2*sin(n*t));
    dz(i) = -z0*n*sin(n*t) + dz0*cos(n*t) + fz/n*sin(n*t);
end

end