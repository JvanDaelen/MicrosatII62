function [state,acceleration] = guidance(mu,R,h,pos_ref,pos_0,pos_t)

% The guidance function uses the CW equations to first compute the nominal 
% trajectory of the chaser from start and until the next SK point in which 
% the S/C is heading to. Then it checks in which approximation phase is the
% S/C in the current position and computes the relative velocity and 
% acceleration required to reach the next SK point.
%
% Inputs:
%   mu: Earth gravitational constant [m3/s2]
%   R: Earth mean radius [m]
%   h: Target S/C orbital altitude [m]
%   pos_ref: relative position of SK0 w.r.t. the target [m]
%   pos_0: current relative position of the chaser w.r.t. the target [m]
%   pos_t: relative position of targeted SK points w.r.t. the target [m]
%          [x1,y1,z1;
%           x2,y2,z2;
%           ...]
%
% Outputs:
%   state: relative position and desired relative velocity of the chaser 
%          w.r.t. the target required to reach the next SK point [m] [m/s]
%          [x; y; z; dx; dy; dz]
%   acceleration: relative constant acceleration of the chaser w.r.t. the
%                 target. Required during altitude raise and zero during 
%                 closing phase. [m/s2]
%                 [ax; ay; az]

a = R+h;          % Semi-major axis of the target's orbit [m]
n = sqrt(mu/a^3); % Mean motion of the target's orbit [rad/s]

% Initial position of the S/C
x0 = pos_0(1);
y0 = pos_0(2);
z0 = pos_0(3);

%% COMPUTATION OF THE NOMINAL TRAJECTORY (up to the next SK point)

vel_ref = [0,0,0]; % Initial relative velocity [m/s]
c = 1; % Counter

% Check between which SK points the chaser S/C is and compute the nominal 
% trajectory up to the next SK point. Update each time the position and
% velocity vectors for the nominal trajectory (reference). For the initial
% conditions of the next part of the trajectory take the state (position 
% and velocity) of the last point of the previous part of the trajectory.

% Between SK0 and SK1
[x_ref0,y_ref0,dx_ref0,dy_ref0,fy] = homingNominalTrajectory(n,pos_ref,vel_ref,pos_t(c,:));
x_ref = x_ref0;
y_ref = y_ref0;
dx_ref = dx_ref0;
dy_ref = dy_ref0;
if y0 > pos_t(c,2)
    c = c+1;
    % Between SK1 and SK2
    [x_ref1,y_ref1,dx_ref1,dy_ref1] = closingNominalTrajectory(n,[x_ref(end),y_ref(end),0],vel_ref,pos_t(c,:));
    x_ref = [x_ref;x_ref1];
    y_ref = [y_ref;y_ref1];
    dx_ref = [dx_ref;dx_ref1];
    dy_ref = [dy_ref;dy_ref1];
    if y0 > pos_t(c,2)
        c = c+1;
        % Between SK2 and SK3
        [x_ref2,y_ref2,dx_ref2,dy_ref2] = closingNominalTrajectory(n,[x_ref(end),y_ref(end),0],[dx_ref(end),dy_ref(end),0],pos_t(c,:));
        x_ref = [x_ref;x_ref2];
        y_ref = [y_ref;y_ref2];
        dx_ref = [dx_ref;dx_ref2];
        dy_ref = [dy_ref;dy_ref2];
        if y0 > pos_t(c,2)
            c = c+1;
            % Between SK3 and SK4
            [x_ref3,y_ref3,dx_ref3,dy_ref3] = closingNominalTrajectory(n,[x_ref(end),y_ref(end),0],[dx_ref(end),dy_ref(end),0],pos_t(c,:));
            x_ref = [x_ref;x_ref3];
            y_ref = [y_ref;y_ref3];
            dx_ref = [dx_ref;dx_ref3];
            dy_ref = [dy_ref;dy_ref3];
            if y0 > pos_t(c,2)
                c = c+1;
                % Between SK4 and SK5
                [x_ref4,y_ref4,dx_ref4,dy_ref4] = closingNominalTrajectory(n,[x_ref(end),y_ref(end),0],[dx_ref(end),dy_ref(end),0],pos_t(c,:));
                x_ref = [x_ref;x_ref4];
                y_ref = [y_ref;y_ref4];
                dx_ref = [dx_ref;dx_ref4];
                dy_ref = [dy_ref;dy_ref4];
            end
        end
    end
end

%% COMPUTATION OF THE RECALCULATED (OR ACTUAL) TRAJECTORY

% Check whether the S/C (actual position) is in a homing or closing phase 
% and compute the required relative velocity in the present position to 
% reach the next SK point.

if (y0 >= pos_ref(1,2)) && (y0 < pos_t(1,2)) % Homing phase
    [x,y,z,dx,dy,dz] = homingTrajectory(n,pos_0,dx_ref,dy_ref,pos_t,c,fy);
elseif (y0 >= pos_t(1,2)) && (y0 < pos_t(5,2)) % Closing phase
    [x,y,z,dx,dy,dz] = closingTrajectory(n,pos_0,dx_ref,dy_ref,pos_t,c);
    fy = 0;
end

% Output variables
state = [x(1),y(1),z(1),dx(1),dy(1),dz(1)];
acceleration = [0;fy;0];

end