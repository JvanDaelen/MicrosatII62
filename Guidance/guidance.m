function [desired_relative_state,desired_acceleration,mode] = guidance(relative_state_chaser,station_keeping_points,n)

% The guidance function uses the CW equations to first compute the nominal 
% trajectory of the chaser from start and until the next SK point in which 
% the S/C is heading to. Then it checks in which approximation phase is the
% S/C in the current position and computes the relative velocity and 
% acceleration required to reach the next SK point.
%
% Inputs:
%   relative_state_chaser: relative position and velocity of the chaser 
%                          w.r.t. the target 
%                          [x;y;z;u;v;w] [m;m;m;m/s;m/s;m/s]
%   station_keeping_points: relative position of targeted SK points w.r.t. 
%                           the target [m]
%                           [x1,y1,z1;
%                           x2,y2,z2;
%                           ...]
%   n: mean motion of the target's orbit [rad/s]
%
% Outputs:
%   desired_relative_state: relative position and desired relative velocity of the chaser 
%                           w.r.t. the target required to reach the next SK point [m] [m/s]
%                           [x; y; z; u; v; w]
%   desired_acceleration: relative constant acceleration of the chaser w.r.t. the
%                         target. Required during altitude raise and zero during 
%                         closing phase. [m/s2]
%                         [ax; ay; az]
%   mode: approximation mode, string variable
%         Values are 'Homing','Closing' or 'Final'


% Initial position of the S/C
x0 = relative_state_chaser(1);
y0 = relative_state_chaser(2);
z0 = relative_state_chaser(3);
pos_0 = [x0,y0,z0];

%% COMPUTATION OF THE NOMINAL TRAJECTORY (up to the next SK point)

vel_ref = [0,0,0]; % Initial relative reference velocity [m/s]
c = 1; % Counter

pos_ref = station_keeping_points(1,:); % Initial position, SK0 [m]
pos_t = station_keeping_points(2:end,:); % Target SK positions [m]

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
if y0 >= pos_t(c,2)
    c = c+1;
    % Between SK1 and SK2
    [x_ref1,y_ref1,dx_ref1,dy_ref1] = closingNominalTrajectory(n,[x_ref(end),y_ref(end),0],vel_ref,pos_t(c,:));
    x_ref = [x_ref;x_ref1];
    y_ref = [y_ref;y_ref1];
    dx_ref = [dx_ref;dx_ref1];
    dy_ref = [dy_ref;dy_ref1];
    if y0 >= pos_t(c,2)
        c = c+1;
        % Between SK2 and SK3
        [x_ref2,y_ref2,dx_ref2,dy_ref2] = closingNominalTrajectory(n,[x_ref(end),y_ref(end),0],[dx_ref(end),dy_ref(end),0],pos_t(c,:));
        x_ref = [x_ref;x_ref2];
        y_ref = [y_ref;y_ref2];
        dx_ref = [dx_ref;dx_ref2];
        dy_ref = [dy_ref;dy_ref2];
        if y0 >= pos_t(c,2)
            c = c+1;
            % Between SK3 and SK4
            [x_ref3,y_ref3,dx_ref3,dy_ref3] = closingNominalTrajectory(n,[x_ref(end),y_ref(end),0],[dx_ref(end),dy_ref(end),0],pos_t(c,:));
            x_ref = [x_ref;x_ref3];
            y_ref = [y_ref;y_ref3];
            dx_ref = [dx_ref;dx_ref3];
            dy_ref = [dy_ref;dy_ref3];
            if y0 >= pos_t(c,2)
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
    desired_relative_state = [x(1),y(1),z(1),dx(1),dy(1),dz(1)];
    mode = 'Homing';
elseif (y0 >= pos_t(1,2)) && (y0 < pos_t(5,2)) % Closing phase
    [x,y,z,dx,dy,dz] = closingTrajectory(n,pos_0,dx_ref,dy_ref,pos_t,c);
    desired_relative_state = [x(1),y(1),z(1),dx(1),dy(1),dz(1)];
    fy = 0;
    mode = 'Closing';
else
    mode = 'Final';
    desired_relative_state = [pos_t(end,1),pos_t(end,2),pos_t(end,3),0,0,0];
end


% Output variables
% desired_relative_state = [x(1),y(1),z(1),dx(1),dy(1),dz(1)];
desired_acceleration = [0;fy;0];

end