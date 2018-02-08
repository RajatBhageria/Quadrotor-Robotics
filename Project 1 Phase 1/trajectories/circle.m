function [desired_state] = circle(t, qn)
% CIRCLE trajectory generator for a circle

% =================== Your code goes here ===================
% You have to set the pos, vel, acc, yaw and yawdot variables
% NOTE: the simulator will spawn the robot to be at the
%       position you return for t == 0

r = 5;            % radius
h = 2.5;            % height
x = r * cos(t);
y = r * sin(t);
z = h/(2*pi)*t;   

if t == 0
    pos = [5;0;0];
    vel = [0;0;0];
    acc = [0;0;0];
elseif t < 2*pi
    pos = [x;y;z];
    vel = [-r*sin(t); r*cos(t); h/(2*pi)];
    acc = [-r*cos(t); -r*sin(t); 0];
else
    pos = [r*cos(2*pi);r*sin(2*pi);2.5];
    vel = [0;0;0];
    acc = [0;0;0];  
end

yaw = 0;
yawdot = 0;

% =================== Your code ends here ===================

desired_state.pos = pos(:);
desired_state.vel = vel(:);
desired_state.acc = acc(:);
desired_state.yaw = yaw;
desired_state.yawdot = yawdot;



end
