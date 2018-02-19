function [desired_state] = circle(t, qn)
% CIRCLE trajectory generator for a circle

% =================== Your code goes here ===================
% You have to set the pos, vel, acc, yaw and yawdot variables
% NOTE: the simulator will spawn the robot to be at the
%       position you return for t == 0

r = 5;            % radius
h = 2.5;            % height
maxT = 14;
cs = [0; 0.0000; 0.0962; -0.0046]';

if t==0
    pos = [5;0;0];
    vel = [0;0;0];
    acc = [0;0;0];
    
elseif t < maxT
    theta = dot(cs, [1, t, t^2, t^3]'); 
    omega = dot(cs, [0, 1, 2*t, 3*t^2]'); 
    alpha = dot(cs, [0, 0, 2, 6*t]');  

    pos = [r * cos(theta); r * sin(theta); h*theta/(2*pi)];
    vel = [-r*sin(theta)*omega; r*cos(theta)*omega; h*omega/(2*pi)];
    acc = [-r*sin(theta)*alpha-omega*r*cos(theta);r*cos(theta)*alpha-omega*r*sin(theta);h*alpha/(2*pi)];

else
    pos = [5;0;2.5];
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
