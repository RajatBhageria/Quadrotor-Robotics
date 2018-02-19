function [desired_state] = hover(t, qn)
% HOVER creates hover

% =================== Your code goes here ===================
% You have to set the pos, vel, acc, yaw and yawdot variables
%    
% NOTE: the simulator will spawn the robot to be at the
%       position you return for t == 0

if t<5
    pos = [0;t;0];
    vel = [0;1;0];
    acc = [0;0;0];
else 
    pos = [0;5;0];
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
