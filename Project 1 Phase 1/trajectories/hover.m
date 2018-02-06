function [desired_state] = hover(t, qn)
% HOVER creates hover

% =================== Your code goes here ===================
% You have to set the pos, vel, acc, yaw and yawdot variables
%    
% NOTE: the simulator will spawn the robot to be at the
%       position you return for t == 0

height = 0:0.2:5;
m = length(height);

pos = [zeros(1,m);zeros(1,m);height];
vel = [zeros(1,m); zeros(1,m); ones(1,m)];
acc = [zeros(1,m); zeros(1,m); zeros(1,m)];
yaw = 0;
yawdot = 0;

% =================== Your code ends here ===================

desired_state.pos = pos;
desired_state.vel = vel;
desired_state.acc = acc;
desired_state.yaw = yaw;
desired_state.yawdot = yawdot;

end
