function [desired_state] = student_control_hover(qd)
persistent desired_pos

if nargin
    desired_pos = qd{1}.pos;
    return;
end

desired_state.pos  = desired_pos;
desired_state.vel  = [0; 0; 0];
desired_state.acc  = [0; 0; 0];
desired_state.yaw    = 0;
desired_state.yawdot = 0;
end