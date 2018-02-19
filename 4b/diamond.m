function [desired_state] = diamond(t, qn)
% DIAMOND trajectory generator for a diamond

% =================== Your code goes here ===================
% You have to set the pos, vel, acc, yaw and yawdot variables
% NOTE: the simulator will spawn the robot to be at the
%       position you return for t == 0
 
maxT = 14; 

%cs for the trajectory smoothing 
cs = [0;0;0.2449;-0.0466];

%vectors for each of the segments 
v1 = [1/4; sqrt(2); sqrt(2)];
v2 = [1/4; -sqrt(2); sqrt(2)];
v3 = [1/4; -sqrt(2); -sqrt(2)];
v4 = [1/4; sqrt(2); -sqrt(2)];

%starting point
if (t==0)
    pos=[0;0;0];
    vel=[0;0;0];
    acc=[0;0;0];
%first leg
elseif (t<.25*maxT)
    pos=v1*theta(t,cs)+[0;0;0];
    vel=v1*omega(t,cs); 
    acc=v1*alpha(t,cs);
%second leg
elseif (t<.50*maxT) && (t >= .25*maxT)
    pos=v2*theta(t-.25*maxT,cs)+[1/4;sqrt(2);sqrt(2)];
    vel=v2*omega(t-.25*maxT,cs); 
    acc=v2*alpha(t-.25*maxT,cs);
%third leg 
elseif (t<.75*maxT) && (t >= .50*maxT)
    pos=v3*theta(t-.50*maxT,cs)+[1/2;0;2*sqrt(2)];
    vel=v3*omega(t-.50*maxT,cs); 
    acc=v3*alpha(t-.50*maxT,cs);
%fourth leg
elseif (t<maxT) && (t >= .75*maxT)
    pos=v4*theta(t-.75*maxT,cs)+[3/4;-1*sqrt(2);sqrt(2)];
    vel=v4*omega(t-.75*maxT,cs); 
    acc=v4*alpha(t-.75*maxT,cs);
else
    pos=[1;0;0];
    vel=[0;0;0]; 
    acc=[0;0;0];
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

function [val] = theta(t,cs)
val = dot(cs, [1, t, t^2, t^3]'); 
end

function [val] = omega(t,cs)
val = dot(cs, [0, 1, 2*t, 3*t^2]'); 
end

function [val] = alpha(t,cs)
val = dot(cs, [0, 0, 2, 6*t]'); 
end 
