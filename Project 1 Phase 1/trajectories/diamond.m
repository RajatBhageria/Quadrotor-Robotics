function [desired_state] = diamond(t, qn)
% DIAMOND trajectory generator for a diamond

% =================== Your code goes here ===================
% You have to set the pos, vel, acc, yaw and yawdot variables
% NOTE: the simulator will spawn the robot to be at the
%       position you return for t == 0

maxT = 20; 

%X(t) from 0 to 1 
[posx, velx, accelx] = trajectory([0; 0.0000; 0.0106;-0.0004]); 

pos = []; 
vel = []; 
acc = [];

if t==0
    pos = [0; 0; 0];
    vel = [0; 0; 0];
    acc = [0; 0; 0];
    
elseif (t > maxT/4) && (t <= maxT/2)
    cs = [0; 0.0000; 0.1697; -0.0226];
    [posyz, velyz, accelyz] = trajectory(cs);  
    pos = [posx; posyz; posyz];
    vel = [velx; velyz; velyz];
    acc = [accelx; accelyz; accelyz];
    
elseif (t > maxT/2) && (t <= 3*maxT/4)
    [posy, vely, accely] = trajectory([1.4142; -0.0000; -0.1697; 0.0226]); %for y
    [posz, velz, accelz] = trajectory(sqrt(2),2*sqrt(2),maxT/4,t); %for z
    pos = [posx; posy; posz];
    vel = [velx; vely; velz];
    acc = [accelx; accely; accelz];
    
elseif (t > 3*maxT/4) && (t < maxT)
    [posy, vely, accely] = trajectory(0,-1*sqrt(2),maxT/4,t); %for y
    [posz, velz, accelz] = trajectory(2*sqrt(2),sqrt(2),maxT/4,t); %for z
    pos = [posx; posy; posz];
    vel = [velx; vely; velz];
    acc = [accelx; accely; accelz];
  
elseif t==maxT
    pos = [1; -1*sqrt(2); sqrt(2)];
    vel = [0; 0; 0];
    acc = [0; 0; 0]; 
    
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

function [pos, vel, accel] = trajectory(cs)
    cs = [1, 0, 0, 0; 1, T, T^2, T^3;0, 1, 0, 0; 0, 1, 2*T, 3*T^2]\[a, b, 0, 0]';
    pos = dot(cs, [1, t, t^2, t^3]'); 
    vel = dot(cs, [0, 1, 2*t^2, 3*t^2]'); 
    accel = dot(cs, [0, 0, 4*t, 6*t]');  
end