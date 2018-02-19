function [F, M, trpy, drpy] = controller(qd, t, qn, params)
% CONTROLLER quadrotor controller
% The current states are:
%% Inputs:
%
% qd{qn}: state and desired state information for quadrotor #qn (qn
%         will be = 1 since we are only flying a single robot)
%
%  qd{qn}.pos, qd{qn}.vel   position and velocity
%  qd{qn}.euler = [roll;pitch;yaw]
%  qd{qn}.omega     angular velocity in body frame
% 
%  qd{qn}.pos_des, qd{qn}.vel_des, qd{qn}.acc_des  desired position, velocity, accel
%  qd{qn}.yaw_des, qd{qn}.yawdot_des
%
% t: current time
%    
% qn: quadrotor number, should always be 1
%    
% params: various parameters
%  params.I     moment of inertia
%  params.grav  gravitational constant g (9.8...m/s^2)
%  params.mass  mass of robot
%
%% Outputs:
%
% F: total thrust commanded (sum of forces from all rotors)
% M: total torque commanded
% trpy: thrust, roll, pitch, yaw (attitude you want to command!)
% drpy: time derivative of trpy
%
% Using these current and desired states, you have to compute the desired
% controls u, and from there F and M
%

% =================== Your code goes here ===================
% ...
% ==============================

% Desired roll, pitch and yaw (in rad). In the simulator, those will be *ignored*.
% When you are flying in the lab, they *will* be used (because the platform
% has a built-in attitude controller). Best to fill them in already
% during simulation.

psi_des   = 0;

%define all the gains
kdi = [5;5;5]; 
kpi = [3;3;3];
kpPhi = 350; 
kdPhi = 15; 
kpTheta = 350; 
kdTheta = 15; 
kpPsi = 40; 
kdPsi = 60; 


%Compute the commanded accelerations  (equation 21) 
acc = qd{qn}.acc_des - kdi.*(qd{qn}.vel - qd{qn}.vel_des) - kpi.*(qd{qn}.pos - qd{qn}.pos_des);

%find the current orientation 
phi = qd{qn}.euler(1); 
theta = qd{qn}.euler(2); 
psi = qd{qn}.euler(3); 

%find the theta_des and phi_des
theta_des = (acc(1)*cos(psi) + acc(2)*sin(psi))/(params.grav*(cos(psi)^2 + sin(psi)^2));
phi_des = -(acc(2)*cos(psi) - acc(1)*sin(psi))/(params.grav*(cos(psi)^2 + sin(psi)^2));

%find u1 
u1 = params.mass*(params.grav + acc(3));

%find p,q,r parts of angular velocity 
p = qd{qn}.omega(1); 
q = qd{qn}.omega(2); 
r = qd{qn}.omega(3); 

%find u2
u2 = params.I*[-kpPhi*(phi-phi_des) - kdPhi*(p-0);
               -kpTheta*(theta-theta_des) - kdTheta*(q-0);
               -kpPsi*(psi-psi_des) - kdPsi*(r-qd{qn}.yawdot_des);];

%define the u vector
u    = [u1;u2]; % control input u, you should fill this in
                  
% Thrust
F    = u(1);       % This should be F = u(1) from the project handout

% Moment
M    = u(2:4);     % note: params.I has the moment of inertia

% =================== Your code ends here ===================

% Output trpy and drpy as in hardware
trpy = [F, phi_des, theta_des, psi_des];
drpy = trpy/t;

end

%
% ------------------------------------------------------------
%    should you decide to write a geometric controller,
%    the following functions should come in handy
%

function m = eulzxy2rotmat(ang)
    phi   = ang(1);
    theta = ang(2);
    psi   = ang(3);
    
    m = [[cos(psi)*cos(theta) - sin(phi)*sin(psi)*sin(theta), -cos(phi)*sin(psi), ...
          cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi)];
         [cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta),  cos(phi)*cos(psi), ...
          sin(psi)*sin(theta) - cos(psi)*cos(theta)*sin(phi)];
         [-cos(phi)*sin(theta), sin(phi), cos(phi)*cos(theta)]];
end

function eul = rotmat2eulzxy(R)
    if R(3,2) < 1
        if R(3,2) > -1
            thetaX = asin(R(3,2));
            thetaZ = atan2(-R(1,2), R(2,2));
            thetaY = atan2(-R(3,1), R(3,3));
        else % R(3,2) == -1
            thetaX = -pi/2;
            thetaZ = -atan2(R(1,3),R(1,1));
            thetaY = 0;
        end
    else % R(3,2) == +1
        thetaX = pi/2;
        thetaZ = atan2(R(1,3),R(1,1));
        thetaY = 0;
    end
    eul = [thetaX, thetaY, thetaZ];
end

function w = veemap(R)
    w = [-R(2,3), R(1,3), -R(1,2)];
end
