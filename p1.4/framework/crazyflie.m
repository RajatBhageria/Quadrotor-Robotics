function [ params ] = crazyflie(  )
% crazyflie: physical parameters for the Crazyflie 2.0
%
% 2016 Bernd Pfrommer
%
% This function creates a struct with the basic parameters for the
% Crazyflie 2.0 quad rotor (without camera, but with about 5 vicon
% markers)
%
% Model assumptions based on physical measurements:
%
% motor + mount + vicon marker = mass point of 3g
% arm length of mass point: 0.046m from center
% battery pack + main board are combined into cuboid (mass 18g) of
% dimensions:
%
%   width  = 0.03m
%   depth  = 0.03m
%   height = 0.012m
%

m = 0.030;  % weight (in kg) with 5 vicon markers (each is about 0.25g)
g = 9.81;   % gravitational constant
I = [1.43e-5,   0,          0; % inertial tensor in m^2 kg
     0,         1.43e-5,    0;
     0,         0,          2.89e-5];
L = 0.046; % arm length in m

Ixx = I(1,1);
Iyy = I(2,2);
Izz = I(3,3);

params.mass = m;
params.I    = I;
params.invI = inv(I);
params.grav = g;
params.arm_length = L;

params.maxangle = 40*pi/180; % you can specify the maximum commanded angle here
params.maxF     = 1.7*m*g;   % left these untouched from the nano plus
params.minF     = 0.05*m*g;  % left these untouched from the nano plus

% You can add any fields you want in params
% for example you can add your controller gains by
% params.k = 0, and they will be passed into controller.m
params.kp_z = 50 * 30/200;
params.kd_z = 10 * 30/200;

params.kp_xy = 5;
params.kd_xy = 2.5;

% t_attitude  = 0.15; %rise time of the attitude controller
% xi_attitude = 1; %damping ratio for attitude controller

% params.kp_roll = 3.24*Ixx/t_attitude^2;
% params.kd_roll = 3.6*xi_attitude*Ixx/t_attitude;
% 
% params.kp_pitch = 3.24*Iyy/t_attitude^2;
% params.kd_pitch = 3.6*xi_attitude*Ixx/t_attitude;

t_yaw  = 0.2; %rime time of yaw controller
xi_yaw = 1.0; %damping ratio for yaw controller
params.kp_yaw = 3.24*Izz/t_yaw^2;
params.kd_yaw = 3.6*xi_yaw*Izz/t_yaw;


params.ki_xy = .03; %integral gains
params.ki_z  = .02;

end

