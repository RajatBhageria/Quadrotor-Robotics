% Add additional inputs after sensor if you want to
% Example:
% your_input = 1;
% estimate_vel_handle = @(sensor) estimate_vel(sensor, your_input);
%
% We will only call estimate_vel_handle in the test function.
% Note that thise will only create a function handle, but not run the function


% Camera Matrix (zero-indexed):
K = [311.0520 0        201.8724;
 0         311.3885 113.6210;
 0         0        1];

% Camera-IMU Calibration (see attached images for details):
XYZ = [-0.04, 0.0, -0.03];
Yaw = pi/4;

% Tag ids:
tagIDs = [0, 12, 24, 36, 48, 60, 72, 84,  96;
 1, 13, 25, 37, 49, 61, 73, 85,  97;
 2, 14, 26, 38, 50, 62, 74, 86,  98;
 3, 15, 27, 39, 51, 63, 75, 87,  99;
 4, 16, 28, 40, 52, 64, 76, 88, 100;
 5, 17, 29, 41, 53, 65, 77, 89, 101;
 6, 18, 30, 42, 54, 66, 78, 90, 102;
 7, 19, 31, 43, 55, 67, 79, 91, 103;
 8, 20, 32, 44, 56, 68, 80, 92, 104;
 9, 21, 33, 45, 57, 69, 81, 93, 105;
10, 22, 34, 46, 58, 70, 82, 94, 106;
11, 23, 35, 47, 59, 71, 83, 95, 107];

estimate_vel_handle = @(sensor) estimate_vel(sensor,K,XYZ, Yaw, tagIDs);
