function [pos, q] = estimate_pose_test(sensor, varargin)
%ESTIMATE_POSE 6DOF pose estimator based on apriltags
%   sensor - struct stored in provided dataset, fields include
%          - is_ready: logical, indicates whether sensor data is valid
%          - rpy, omg, acc: imu readings, you should not use these in this phase
%          - img: uint8, 240x376 grayscale image
%          - id: 1xn ids of detected tags
%          - p0, pi1, p2, p3, p4: 2xn pixel position of center and
%                                four corners of detected tags
%            Y
%            ^ P3 == P2
%            | || P0 ||
%            | P4 == P1
%            o---------> X
%   varargin - any variables you wish to pass into the function, could be
%              a data structure to represent the map or camera parameters,
%              your decision. But for the purpose of testing, since we don't
%              know what inputs you will use, you have to specify them in
%              init_script by doing
%              estimate_pose_handle = ...
%                  @(sensor) estimate_pose(sensor, your personal input arguments);
%   pos - 3x1 position of the quadrotor in world frame
%   q   - 4x1 quaternion of the quadrotor [w, x, y, z] where q = w + x*i + y*j + z*k

%Get the raw parameters 
K = varargin{1};
XYZ = varargin{2}; 
Yaw = varargin{3};
tagIDs = varargin{4};

isReady = sensor.is_ready;
if (isReady)
    ids = sensor.id;
    [~,numIds] = size(ids);
    %all the camera coords 
    B = [];
    %all the world coords 
    A = []; 
    %get the world and camera coordinates for each of the april tags 
    for i = 1:numIds
        id = ids(i);
        %get the homogeneous coordiantes for the particular corner
        p1 = [sensor.p1(1:2,i);1]; 
        p2 = [sensor.p2(1:2,i);1]; 
        p3 = [sensor.p3(1:2,i);1];
        p4 = [sensor.p4(1:2,i);1];
        
        %convert the corners of the April tag to camera coordinates 
        cameraP1 = getCameraCoords(K,p1);
        cameraP2 = getCameraCoords(K,p2);
        cameraP3 = getCameraCoords(K,p3);
        cameraP4 = getCameraCoords(K,p4); 
        Btemp = [cameraP1, cameraP2, cameraP3, cameraP4];
        
        %get the world coordinate of the top left corner 
        worldP4 = [getWorldCoords(id,tagIDs); 1]; 
        worldP3 = worldP4 + [0;.152;0];
        worldP2 = worldP4 + [.152;.152;0];
        worldP1 = worldP4 + [.152;0;0];
        Atemp = [worldP1, worldP2, worldP3, worldP4];
        
        A = [A,Atemp];
        B = [B,Btemp];
    end 

    %get the centroid of A and B matrices
    ABar = mean(A,2); 
    BBar = mean(B,2); 
    
    %subtract the centroid from both A and B matrices 
    A = A - ABar; 
    B = B - BBar; 
    
    %find the R matrix using SVD 
    [U,~,V] = svd(B*A');
    
    %find the R matrix from the camera to the world 
    R = eye(3); 
    R(3,3) = det(U*V'); 
    R = U*R*V';
    
    %find the translation vector t 
    t = ABar - R*BBar; 
    
    %find the H matrix for the camera 
    HCamera = [[R, t;];[zeros(1,3),1]];
    
    %get the rotation matrix from the camera to the robot 
    angle = Yaw;
    cameraToRobotR = [sin(angle) -sin(angle) 0; -sin(angle) -sin(angle) 0; 0 0 -1];
    HCameraToRobot = [[cameraToRobotR, -cameraToRobotR*XYZ';];[zeros(1,3),1]];
    
    %convert the R matrix from the camera to world TO robot to world 
    finalR =  HCamera*HCameraToRobot; 
    RRobot = finalR(1:3,1:3);
    pos = finalR(1:3,4); 
    
    %convert the R matrix to quaternion 
    %find the axis angle representation 
    theta = real(acos((trace(RRobot)-1)/2.0));
    S = .5*(RRobot-RRobot');
    r = [S(3,2),S(3,1),S(2,1)];
    
    %convert the axis angle representaion to the quaternion 
    w = cos(theta/2);
    qv = sin(theta/2)*r; 
    q= [w,qv]';
        
end 

end

function [cameraCoords] = getCameraCoords(K,p)
    cameraCoords = inv(K)*p; 
end 

%returns the top left coordinate 
function coords = getWorldCoords(id,allIds)
    [i, j] = find(allIds == id); 
    
    %get the x coordiante of top left coordinate 
    x = 0.152*2*(i-1);
    
    %get the y coordiante of the top left coordinate 
    y = 0.152*2*(j-1) + floor((j-1)/3)*(.178-.152); 
   
    coords = [x;y];
    
end 

% function R = rotz(alpha)
% 
% R = [cos(alpha) -sin(alpha) 0; ...
%      sin(alpha)  cos(alpha) 0; ...
%               0           0 1];
% end