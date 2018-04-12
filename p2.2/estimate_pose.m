function [H] = estimate_pose(sensor, K, XYZ, Yaw, tagIDs)
%ESTIMATE_POSE 6DOF pose estimator based on apriltags
%   sensor - struct stored in provided dataset, fields include
%          - is_ready: logical, indicates whether sensor data is valid
%          - rpy, omg, acc: imu readings, you should not use these in this phase
%          - img: uint8, 240x376 grayscale image
%          - id: 1xn ids of detected tags
%          - p0, p1, p2, p3, p4: 2xn pixel position of center and
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

ids = sensor.id;
[~,numIds] = size(ids);


isReady = sensor.is_ready;

if (numIds == 0)
    pos = [];%zeros(3,1);
    q = [];%[1; zeros(3,1)];
    
elseif (isReady)    
    %all the world coords 
    A = []; 
    b = [];
    
    %get the world and camera coordinates for each of the april tags 
    for i = 1:numIds
        id = ids(i);
        
        %get the homogeneous coordiantes for the particular corner
        p1 = [sensor.p1(1:2,i);1]; 
        p2 = [sensor.p2(1:2,i);1]; 
        p3 = [sensor.p3(1:2,i);1];
        p4 = [sensor.p4(1:2,i);1];
        
        %get the world coordinate of the top left corner 
        worldP4 = [getWorldCoords(id,tagIDs); 1]; 
        worldP3 = worldP4 + [0;.152;0];
        worldP2 = worldP4 + [.152;.152;0];
        worldP1 = worldP4 + [.152;0;0];
        
       %get ax and ay 
       [ax1, ay1] = getAxAy(worldP1,p1);
       [ax2, ay2] = getAxAy(worldP2,p2);
       [ax3, ay3] = getAxAy(worldP3,p3);
       [ax4, ay4] = getAxAy(worldP4,p4);
           
        %Make the A vector 
        tempA = [ax1;ay1;ax2;ay2;ax3;ay3;ax4;ay4];
        A = [A;tempA];
    end 
    

    %solve A using SVD to get to the H 
    [U,S,V]=svd(A);
    H=reshape(V(:,end),[3,3]);
    H=H/V(9,9);
    H=H';
    
%     %convert to camera coordinates 
%     HPrime = K\H;
%    
%     %get the two first 
%     h1 = HPrime(:,1);
%     h2 = HPrime(:,2);
%     
%     %find the translation vector t 
%     t = HPrime(:,3)/norm(h1); 
%     
%     %find the R matrix 
%     [U1,~,V1] = svd([h1 h2 cross(h1,h2)]);
%     R1 = eye(3);
%     R1(3,3) = det(U1*V1');
%     R = U1*R1*V1';
%     
%     %find the H matrix for the camera 
%     angle = Yaw;
%     cameraToRobotR = [sin(angle) -sin(angle) 0; -sin(angle) -sin(angle) 0; 0 0 -1];
%     HCamera = [[R, t;];[zeros(1,3),1]];
         
%     %get the rotation matrix from the camera to the robot 
%     HCameraToRobot = [[cameraToRobotR, -cameraToRobotR*XYZ';];[zeros(1,3),1]];
%     
%     %convert the R matrix from the camera to world TO robot to world 
%     finalR =  HCameraToRobot*HCamera; 
%     finalR = inv(finalR); 
%     R = finalR(1:3,1:3);
%     pos = finalR(1:3,4); 

end 

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

function [ax,ay] = getAxAy(x1Vec, x2Vec)
x1 = x1Vec(1);
y1 = x1Vec(2);
x2 = x2Vec(1); 
y2 = x2Vec(2); 
ax = [-x1, -y1, -1,0,0,0,x2*x1,x2*y1,x2];
ay = [0,0,0,-x1,-y1,-1, y2*x1,y2*y1,y2];

end 
