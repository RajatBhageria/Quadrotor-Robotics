function [vel, omg] = estimate_vel(sensor, varargin)
%ESTIMATE_VEL 6DOF velocity estimator
%   sensor - struct stored in provided dataset, fields include
%          - is_ready: logical, indicates whether sensor data is valid
%          - t: timestamp
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
%              estimate_vel_handle = ...
%                  @(sensor) estimate_vel(sensor, your personal input arguments);
%   vel - 3x1 velocity of the quadrotor in world frame
%   omg - 3x1 angular velocity of the quadrotor

ids = sensor.id;
[~,numIds] = size(ids);

% %Get the raw parameters 
K = varargin{1};
XYZ = varargin{2}; 
Yaw = varargin{3};
tagIDs = varargin{4};

isReady = sensor.is_ready;

persistent points; 
persistent tracker; 
persistent oldTime;
persistent oldDeltaT; 
persistent oldImg; 

%find the number of points 
[numPts,~] = size(points); 

if (numIds == 0)
    vel = [];
    omg = [];
   
elseif (numPts == 0)
    I = sensor.img;
    pointsObj = detectFASTFeatures(I);
    pointsObj = pointsObj.selectStrongest(200);
    points = pointsObj.Location;
    oldTime = sensor.t; 
    oldDeltaT = 0;
    
    tracker = vision.PointTracker;
    initialize(tracker,points,I); 
    
    vel = [0;0;0];
    omg = [0;0;0];
   
elseif (isReady)
    I = sensor.img;
    minNumberCornersNeeded = 100;
       
    %find the optical flow between consecutive points in pixels 
    [newPts,point_validity] = tracker(I);
    
    [numPts] = sum(point_validity); 
    
    if (numPts < minNumberCornersNeeded) 
        %get all the harris corners of the image 
        release(tracker);
        pointsObj = detectFASTFeatures(oldImg);
        points = pointsObj.selectStrongest(200).Location;
        initialize(tracker,points,oldImg); 
    end 
    
    [newPts,point_validity] = tracker(I);

    %remove the unvalid points in pixels 
    oldPts = points(point_validity,:);     
    points = newPts; 
    newPts = newPts(point_validity,:); 
    [numPts, ~] = size(newPts); 
        
    %convert the points to camera frame 
    oldPts = inv(K)*[oldPts,ones(numPts,1)]';
    newPts = inv(K)*[newPts,ones(numPts,1)]';
    
    %find deltaT 
    currTime = sensor.t; 
    currDeltaT = currTime - oldTime; 
    
    weight = 0.3;
    deltaT = weight* currDeltaT +(1-weight) * oldDeltaT; 
    
    %set old time = curr time 
    oldTime = currTime; 
    oldDeltaT = currDeltaT; 
       
    %Find the velocity in camera frame 
    distVector = newPts - oldPts;
    velocityVector = distVector / deltaT; 

    %find average of the points to use 
    midPts = (oldPts + newPts)./2;
    
    %Estimate pose (from world to camera) from the last project 
    [H,RCameraToWorld] = estimate_pose(sensor, K, XYZ, Yaw, tagIDs);
    
    %find the z, depth 
    worldPts = inv(H) * midPts; 
    Zs = worldPts(3,:).^(-1);
    
    %Do RANSAC: 
    pixels = 2;
    focalLength = 311;
    maxDistance = pixels/focalLength; 
    numIter = 100; 
    maxInlierCount = 0;
    bestIndices = [];
    
    for k = 1:numIter
        %get three random points to estimate a 3D line 
        indices = randperm(numPts,3)';
        
        %find the three velocity vector 
        velocitiesForLine = velocityVector(:,indices);
        positionsForLine = midPts(:,indices); 
        XsForLine = positionsForLine(1,:)';
        YsForLine = positionsForLine(2,:)';
        ZsForLine = Zs(indices)'; 
                    
        %Comupte Velocity Estimation 
        %find the F matrix (6x6) 
        f = findF(XsForLine, YsForLine,ZsForLine);
        
        %f = findF(positionsForLine,ZsForLine);
        %Find the UV vector to find the V and Omega 
        uvVector = [velocitiesForLine(1,:),velocitiesForLine(2,:)]';
        %[velocitiesForLine(1,1);velocitiesForLine(2,1);velocitiesForLine(1,2);velocitiesForLine(2,2);velocitiesForLine(1,3);velocitiesForLine(2,3)];
        %Find V and Omega 
        velocities = inv(f) *  uvVector;
        
        %local num Inliers 
        numInliers = 0;

        %Find the estimated velocity P-dot
        %Find the Xs and Ys and Zs for the F matrix 
        Xs = midPts(1,:)';
        Ys = midPts(2,:)';
        ZsT = Zs';

        %find the F for all the points 
        F1 = [-1./ZsT, zeros(numPts,1), Xs./ZsT, Xs.*Ys, -(1+Xs.^2), Ys];
        F2 = [zeros(numPts,1), -1./ZsT, Ys./ZsT, (1+Ys.^2), -Xs.*Ys, -Xs];
        
        F = [F1;F2];
        
        %Find the actual velocities in udot...., vdot....
        expectedVelocities = F * velocities;
        
        %split the uuuuvvvv array in half 
        expectedVelXDotYDot = [expectedVelocities(1:numPts,1),expectedVelocities(numPts+1:end,1)];
        
        %Find the actual velocities 
        XDots = velocityVector(1,:)';
        YDots = velocityVector(2,:)';
        actualVelocities = [XDots,YDots];
        
        %Find the difference between actual and expected 
        differenceVec = (actualVelocities - expectedVelXDotYDot);
        
        %find the norms 
        norms = sqrt(differenceVec(:,1).^2 + differenceVec(:,2).^2);
        
        %find inliers 
        indiciesToKeep = norms < maxDistance;
        [numInliers,~] = size(find(indiciesToKeep == 1));
    
        %line results in a better fit of data than old line 
        if (numInliers > maxInlierCount)
            maxInlierCount = numInliers;
            bestIndices = indices; 
        end 
        
    end 

    %Comupte Velocity Estimation for the best points 
    %find the F matrix (6x6) 
    bestPositions = midPts(:,bestIndices);
    bestXs = bestPositions(1,:)';
    bestYs = bestPositions(2,:)';
    bestVelocities = velocityVector(:,bestIndices);
    bestZs = Zs(bestIndices)';
    
    %find the fs
    f = findF(bestXs,bestYs, bestZs); 
    
    %Find the UV vector to find the V and Omega 
    uvVector = [bestVelocities(1,:),bestVelocities(2,:)]';
    %uvVector = [bestVelocities(1,1);bestVelocities(2,1);bestVelocities(1,2);bestVelocities(2,2);bestVelocities(1,3);bestVelocities(2,3)];
    %Find V and Omega 
    velocities = inv(f)*uvVector;
    cameraVel = velocities(1:3);
    cameraOmg = velocities(4:6); 
    
    %convert the results to the world frame 
    RCameraToWorld = RCameraToWorld';
    omg = RCameraToWorld * cameraOmg;
    vel = RCameraToWorld * cameraVel + cross(omg,RCameraToWorld*XYZ'); 
       
%     if (numPts < minNumberCornersNeeded) 
%         %get all the harris corners of the image 
%         release(tracker);
%         pointsObj = detectFASTFeatures(I);
%         newPts = pointsObj.selectStrongest(200).Location;
%         initialize(tracker,newPts,I); 
%     end 
    oldImg = I; 
end


end
function [f] = findF(XsForLine,YsForLine,ZsForLine)
     f1 = [-1./ZsForLine, zeros(3,1), XsForLine./ZsForLine, XsForLine.*YsForLine, -(1.+XsForLine.^2), YsForLine];
     f2 = [zeros(3,1), -1./ZsForLine, YsForLine./ZsForLine, (1.+YsForLine.^2), -XsForLine.*YsForLine, -XsForLine];
     f = [f1;f2];
%     f = [];
%     for i = 1:3
%         X = XsForLine(i); 
%         Y = YsForLine(i); 
%         Z = ZsForLine(i); 
%         
%         %find F1 and F2
%         f1 = [-1/Z, 0, X/Z, X*Y, -(1+X^2), Y];
%         f2 = [0, -1/Z, Y/Z, (1+Y^2), -X*Y, -X];
%         
%         f = [f; f1; f2];
%     end 
end 

% 
% function [f] =  findF(positions,Zs)
%     f = [];
%     for i = 1:3
%         X = positions(i,1); 
%         Y = positions(i,2); 
%         Z = Zs(i); 
%         
%         %find F1 and F2
%         f1 = [-1/Z, 0, X/Z, X*Y, -(1+X^2), Y];
%         f2 = [0, -1/Z, Y/Z, (1+Y^2), -X*Y, -X];
%         
%         f = [f; f1; f2];
%     end 
%end 

