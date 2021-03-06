function [path, num_expanded] = dijkstra(map, start, goal, astar)
% DIJKSTRA Find the shortest path from start to goal.
%   PATH = DIJKSTRA(map, start, goal) returns an M-by-3 matrix, where each row
%   consists of the (x, y, z) coordinates of a point on the path.  The first
%   row is start and the last row is goal.  If no path is found, PATH is a
%   0-by-3 matrix.  Consecutive points in PATH should not be farther apart than
%   neighboring cells in the map (e.g.., if 5 consecutive points in PATH are
%   co-linear, don't simplify PATH by removing the 3 intermediate points).
%
%   PATH = DIJKSTRA(map, start, goal, astar) finds the path using euclidean
%   distance to goal as a heuristic if astar is true.
%
%   [PATH, NUM_EXPANDED] = DIJKSTRA(...) returns the path as well as
%   the number of points that were visited while performing the search.
if nargin < 4
    astar = false;
end

grid = map.occgrid;
[maxI,maxJ,maxK] = size(grid); 
%safeDistance = map.margin; 
res = map.res_xyz;

%initialize the matrices 
path = zeros(1,3);
num_expanded = 0;
distances = ones(maxI,maxJ,maxK)*1000000;
parents = zeros(maxI,maxJ,maxK); 

%create a list of the unvisited nodes
unvisited = ones(maxI,maxJ,maxK); 
unvisited = unvisited .* ~grid; %shows one where it's possible to go

%create the H matrix for the A* algorithm 
H = zeros(maxI,maxJ,maxK);
%xyzs = zeros(maxI,maxJ,maxK);
[J,I,K] = meshgrid(1:maxJ,1:maxI,1:maxK);
for index = 1:(size(H(:),1))
    iH = I(index);
    jH = J(index); 
    kH = K(index); 
    xyzH = map.subToXYZ([iH,jH,kH]);
    H(index) = norm(xyzH - goal);
end 

%initialization of algorithm
startIJK = map.xyzToSub(start);
iStart = startIJK(1); 
jStart = startIJK(2); 
kStart = startIJK(3);   
distances(iStart,jStart,kStart) = .01; 

%main loop 
while (goalUnvisited(goal,unvisited,map) && findMinPath(unvisited,distances,astar,H) <= 1000000)
    [minNodeVal,indexMinNode] = findMinPath(unvisited, distances, astar,H);
    unvisited(indexMinNode) = 0; %set as visited 
    
    coordsOfNode = map.indToXYZ(indexMinNode);
    xNode = coordsOfNode(1); 
    yNode = coordsOfNode(2); 
    zNode = coordsOfNode(3); 
    
    subsOfNode = map.xyzToSub(map.indToXYZ(indexMinNode));
    iNode = subsOfNode(1); 
    jNode = subsOfNode(2); 
    kNode = subsOfNode(3); 
    
    num_expanded = num_expanded + 1;
    
    %find all the neighbors of the min node 
    neighborsOfMin = findNeighbors(map,indexMinNode); 
    
    %loop through all the neighbors of the minNode
    for node = 1:size(neighborsOfMin,1)
        %check if the nodes are possible or not
        subsNeighbor = neighborsOfMin(node,:);
        xyzNeighbor = map.subToXYZ(subsNeighbor); 
        xNeighbor = xyzNeighbor(1); 
        yNeighbor = xyzNeighbor(2); 
        zNeighbor = xyzNeighbor(3); 
        
        iNeighbor = subsNeighbor(1);
        jNeighbor = subsNeighbor(2); 
        kNeighbor = subsNeighbor(3);
        
        if (nodeIsPossible(subsNeighbor(1),subsNeighbor(2),subsNeighbor(3),maxI,maxJ,maxK))
            costOfPath = ((xNode-xNeighbor)^2+(yNode-yNeighbor)^2 +(zNode-zNeighbor)^2)^.5;
            newDist = distances(iNode,jNode,kNode) + costOfPath; 
            if (newDist < distances(iNeighbor,jNeighbor,kNeighbor))
                distances(iNeighbor,jNeighbor,kNeighbor) = newDist; 
                parents(iNeighbor,jNeighbor,kNeighbor) = indexMinNode; 
            end 
        end 
        
    end 
    
end

if ~goalUnvisited(goal,unvisited,map)
    %find the actual path 
    goalSubs = map.xyzToSub(goal);
    path(1,:) = goalSubs;
    counter = 2; 
    startSubs = map.xyzToSub(start); 
    parent = 100; 
    while parent ~= 0 
        lastItem = path(counter-1,:);
        parent = parents(lastItem(1),lastItem(2),lastItem(3));
        subsOfParent = map.xyzToSub(map.indToXYZ(parent)); 
        path(counter,:) = subsOfParent;
        counter = counter + 1; 
    end

    %flip the path so that the start node is at the beginning
    path = flip(path); 
    path = map.subToXYZ(path); 

    %set the first value as the start and the last node as the goal
    path(1,:) = start; 
    path(size(path,1),:) = goal;
    
else %algo didn't see the goal
    path= zeros(0,3); 
end 

end

function [neighbors] = findNeighbors(map, indexMinNode)
    subs = map.xyzToSub(map.indToXYZ(indexMinNode)); 
    i = subs(1); 
    j = subs(2); 
    k = subs(3); 
    
    neighbors = [i-1,j,k;
                i+1,j,k;
                i,j-1,k;
                i,j+1,k;
                i,j,k-1;
                i,j,k+1;];
end 

function [isPossible] = nodeIsPossible(x,y,z,maxI,maxJ,maxK)
    isPossible = x > 0 && x <= maxI && y > 0 && y <= maxJ && z > 0 && z <= maxK;
end 

function [minNodeValue,index] = findMinPath(unvisited,distances,astar,H)
    allPossibleVals = unvisited.*distances; 
    allPossibleVals(allPossibleVals==0) = 1000000;
    if (astar)
        allPossibleVals = allPossibleVals + H;
    end 
    [minNodeValue,index] = min(allPossibleVals(:)); 
end 

function [pathIsFound] = goalUnvisited(goal,unvisited,map)
    indexOfGoal = map.xyzToInd(goal);
    pathIsFound = (unvisited(indexOfGoal)==1);
end 