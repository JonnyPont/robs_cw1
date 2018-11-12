function [path] = aStarSearch(start,goal,map,resolution)
% A* search algorithm - printing the values of the nodes that get entered
% into open shows there must be some sort of bug somewhere for nodes where i=j
%Ensure problem is possible before starting

if ~inpolygon(start(1),start(2),map(:,1),map(:,2))
    fprintf('Start point not in map')
    completable = 0;
elseif ~inpolygon(goal(1),goal(2),map(:,1),map(:,2))
    fprintf('Goal point not in map')
    completable = 0;
end

sizes = max(map);
mapGrid = zeros(sizes(1)+1,sizes(2)+1); % overcome 0 indexing
size(mapGrid);

% initialise g(x) - inf and h(x) - based on euclidean distances
gCostsMap = zeros(sizes(1)+1,sizes(2)+1)+Inf; %make a cost matrix - overcoming 1 indexing
startCostsMap = zeros(sizes(1)+1,sizes(2)+1)+Inf; %make a cost matrix - overcoming 1 indexing
parents = cell(sizes(1)+1,sizes(2)+1);
hCostsMap = zeros(sizes(1)+1,sizes(2)+1);


%Find nearest node to start position.


for i=1:resolution:sizes(1)+1
    for j=1:resolution:sizes(2)+1
        hCostsMap(i,j) = distance([i j],goal);
        startCostsMap(i,j) = distance([i j],start);
        plot(i,j,'k.')
    end
end

%I need to make sure that this only ever gives 1 value. Even when
%equidistant
[minVal,minValIndexSquashed] = min(reshape(startCostsMap, numel(startCostsMap),1));
[indexI,indexJ] = ind2sub(size(startCostsMap),minValIndexSquashed)
gCostsMap(indexI+1,indexJ+1) = 0; %set start cost to zero
firstNode = [indexI indexJ];


closed = [];
open = firstNode;
completable = 1;
while isempty(open) == 0 && completable == 1
    %open all nodes in open storing the cost g(x) to get to them.
    openSize = size(open); %Get list length of open
    currentNode = open(1,:); % Get two corresponding points
    currentNodeG = gCostsMap(currentNode(1)+1,currentNode(2)+1);
    goalVisible = 1;
    
    % use intersection code to find visible nodes.
    neighbourList = [];
    lengthClosed = size(closed);
    
    for i = 1:resolution:sizes(1)+1
        for j = 1:resolution:sizes(2)+1
            check = 0;
            alreadyExplored = 0;
            neighbourTemp = [i,j];
            %I should make an iteration where the first loop, the code
            %should check if the goal destination is visible and if it is,
            %then move straight to it. ie. as soon as the goal is in sight,
            %go for it.
            for k = 1:length(map)
                if k <length(map)
                    x = intersection1([currentNode goal],[map(k,:),map(k+1,:)]);
                elseif k == length(map)
                    x = intersection1([currentNode goal],[map(k,:),map(1,:)]);
                end
                if ~isnan(x)
                    goalVisible = 0;
                end
            end
            
            %Test if node has been explored already
            for testClosed = 1:lengthClosed(1)
                if neighbourTemp == closed(testClosed,:)
                    alreadyExplored = 1;
                end
            end
            
            %Test if node is current being explored
            if neighbourTemp == open(1,:)
                alreadyExplored = 1;
            end
            
            % If ok, add node to neighbours
            if alreadyExplored == 0
                for k = 1:length(map)
                    if k <length(map)
                        x = intersection1([currentNode neighbourTemp],[map(k,:),map(k+1,:)]);
                    elseif k == length(map)
                        x = intersection1([currentNode neighbourTemp],[map(k,:),map(1,:)]);
                    end
                    if ~isnan(x)
                        check = 1;
                    end
                end
                if check == 0
                    neighbourList(end+1,:) = neighbourTemp;
                end
            end
        end
    end
    
    neighbourList;
    %This neighbour list is good and correct. Need to go on to find g scores
    %of each member next!
    
    lengthNeighbourList = size(neighbourList);    
    tempFScores = zeros(1,lengthNeighbourList(1));
    for i = 1:lengthNeighbourList(1)
        dist = distance(currentNode,neighbourList(i,:));
        newGScore = dist + currentNodeG;
        gCostsMap(neighbourList(i,1),neighbourList(i,2));
        if newGScore < gCostsMap(neighbourList(i,1)+1,neighbourList(i,2)+1)
            gCostsMap(neighbourList(i,1)+1,neighbourList(i,2)+1) = newGScore;
            neighbourList(i,1);
            neighbourList(i,2);
            parents{neighbourList(i,1)+1,neighbourList(i,2)+1} = currentNode; 
            tempFScores(i) = newGScore + hCostsMap(neighbourList(i,1)+1,neighbourList(i,2)+1);
        else
            tempFScores(i) = gCostsMap(neighbourList(i,1)+1,neighbourList(i,2)+1) + hCostsMap(neighbourList(i,1)+1,neighbourList(i,2)+1);
        end
    end
        
    tempFScores;

    
    if goalVisible == 1
        fprintf('Goal found')
        nodePathTrack = open(1,:);
        path = [goal;nodePathTrack];
        while nodePathTrack(1) ~= firstNode(1) || nodePathTrack(2) ~= firstNode(2)
            nodePathTrack = parents{nodePathTrack(1)+1,nodePathTrack(2)+1};
            path(end+1,:) = nodePathTrack;
        end
        path(end+1,:) = start;
        break
    elseif open(1,:) == goal
        fprintf("YEET")
        %Need to trace back through the parents matrix to find the path.
        nodePathTrack = open(1,:);
        path = nodePathTrack;
        while nodePathTrack(1) ~= firstNode(1) || nodePathTrack(2) ~= firstNode(2)
            nodePathTrack = parents{nodePathTrack(1)+1,nodePathTrack(2)+1};
            path(end+1,:) = nodePathTrack;
        end
        path(end+1,:) = start;
        break
    end   

    % Find the last lowest value in the list. The idea is that will be furthest
% away in the right direction, given the case where multiple nodes have the
% same score. Better to move big.
    nextNodeIndex = find(tempFScores==min(tempFScores),1,'last');
    open = []; %only one node stored so reset open
    open(end+1,:) = [neighbourList(nextNodeIndex,1) neighbourList(nextNodeIndex,2)];
    closed(end+1,:) = [currentNode(1) currentNode(2)]; %add explored node to closed
%     if ~isempty(nextNodeIndex)
% %         fprintf('problemo')
% 
%     end
    
%    for i = 1:openSize(1)
%        testNode = open(i,:); 
%        testNodeG = gCostsMap(testNode(1)+1,testNode(2)+1);
%        if testNodeG <= currentNodeG
%            currentNode = testNode;
%            currentNodeLoc = i;
%            currentNodeG = testNodeG;
%        end
%    end
   
   %At this point, the currentNode should be the node with the lowest f(x)
   %score

   
%    %This loop should run through and find all neighbours of the currentNode
%    %ready to be tested and update the f(x) values of all these new nodes.
%    %I need to add a check of if the node is in the map or not
%    neighbourList = [];
%    for i = -1:1:1
%        for j = -1:1:1
%            neighbour = [currentNode(1)+i,currentNode(2)+j];
%            if inpolygon(neighbour(1),neighbour(2),map(:,1),map(:,2)) == 1 %no need for +1 here due to both being pre-calibrated
%                neighbourList = [neighbourList;neighbour];
%                %Find G of neighbour
%                neighbourG = currentNodeG + norm([[0 0] [i j]]); %gcost of node + distance to neighbour
%                %Update the neighbour's gCost matrix value IF this path to the specific node is shorted.
%                if neighbourG < gCostsMap(neighbour(1)+1,neighbour(2)+1)
%                    gCostsMap(neighbour(1)+1,neighbour(2)+1) = neighbourG;
%                end
%            end
%        end
%    end
%    
%    %Check to see if a neighbour has already been explored. - hasn't been
%    %tested for functionality
%    neighbourListLength = size(neighbourList);
%    lengthClosed = size(closed);
%    for i = 1:neighbourListLength(1)
%        alreadyExplored = 0;
%        for j = 1:lengthClosed(1)
%            if neighbourList(i,:) == closed(j,:)
%                 alreadyExplored = 1;
%            end
%        end
%        if alreadyExplored == 0
%             open(end+1,:) = neighbourList(i);
%        end
%    end    

end

end

function [ crossingPoint] = intersection1(infVec1,line2)
%INTERSECTION calculates the intersection point between an infinite vector
%and a bounded line
ua = ((line2(:,3)-line2(:,1)).*(infVec1(:,2)-line2(:,2))-(line2(:,4)-line2(:,2)).*(infVec1(:,1)-line2(:,1)))./(((line2(:,4)-line2(:,2)).*(infVec1(:,3)-infVec1(:,1))-(line2(:,3)-line2(:,1)).*(infVec1(:,4)-infVec1(:,2))));
ub = ((infVec1(:,3)-infVec1(:,1)).*(infVec1(:,2)-line2(:,2))-(infVec1(:,4)-infVec1(:,2)).*(infVec1(:,1)-line2(:,1)))./(((line2(:,4)-line2(:,2)).*(infVec1(:,3)-infVec1(:,1))-(line2(:,3)-line2(:,1)).*(infVec1(:,4)-infVec1(:,2))));
filter = ub >= 0 & ub <= 1 & ua >= 0 & ua <= 1;
% filter = ub >= 0 & ub <= 1 & ua>=0; %
crossingPoint =[infVec1(:,1)+ua.*(infVec1(:,3)-infVec1(:,1)) infVec1(:,2)+ua.*(infVec1(:,4)-infVec1(:,2))];
for i = 1:length(filter)
    if filter(i) == 0
        crossingPoint(i,:) = NaN(1,2);
    end
end
end