function [ nextNode ] = aStarSearch(start,goal)
% A* search algorithm - printing the values of the nodes that get entered
% into open shows there must be some sort of bug somewhere for nodes where i=j 

closed = [];
open = start;

map=[0,0;60,0;60,45;45,45;45,59;106,59;106,105;0,105];  %default map

mapGrid = zeros(10,10); % zeros(sizes(1)+1,sizes(2)+1); %overcome 0 indexing
sizes = size(mapGrid);
size(mapGrid)
% gCostsMap(start(1)+1,start(2)+1) = 0; %set start cost to zero 

% initialise g(x) - inf and h(x) - based on euclidean distances
gCostsMap = zeros(sizes(1)+1,sizes(2)+1)+Inf; %make a cost matrix - overcoming 1 indexing
gCostsMap(start(1)+1,start(2)+1) = 0; %set start cost to zero 
hCostsMap = zeros(sizes(1)+1,sizes(2)+1);

for i=1:sizes(1)+1
    for j=1:sizes(2)+1
        hCostsMap(i,j) = distance([i j],goal);
    end
end    

fCostsMap = gCostsMap + hCostsMap; % Which node is explored next completely depends on the fCostsMap. I will make sure this happens after ensuring heuristics get included and added in the script.

% Determine what points are inside and outside of the map - might not be
% needed
for i=1:sizes(1)
    for j=1:sizes(2)
        if inpolygon(i,j,map(:,1),map(:,2)) == 1 %This is equivalent to botSim.pointInsideMap(mapGrid(i,j)) == 1 
            mapGrid(i,j) = 1;
        elseif inpolygon(i,j,map(:,1),map(:,2)) ~= 1
            mapGrid(i,j) = 0;
        end
    end
end


while length(open) ~= 0
   %open all nodes in open storing the cost g(x) to get to them.
   openSize = size(open); %Get list length of open
   currentNode = open(1,:); % Get two corresponding points
   currentNodeG = gCostsMap(currentNode(1)+1,currentNode(2)+1);
   
   % use intersection code to find visible nodes.
   neighbourList = [];
   for i = 1:sizes(1)+1
       for j = 1:sizes(2)+1
           check = 0;
           neighbourTemp = [i,j]*9;
           for k = 1:length(map)
               if k <length(map)
%                    boundingLine = [map(k,:),map(k+1,:)];
                   x = intersection([currentNode neighbourTemp],[map(k,:),map(k+1,:)]);
               elseif k == length(map)
%                    boundingLine = [map(k,:),map(1,:)];
                   x = intersection([currentNode neighbourTemp],[map(k,:),map(1,:)]);
               end 
               if ~isnan(x)
                   check = 1;
               end
           end
           if check == 0
               neighbourList(end+1,:) = neighbourTemp;
               fprintf('Add node')
           end
       end    
   end
   
   neighbourList
   %This neighbour list is good and correct. Need to go on to find g scores
   %of each member next!
   
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
   
   if currentNode == goal
       fprintf("YEET")
       break
       make_path() %this is still pseudocode I think
   end

   open(currentNodeLoc,:) = [];
   closed(end+1,:) = [currentNode(1) currentNode(2)];
   
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
   %Check to see if a neighbour has already been explored. - hasn't been
   %tested for functionality
   neighbourListLength = size(neighbourList);
   lengthClosed = size(closed);
   for i = 1:neighbourListLength(1)
       alreadyExplored = 0;
       for j = 1:lengthClosed(1)
           if neighbourList(i,:) == closed(j,:)
                alreadyExplored = 1;
           end
       end
       if alreadyExplored == 0
            open(end+1,:) = neighbourList(i);
       end
   end    

%    calculate h(x) for all of the neighbouring nodes  --- I've only made a
%    very small start on this. I fear I'm going too deep without having
%    tested any of the code so far.
%    heuristics = zeros(neighbourListLength(1),1);
%    for i = 1:neighbourListLength(1)
%         heuristics(i) = distance(neighbourList(i,:),goal);
%    end

fCostsMap;

%    for 'each neighbour of currentNode(in all directions)'
%         if neighbour 'in closed'
%             skip
%         else
%             heuristic = 'euclidean distance to goal';
%             potentialG = current.gScore+dist_between(currentNode,neighbour);
%             
%             if neighbour not in open
%                 open.add(neighbour)
%             elseif potentialG > neighbour.gscore
%                 pass
%             else
%                 neighbour.gscore = potentialG
%                 neighbour.parent = currentNode
%                 neighbour.fscore = potentialG + neighbour.hscore
%         end
%    end    
   %calculate the predicted g(x) to get to the final goal
   
   %select the node with lowest G and either terminate if it is the goal
   %state or repeat again if it isn't
end