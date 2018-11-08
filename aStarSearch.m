function [ output_args ] = aStarSearch(start,goal)
% A* search algorithm

closed = [];
open = [start];

map=[0,0;60,0;60,45;45,45;45,59;106,59;106,105;0,105];  %default map


%Makes a kind of iterable map of the world. I think this is a map of points
%rather than a map of squares/grids. I think I need it to be squares/grids
%to be able to get anywhere as the robot will not move in exact integer
%increments.
sizes = max(map);
mapGrid = zeros(sizes(1),sizes(2));
fCostsMap = zeros(sizes(1),sizes(2))+1000; %make a cost matrix
fCostsMap(start(1),start(2)) = 0; %set start cost to zero 
for i=1:sizes(1)
    for j=1:sizes(2)
        if botSim.pointInsideMap(mapGrid(i,j)) == 1 %inpolygon(i,j,map(:,1),map(:,2)) == 1 %pointInsideMap() will be better once botSim objects are created
            mapGrid(i,j) = 1;
        elseif inpolygon(i,j,map(:,1),map(:,2)) ~= 1
            mapGrid(i,j) = 0;
        end
    end
end



while length(open) ~= 0
   %open all nodes in open storing the cost f(x) to get to them. PROBLEM:
   %This currently will just find that the values of the f function are
   %1000 for all new nodes. When do these need updating? --- I think this will
   %be fine as the nodes here will have been put into the "open list" 
   currentNode = [open(1),open(1+length(open))];
   currentNodeF = fCostsMap(currentNode(1),currentNode(2));
   for i = 1:length(open)
       testNode = [open(i),open(i+length(open))];
       testNodeF = fCostsMap(testNode(1),testNode(2));
       if testNodeF < currentNodeF
           currentNode = testNode;
           currentNodeF = testNodeF;
       end
   end
   
   %Use NORM as a function to find Euclidean Distance
   
   %At this point, the currentNode should be the node with the lowest f(x)
   %score
   
   
   if currentNode == goal
       make_path()
   end
   open.remove(currentNode)
   closed.add(currentNode)
   
   %This loop should run through and find all neighbours of the currentNode
   %ready to be tested and update the f(x) values of all these new nodes.
   neighbourList = [];
   for i = -1:1:1
       for j = -1:1:1
           neighbour = [currentNode(1)+i,currentNode(2)+j];
           neighbourList = [neighbourList;neighbour];
           %Find F of neighbour
           neighbourF = currentNodeF + norm([[0 0] [i j]]); %fcost of node + distance to neighbour
           %Update the neighbour's fCost matrix value IF this path to the specific node is shorted.
           if neighbourF < fCostsMap(neighbour(1),neighbour(2))
               fCostsMap(neighbour(1),neighbour(2)) = neighbourF;
           end    
       end
   end
   
   
   
   open.add('All surrounding states')

%    calculate h(x) for all of the neighbouring nodes  --- I've only made a
%    very small start on this. I fear I'm going too deep without having
%    tested any of the code so far.
%    for i = 1:length(neighbourList)
%        if [neighbour(i),neighbour(i+length(neighbourList))] ISMEMBER closed
%            continue
%        end
%    end
%    
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
    return currentNode;
end