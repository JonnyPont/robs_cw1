clf;        %clears figures
clc;        %clears console
clear;      %clears workspace
axis equal; %keeps the x and y scale the same
%default map
%map=[0,0;60,0;60,45;45,45;45,59;106,59;106,105;0,105];  
% A square
map=[0,0;0,100;100,100;100,0];
%Gus map of fun
% map=[0,0;0,50;75,50;75,65;10,65;10,85;35,85;35,125;10,125;10,145;75,145;75,85;110,85;110,155;125,155;125,40;150,40;150,15;100,15;100,0;80,0;80,30;40,30;40,0];
% A symmetric map

robot = BotSim(map,[0.01,0.005,0]);  %sets up a botSim object a map, and debug mode on.

%Random Positions
start = robot.getRndPtInMap(10)
goal = robot.getRndPtInMap(10)

%Error positions
% start = [45.18 124.34]
% goal = [115 73]
% 
% start = [10.25 33.61]
% goal = [47.15 82.74]

robot.drawMap();
plot(goal(1),goal(2),'r*');
plot(start(1),start(2),'g*');
drawnow;

resolution = 12;

path = aStarSearch(start,goal,map,resolution);

while isempty(path)
    resolution = resolution - 2;
    path = aStarSearch(start,goal,map,resolution);
end    

pathLength = size(path);
for i = 1:(pathLength(1)-1)
%     myLine = [path(i,:)]
    plot([path(i,1),path(i+1,1)],[path(i,2),path(i+1,2)])
end 

% plot([path(i,1),path(i+1,1)],[path(i,2),path(i+1,2)])
