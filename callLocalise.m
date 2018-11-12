clf;        %clears figures
clc;        %clears console
clear;      %clears workspace
axis equal; %keeps the x and y scale the same
map=[0,0;60,0;60,45;45,45;45,59;106,59;106,105;0,105];  %default map
%Gus map of fun
% map=[0,0;0,50;75,50;75,65;10,65;10,85;35,85;35,125;10,125;10,145;75,145;75,85;110,85;110,155;125,155;125,40;150,40;150,15;100,15;100,0;80,0;80,30;40,30;40,0];


robot = BotSim(map,[0.01,0.005,0]);  %sets up a botSim object a map, and debug mode on.
%botSim = BotSim(map,[0,0,0]);  %sets up a botSim object a map, and debug mode on.

robot.setSensorNoise(0);
robot.drawMap();
drawnow;
robot.randomPose(10); %puts the robot in a random position at least 10cm away from a wall
target = robot.getRndPtInMap(10);  %gets random target.

tic %starts timer

%your localisation function is called here.
var = 100;
[returnedBot,probabilities,locations] = localise2(robot,map,target,var); %Where the magic happens
resultsTime = toc %stops timer

%calculated how far away your robot is from the target.
resultsDis =  distance(target, returnedBot.getBotPos())