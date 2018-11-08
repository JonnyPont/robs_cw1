clc;
clf;
clear;

map = [0,0;100,0;100, 100;0, 100];
count = 0;
robot = BotSim(map);

%Initial positioning
robot.randomPose(0); %robot

goal = genGoal(100,100);

%Set some movement noise into the system.
robot.setMotionNoise(0.2) 
robot.setTurningNoise(0.2)

while norm(robot.getBotPos - goal) > 1
    
    if norm(robot.getBotPos - goal) < 4 && count < 5
        count = count + 1;
        goal = genGoal(100,100);
    end    
    
    robotLoc = robot.getBotPos;
    xDist = goal(1)-robotLoc(1);
    yDist = goal(2)-robotLoc(2);
    angleToGoal = atan2(yDist,xDist);
    robot.turn(angleToGoal-robot.getBotAng);
    robot.move(2)

    %rather than doing this it would be better to pre-check if the robot is
    %going to move into a wall and not move if that is the case.
    if robot.pointInsideMap(robotLoc) == 0
        fprintf("Out of Map")
        break
    end    
    
    robot.drawMap;
    hold on
    robot.drawBot(0);
    plot(goal(1),goal(2),'r*');
    
end


function goal = genGoal(maxX,maxY)

goalX = maxX*rand;
goalY = maxY*rand;
goal = [goalX goalY]; %Random goal location

end