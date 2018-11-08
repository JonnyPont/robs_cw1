clear all;
clf;


map=[0,0;60,0;60,45;45,45;45,59;106,59;106,105;0,105]; 
variance = 0;
sensors = 20;

robot = BotSim(map); 
robot.randomPose(10);

robot.setScanConfig(robot.generateScanConfig(sensors));
robot.setSensorNoise(variance);

while true
    currentPos = robot.getBotPos();
    currentAng = robot.getBotAng();
    if inpolygon(currentPos(1),currentPos(2),map(:,1),map(:,2)) ~= 1
        break
    else
        [distances, crossingPoint]  = robot.ultraScan();
        if distances(1) < 12
            robot.turn(pi + 0.1*pi*(rand*2 - 1))
            robot.move(5)
        else
            robot.turn(0.25*pi*(rand*2 - 1))
            robot.move(10*rand)
        end
    end
    
    clf; 
    hold on
    robot.drawMap() 
    robot.drawBot(1) 
    drawnow
    i
end

robot.drawMap() 
robot.drawScanConfig() 
robot.drawBot(1)