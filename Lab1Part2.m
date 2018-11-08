clear all;
clf;


map=[0,0;60,0;60,45;45,45;45,59;106,59;106,105;0,105]; 
variance = 0;
sensors = 20;




robot = BotSim(map); 
robot.randomPose(10);

robot.setScanConfig(robot.generateScanConfig(sensors));
robot.setSensorNoise(variance);

for i = 1:1:100
    
    nextTurn = 0.5*pi*(rand*2 - 1);
    nextMove = 10*abs(rand);

    currentPos = robot.getBotPos();
    currentAng = robot.getBotAng();
    
    [distances, crossingPoint]  = robot.ultraScan();
    test = min(distances);
    
    if inpolygon(currentPos(1),currentPos(2),map(:,1),map(:,2)) ~= 1
        break
        
    elseif test > 10
        
        %Random robot move
        robot.turn(nextTurn);
        robot.move(nextMove);
    
        %problem arises when multiple walls set off sensors causing the
        %roboto to pick an incorrect direction. Get it to take the average
        %of dangerous angles and move away from the average one.
    elseif test < 10
        fprintf("Wall!")
                for k = 1:length(distances)
                    if distances(k) == test
                        detectedWall = [crossingPoint(k) crossingPoint(k+sensors)];
                        yDist = detectedWall(2)-currentPos(2);
                        xDist = detectedWall(1)-currentPos(1);
                        angleToWall = atan2(yDist,xDist);
                        precautionTurn = pi  - (angleToWall); %- mod(currentAng,2*pi)
                    end
                end
            
            %Random robot move
            robot.turn(precautionTurn);
            robot.move(nextMove);
    end
    
    %clf; 
    
    hold on
    robot.drawMap() 
    robot.drawBot(1) 
    drawnow
    
    i
end



robot.drawMap() 
robot.drawScanConfig() 
robot.drawBot(1) 