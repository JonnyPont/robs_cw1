function [botSim] = localise_handin(botSim,map,target)
%This function returns botSim, and accepts, botSim, a map and a target.
%LOCALISE Template localisation function

%% setup code
%you can modify the map to take account of your robots configuration space
% map=[0,0;0,50;75,50;75,65;10,65;10,85;35,85;35,125;10,125;10,145;75,145;75,85;110,85;110,155;125,155;125,40;150,40;150,15;100,15;100,0;80,0;80,30;40,30;40,0];
modifiedMap = map; %you need to do this modification yourself
botSim.setMap(modifiedMap);

%generate some random particles inside the map
num =300; % number of particles
sensors = 6;
particles(num,1) = BotSim; %how to set up a vector of objects
for i = 1:num
    particles(i) = BotSim(modifiedMap);  %each particle should use the same map as the botSim object
    particles(i).randomPose(0); %spawn the particles in random locations
    particles(i).setScanConfig(particles(i).generateScanConfig(sensors));
end
botSim.setScanConfig(botSim.generateScanConfig(sensors));

%% Localisation code
maxNumOfIterations = 30;
n = 0;
converged =0; %The filter has not converged yet
convergedCount = 0;
foundGoal = 0;
positionEstimate=[];
path = [];
while(foundGoal == 0)%&& n < maxNumOfIterations) %%particle filter loop
    n = n+1; %increment the current number of iterations
    robotDist = botSim.ultraScan(); %get a scan from the real robot.
    
    %% Write code for updating your particles scans
    locations = zeros(2,num);
    angles = zeros(1,num);
    for i = 1:num %For each particle
        %Check that particle is within map
        if particles(i).insideMap() == 0
            particles(i).randomPose(5)
        end
        locations(:,i) = particles(i).getBotPos(); % Save particle location
    end
    %% Write code for scoring your particles
    %Matrix initialisations
    probabilities = zeros(1,num);
    partWeight = zeros(sensors,1);
    dampFactor = 0;
    var = 100;
    for i = 1:num
        particleDistance = particles(i).ultraScan(); % Particle distance reading
        for j = 1:sensors
            possDist = circshift(particleDistance,j);
            currDiff = norm(possDist - robotDist);
            partWeight(j) = (1/sqrt(2*pi*var))*exp(-(currDiff)^2/(2*var));
        end
        [bestProb,bestRot] = max(partWeight);
        probabilities(i) = bestProb + dampFactor; % Mean probability of distance measures to robot
        angles(i) = particles(i).getBotAng() + 2*pi*(bestRot/sensors); % Set angle 
    end
    %% Write code for resampling your particles
    
    probabilities = probabilities/sum(probabilities);
    
    %Roulette Wheel Sampling
    for i=1:num
        seed = rand(); %Cannot exceed 1 as cumsum is 1.
        cumProbs = cumsum(probabilities);
        newLocIndex = find(seed <= cumProbs,1);
        particles(i).setBotPos([locations(1,newLocIndex)'+2*rand-1 locations(2,newLocIndex)'+2*rand-1]);
        particles(i).setBotAng(angles(newLocIndex)+0.2*rand-0.1);
    end
    %% Write code to check for convergence
    %Save updated particle locations
    for i=1:num
        locations(:,i) = particles(i).getBotPos();
    end
    %Check for convergence
    if std(locations(1,:)) < 2 && std(locations(2,:)) < 2
        fprintf('CONVERGED')
        convergedCount = convergedCount + 1
    %Only reset convergence count before robot found for the first time
    elseif converged == 0 
        convergedCount = 0;
    end
    if convergedCount >= 3
        positionEstimate = [mean(locations(1,:)) mean(locations(2,:))];
        converged = 1;
        found = 1;
    end
    %% Write code to decide how to move next
    if converged == 0
        %Random walker code
        currentPos = botSim.getBotPos();
        if inpolygon(currentPos(1),currentPos(2),map(:,1),map(:,2)) ~= 1
            break
        else
            robotDist = botSim.ultraScan();
            if robotDist(1) < 10
                turn = pi + 0.1*pi*(rand*2 - 1);
                move = 4;
                botSim.turn(turn)
                botSim.move(move)
            else
                turn = 0.25*pi*(rand*2 - 1);
                move = 8*rand;
                botSim.turn(turn)
                botSim.move(move)
            end
        end
    elseif converged == 1
        %Implement A* Algorithm
        %If we think the estimated robot position is next to the goal then cut
        if norm(positionEstimate - target) < 10
            foundGoal = 1;
        end
        resolution = 12; %set grid resolution       
        while isempty(path)
            path = aStarSearch(positionEstimate,target,map,resolution);
            resolution = resolution - 2;
            %Once path is made, define first direction node.
            pathLength = size(path);
            pathCounter = pathLength(1)-1;
        end
        
        %This finds the subsequent nodes that need to be moved towards in
        %the direction of the goal with a check for if we've reached them
        %or if they're the final node to be sought.
        tempTarget = path(end-pathCounter,:);      
        %If we're at the node then stop
        if norm(tempTarget - positionEstimate) < 1
            if tempTarget == target
                foundGoal = 1;
            end
            pathCounter = pathCounter+1;
        %Else move 1 step in the direction of the nearest node    
        else
            %Move a step of 4 unless the target is less than 4 away
            if norm(tempTarget - positionEstimate) < 4
                move = norm(tempTarget - positionEstimate);
            else 
                move = 4;
            end
            %Set the angle the bot needs to move in.
            xDiff = tempTarget(1) - positionEstimate(1);% - tempTarget(1);
            yDiff = tempTarget(2) - positionEstimate(2);% - tempTarget(2);
            angle = 0.5*pi - atan2(xDiff,yDiff)
            turn = angle - mod(abs(botSim.getBotAng()),2*pi);
            botSim.turn(turn)
            botSim.move(move)
        end
    end    

    %Randomly place some points in case of incorrect convergence
    for i =1:0.1*num
        someLoc = floor(rand*num)+1;
        particles(someLoc).randomPose(0)
        particles(someLoc).setBotAng(pi - 2*pi*rand)
    end

    for i=1:num %for all the particles.
        particles(i).turn(turn); %turn the particle in the same way as the real robot
        particles(i).move(move); %move the particle in the same way as the real robot       
    end
    %% Drawing
    %only draw if you are in debug mode or it will be slow during marking
    if botSim.debug()
        hold off; %the drawMap() function will clear the drawing when hold is off
        botSim.drawMap(); %drawMap() turns hold back on again, so you can draw the bots
        botSim.drawBot(30,'g'); %draw robot with line length 30 and green
        if ~isempty(positionEstimate)
            plot(positionEstimate(1),positionEstimate(2),'r*');
        end
        if ~isempty(path)
            for i = 1:(pathLength(1)-1)
                plot([path(i,1),path(i+1,1)],[path(i,2),path(i+1,2)])
            end
        end    
%         for i =1:num
%             particles(i).drawBot(3); %draw particle with line length 3 and default color
%         end
        drawnow;
    end
end
end
