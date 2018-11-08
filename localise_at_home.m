function [botSim] = localise_at_home(botSim,map,target)
%This function returns botSim, and accepts, botSim, a map and a target.
%LOCALISE Template localisation function

%% setup code
%you can modify the map to take account of your robots configuration space
modifiedMap = map; %you need to do this modification yourself
botSim.setMap(modifiedMap);

%generate some random particles inside the map
num =300; % number of particles
particles(num,1) = BotSim; %how to set up a vector of objects
for i = 1:num
    particles(i) = BotSim(modifiedMap);  %each particle should use the same map as the botSim object
    particles(i).randomPose(0); %spawn the particles in random locations
end
%% Localisation code
maxNumOfIterations = 30;
n = 0;
converged =0; %The filter has not converged yet
while(converged == 0 && n < maxNumOfIterations) %%particle filter loop
    n = n+1; %increment the current number of iterations
    robotDist = botSim.ultraScan(); %get a scan from the real robot.
    
    %% Write code for updating your particles scans
    locations = zeros(2,num);
    angles = zeros(1,num);
    angles2 = angles;
    particleDistance = [];
    for i = 1:num %For each particle
        locations(:,i) = particles(i).getBotPos(); % Save particle location
    end
    %% Write code for scoring your particles
    %Matrix initialisations
    probabilities = zeros(1,num);
    dampFactor = 0;
    
    for i = 1:num
        particleDistance = particles(i).ultraScan(); % Particle distance reading
        currPos = particleDistance; % Current particle position object
%         locations(:,i) = particles(i).getBotPos(); % Save particle location
       
        currRotProb = mean(normpdf(particleDistance,robotDist,0.2)); %Probability of robot being at current orientation
        newAngle = particles(i).getBotAng(); % Angle of particle
        
        %COMMENTING OUT THE ANGLE MAKES THE CODE WORK
        %AND PARTICLES CONVERGE REALLY QUICKLY. I DON'T UNDERSTAND WHY, BUT
        %SOMETHING ABOUT SETTING THE NEW ANGLE ISN'T WORKING. IMPLIES
        %RANDOMNESS
        for j = 1:size(particleDistance)-1 % Check all other particle orientations
            %Find the most similar orientation of reading and assume
            %correct.
            possDist = circshift(particleDistance,j); % Find possible rotation distances
            newRotProb = mean(normpdf(possDist,robotDist,0.2)); % Probability of possible rotation
            if newRotProb > currRotProb % If new rotation appears better than previous
                currPos = possDist; % Set current position distances to the rotated distances
                currRotProb = newRotProb; % Current best probability gets set to new orientation
                rotAngle = 2*pi*(j/length(particleDistance));
                newAngle = particles(i).getBotAng() + rotAngle;  % Update angle
            end
        end
        probabilities(i) = mean(normpdf(currPos,robotDist,0.2)) + dampFactor; % Mean probability of distance measures to robot
%         max(probabilities)
        angles(i) = newAngle; % Set angle %Using this as a debugger, it appears that the angles are converging!!
        angles2(i) = botSim.getBotAng();
    end
%     max(probabilities)
%     angles2
%     angles
    %% Write code for resampling your particles

    totalWeight = sum(probabilities);
    
    for i = 1:num
        probabilities(i) = probabilities(i)/totalWeight;
    end
    
    probabilities
    
    for i=1:num
        seed = rand; %Cannot exceed 1 as cumsum is 1.
        j=1;
        while seed > 0 && j < 300
            score = probabilities(j);
            seed = seed - score;
            j = j+1;
        end
        j
        particles(i).setBotPos(locations(:,j)');
        particles(i).setBotAng(angles(j)+0.1*rand); % Angle never seems to converge
    end
%     for i =1:num
%         %Currently the highest probability points are being lost. A very
%         %high probability point should immediately spawn a huge amount
%         %proportional to its size. I think the randsample function is a
%         %poor way to get around this.
%         newLoc = locations(:,randsample(1:num,1,true,probabilities')); %Maybe randsample is bonking ma code
%         particles(i).setBotPos(newLoc') % Position converges, albeit often incorrectly
%         particles(i).setBotAng(angles(i)+0.3*rand) % Angle never seems to converge
% %         fprintf('Hello world')
%     end
    
    for i =1:0.01*num
        someLoc = floor(rand*num)+1;
        particles(someLoc).randomPose(0)
        particles(someLoc).setBotAng(particles(i).getBotAng()+2*pi*rand)
    end
    

    %% Write code to check for convergence
%     	if converged == hashappend.
%             converged == 1
%         end
%     
    %% Write code to decide how to move next
    %Random walker code
    currentPos = botSim.getBotPos();
    if inpolygon(currentPos(1),currentPos(2),map(:,1),map(:,2)) ~= 1
        break
    else
        robotDist = botSim.ultraScan();
        if robotDist(1) < 5
            turn = pi + 0.1*pi*(rand*2 - 1);
            move = 2;
            botSim.turn(turn)
            botSim.move(move)
        else
            turn = 0.25*pi*(rand*2 - 1);
            move = 4*rand;
            botSim.turn(turn)
            botSim.move(move)
        end
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
        for i =1:num
            particles(i).drawBot(3); %draw particle with line length 3 and default color
        end
        drawnow;
    end
end
end
