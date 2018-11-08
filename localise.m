function [botSim] = localise(botSim,map,target)
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
particles(1).getBotPos
%% Localisation code
maxNumOfIterations = 30;
n = 0;
converged =0; %The filter has not converged yet
while(converged == 0 && n < maxNumOfIterations) %%particle filter loop
    n = n+1; %increment the current number of iterations
    botScan = botSim.ultraScan(); %get a scan from the real robot.
    
    %% Write code for updating your particles scans
    locations = zeros(2,num);
    for i = 1:num %For each particle
        locations(:,i) = particles(i).getBotPos(); % Save particle location
    end
    %% Write code for scoring your particles
    distances = botSim.ultraScan();
    probabilities = zeros(1,num);
    dampFactor = 0;
    angles = zeros(1,num);
%     particleDistance = [];
    for i = 1:num
%         size(particles(i).ultraScan())
%         size(particleDistance)
        particleDistance = particles(i).ultraScan();        
        currPos = particleDistance;
%         
%         currentPossibleRotation = norm(particleDistance - distances);
%         
        currRotProb = mean(normpdf(particleDistance,distances,0.2));
        newAngle = particles(i).getBotAng();
        
        %The issue is somewhere in this next block of code. The particles
        %are converging to something - but it isn't my robot. Probably to
        %do with the angle.
        for j = 1:size(particleDistance)-1
            %Find the most similar orientation of reading and assume
            %correct.
            possDist = circshift(particleDistance,j);
            newRotProb = mean(normpdf(possDist,distances,0.2));
            if newRotProb < currRotProb
                currPos = possDist;
                currRotProb = newRotProb;
                newAngle = particles(i).getBotAng() - 2*pi*(j/length(particleDistance));
            end
        end
        probabilities(i) = mean(normpdf(currPos,distances,0.2))+dampFactor;
        angles(i) = newAngle;
%         angles(i) = botSim.getBotAng();
    end
    %% Write code for resampling your particles

    totalWeight = sum(probabilities);
    
    for i = 1:num
        probabilities(i) = probabilities(i)/totalWeight;
    end
    
    for i=1:num
        seed = rand; %Cannot exceed 1 as cumsum is 1.
        j=1;
        while seed > 0 && j < 300
            score = probabilities(j);
            seed = seed - score;
            j = j+1;
        end
        locations(:,j)
        particles(i).setBotPos(locations(:,j)');
        particles(i).setBotAng(angles(i)+0.3*rand); % Angle never seems to converge
    end
    
%     for i =1:num
%         newLoc = locations(:,randsample(1:num,1,true,probabilities')) %Jed wrote this
% %         particles(i).setBotPos(newLoc')
% %         particles(i).setBotAng(angles(i))
%     end
    
    for i =1:0.1*num
        particles(floor(rand*num)+1).randomPose(0)
    end
    
    %         robotPose = botSim.getBotPos();
    %     for i = 1:num
    %          particles(i).setBotPos(robotPose)
    %     end
    %% Write code to check for convergence
    % 	if converged == hashappend.
    %         converged == 1
    %     end
    
    %% Write code to decide how to move next
    % here they just turn in cicles as an example
    turn = 0.5;
    move = 2;
    botSim.turn(turn); %turn the real robot.
    botSim.move(move); %move the real robot. These movements are recorded for marking
    for i =1:num %for all the particles.
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
