function [newSpeedX, newSpeedY, newSpeedZ] = SpeedProcessing(drone, i, desiredVector, dt)
    
    if contains(drone.phase, 'stand-by') | contains(drone.phase, 'reload')
        newSpeedX = 0;
        newSpeedY = 0;
        newSpeedZ = 0;
        return
    end

    tholdDesiredVectorV = 0.05;
    tholdDesiredVectorH = 0.025;
    
    nClimb = 1; % a changer par une valeur d'acceleration modulable en TR
    nTurn = 5;
    g = 9.81;
    targetSpeed = 40;
    aProp = 8; % m/s2
    aDecel = 5; % m/s2
    runwayHeading_degrees = drone.TOheading;
    runwayHeading_radians = deg2rad(runwayHeading_degrees);
    
    tempSpeedState = desiredVector(i,:);
    desiredVectorX = tempSpeedState(1);
    desiredVectorY = tempSpeedState(2);
    desiredVectorZ = tempSpeedState(3);
    if strcmp(drone.Type,'multirotor')

        newSpeedX = desiredVectorX*drone.CruiseSpeed;
        newSpeedY = desiredVectorY*drone.CruiseSpeed;
        newSpeedZ = desiredVectorZ*drone.CruiseSpeed;
        newTotalSpeed = sqrt(newSpeedX^2 + newSpeedY^2 + newSpeedZ^2);
                     
        if newTotalSpeed > drone.MaxSpeed
            desiredVectorX = (desiredVectorX/newTotalSpeed)*drone.MaxSpeed;
            desiredVectorY = (desiredVectorY/newTotalSpeed)*drone.MaxSpeed;
            desiredVectorZ = (desiredVectorZ/newTotalSpeed)*drone.MaxSpeed;

            newSpeedX = desiredVectorX*drone.CruiseSpeed;
            newSpeedY = desiredVectorY*drone.CruiseSpeed;
            newSpeedZ = desiredVectorZ*drone.CruiseSpeed;
            newTotalSpeed = sqrt(newSpeedX^2 + newSpeedY^2 + newSpeedZ^2);
        end
        return
    end
    
    previousSpeedState = drone.speedState;
    previousSpeedState(isnan(previousSpeedState)) = 0;
    previousSpeedX = previousSpeedState(1);
    previousSpeedY = previousSpeedState(2);
    previousSpeedZ = previousSpeedState(3);
    previousTotalSpeed = sqrt(previousSpeedX^2 + previousSpeedY^2 + previousSpeedZ^2);

    if previousTotalSpeed < drone.MinSpeed
        newSpeedZ = 0;
        turnVelocity = 0;
        newTotalSpeed = previousTotalSpeed + aProp * dt;
        previousHeading_radians = atan2(previousSpeedY, previousSpeedX);

        newSpeedX = newTotalSpeed * cos(previousHeading_radians);
        newSpeedY = newTotalSpeed * sin(previousHeading_radians);
        return

    else

        maxSpeed = drone.MaxSpeed;
        if contains(drone.phase, 'landing')
            maxSpeed = drone.MaxSpeed/5;
        end
        %%
    
        dampingFactorV = sqrt(max(0,min(1,  0.3 + abs(desiredVectorZ) / tholdDesiredVectorV)));
        % newSpeedZ = max(drone.MaxDescentRate, min(drone.MaxClimbRate, sign(desiredVectorZ) * dampingFactorV * (previousSpeedZ + sign(desiredVectorZ) * nClimb * g * dt)));
        
        if desiredVectorZ > 0                                     
            newSpeedZ = dampingFactorV * (previousSpeedZ + nClimb * g * dt);
            newSpeedZ = min(newSpeedZ, drone.MaxClimbRate);
        elseif desiredVectorZ < 0
            newSpeedZ = dampingFactorV * (previousSpeedZ - nClimb * g * dt);
            newSpeedZ = max(newSpeedZ, drone.MaxDescentRate);
        else
            newSpeedZ = 0;
        end
    
        previousHeading_radians = atan2(previousSpeedY, previousSpeedX);
        desiredHeading_radians = atan2(desiredVectorY, desiredVectorX);
    
        % dampingFactorH = sqrt(max(0,min(1,  0.3 + abs(desiredVectorX ^ 2 + desiredVectorY ^ 2) / tholdDesiredVectorH)));
    
        deltaAngle = angdiff(desiredHeading_radians, previousHeading_radians);
        
        if abs(deltaAngle) > deg2rad(30)
            newTotalSpeed = previousTotalSpeed - aDecel * dt;
        else
            if previousTotalSpeed < maxSpeed
                newTotalSpeed = previousTotalSpeed + aProp * dt;
            elseif previousTotalSpeed > maxSpeed
                newTotalSpeed = previousTotalSpeed - aDecel * dt;
            else
                newTotalSpeed = previousTotalSpeed;
            end
        end
        

        if deltaAngle < 0
            % turnVelocity = g * sqrt((dampingFactorH * nTurn)^2 - 1) / newTotalSpeed;
            turnVelocity = g * sqrt(nTurn^2 - 1) / newTotalSpeed;
        elseif deltaAngle > 0
            % turnVelocity = - (g * sqrt((dampingFactorH * nTurn)^2 - 1) / newTotalSpeed);
            turnVelocity = - (g * sqrt(nTurn^2 - 1) / newTotalSpeed);
        else
            turnVelocity = 0.0001;
        end

    end
    
    newHeading_radians = previousHeading_radians + turnVelocity * dt;
    
    newSpeedH = sqrt(newTotalSpeed^2 - newSpeedZ^2);
    newSpeedX = newSpeedH * cos(newHeading_radians);
    newSpeedY = newSpeedH * sin(newHeading_radians);
    
    

end

