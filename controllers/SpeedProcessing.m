function [newSpeedX, newSpeedY, newSpeedZ] = SpeedProcessing(drone, i, newSpeedMatrix, dt)
    tholdDesiredVectorV = 0.05;
    tholdDesiredVectorH = 0.025;
    
    nClimb = 1; % a changer par une valeur d'acceleration modulable en TR
    nTurn = 5;
    g = 9.81;
    targetSpeed = 40;
    aProp = 8; % m/s2
    aDecel = 5; % m/s2
    runwayHeading_degrees = 0;
    runwayHeading_radians = deg2rad(runwayHeading_degrees);
    
    tempSpeedState = newSpeedMatrix(i,:);
    desiredVectorX = tempSpeedState(1);
    desiredVectorY = tempSpeedState(2);
    desiredVectorZ = tempSpeedState(3);
    
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
        previousHeading_radians = runwayHeading_radians;
    else
        if previousTotalSpeed < targetSpeed
            newTotalSpeed = previousTotalSpeed + aProp * dt;
        elseif previousTotalSpeed > targetSpeed
            newTotalSpeed = previousTotalSpeed - aDecel * dt;
        else
            newTotalSpeed = previousTotalSpeed;
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
    
        if angdiff(desiredHeading_radians, previousHeading_radians) < 0
            % turnVelocity = g * sqrt((dampingFactorH * nTurn)^2 - 1) / newTotalSpeed;
            turnVelocity = g * sqrt(nTurn^2 - 1) / newTotalSpeed;
        elseif angdiff(desiredHeading_radians, previousHeading_radians) > 0
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

