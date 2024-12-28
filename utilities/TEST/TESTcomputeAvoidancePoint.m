% utilities/computeAvoidancePoint.m
% Calcule un nouveau point pour éviter un obstacle

function avoidancePoint = computeAvoidancePoint(currentPos, obstaclePoint)
    % Calculer un nouveau point éloigné de l'obstacle
    direction = currentPos - obstaclePoint;
    avoidanceDistance = 10; % Distance à maintenir par rapport à l'obstacle
    avoidancePoint = obstaclePoint + avoidanceDistance * (direction / norm(direction));
end
