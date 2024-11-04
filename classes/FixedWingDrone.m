% classes/FixedWingDrone.m
% Classe pour les drones à voilure fixe avec un contrôle de base basé sur l'attitude

classdef FixedWingDrone < DroneBase
    properties
        Destination      % Position cible actuelle
        MaxSpeed         % Vitesse maximale
        MinSpeed         % Vitesse minimale (vitesse de décrochage)
        MaxClimbRate     % Taux de montée maximal
        MaxDescentRate   % Taux de descente maximal
        Controller       % Instance de contrôleur d'attitude de base
    end
    
    methods
        function obj = FixedWingDrone(id, env, initialPosition, params)
            obj@DroneBase(id, env, 'fixedwing', initialPosition);
            
            % Charger les paramètres depuis la structure params
            obj.MaxSpeed = params.MaxSpeed;
            obj.MinSpeed = params.MinSpeed;
            obj.MaxClimbRate = params.MaxClimbRate;
            obj.MaxDescentRate = params.MaxDescentRate;
            
            % Initialiser le contrôleur d'attitude
            obj.Controller = BasicAttitudeController(obj);
            obj.Destination = initialPosition;
        end
        
        function update(obj, dt)
            % Obtenir la position actuelle et calculer la vitesse et le taux de montée
            currentPos = obj.Platform.Pose(1:3);
            [velocity, climbRate] = obj.Controller.computeControlSignal(currentPos, obj.Destination, dt);
            
            % Contraindre le taux de montée pour le drone à voilure fixe
            if climbRate > obj.MaxClimbRate
                climbRate = obj.MaxClimbRate;
            elseif climbRate < obj.MaxDescentRate
                climbRate = obj.MaxDescentRate;
            end
            
            % Mettre à jour la position avec la vitesse et le taux de montée
            newPos = currentPos + [velocity(1:2); climbRate] * dt;
            obj.Platform.updatePose('Position', newPos);
        end
        
        function setDestination(obj, dest)
            obj.Destination = dest;
        end
    end
end
