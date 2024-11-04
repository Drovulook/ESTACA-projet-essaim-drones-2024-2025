% classes/MultirotorDrone.m
% Classe pour les drones multirotors avec contrôle basé sur l'attitude

classdef MultirotorDrone < DroneBase
    properties
        Destination      % Position cible actuelle
        MaxSpeed         % Vitesse maximale
        CruiseSpeed      % Vitesse de croisière
        MaxVarioUp       % Taux de montée maximal
        MaxVarioDown     % Taux de descente maximal
        MaxTurnGLoad     % Charge en G maximale lors d'un virage
        MaxTurnRate      % Taux de virage maximal [rad/s]
        Controller       % Instance du contrôleur d'attitude
    end
    
    methods
        function obj = MultirotorDrone(id, env, initialPosition, params)
            obj@DroneBase(id, env, 'multirotor', initialPosition);
            
            % Charger les paramètres depuis la structure params
            obj.MaxSpeed = params.MaxSpeed;
            obj.CruiseSpeed = params.CruiseSpeed;
            obj.MaxVarioUp = params.MaxVarioUp;
            obj.MaxVarioDown = params.MaxVarioDown;
            obj.MaxTurnGLoad = params.MaxTurnGLoad;
            obj.MaxTurnRate = params.MaxTurnRate;
            
            % Initialiser le contrôleur d'attitude
            obj.Controller = BasicAttitudeController(obj);
            obj.Destination = initialPosition;
        end
        
        function update(obj, dt)
            % Obtenir la position actuelle et calculer la vitesse et le taux de montée
            currentPos = obj.Platform.Pose(1:3);
            [velocity, climbRate] = obj.Controller.computeControlSignal(currentPos, obj.Destination, dt);
            
            % Contraindre le taux de montée
            if climbRate > obj.MaxVarioUp
                climbRate = obj.MaxVarioUp;
            elseif climbRate < obj.MaxVarioDown
                climbRate = obj.MaxVarioDown;
            end
            
            % Mettre à jour la position avec la vitesse et le taux de montée
            newPos = currentPos + [velocity(1:2); climbRate] * dt;
            obj.Platform.updatePose('Position', newPos);
        end
        
        function setDestination(obj, dest)
            obj.Destination = dest
        end
    end
end
