% classes/FixedWingDrone.m
% Classe pour les drones à voilure fixe avec un contrôle de base basé sur l'attitude

classdef FixedWingDrone < DroneBase & handle
    properties
        Destination      % Position cible actuelle du drone
        CurrentPos       % Position physique actuelle du drone
        MaxSpeed         % Vitesse maximale du drone
        MinSpeed         % Vitesse minimale du drone (vitesse de décrochage)
        MaxClimbRate     % Taux de montée maximal autorisé
        MaxDescentRate   % Taux de descente maximal autorisé
        Controller       % Instance de contrôleur d'attitude de base
    end
    
    methods
        % Constructeur pour initialiser un drone à voilure fixe avec ses paramètres
        function obj = FixedWingDrone(id, env, initialPosition, params)
            obj@DroneBase(id, env, 'fixedwing', initialPosition); % Appel du constructeur de la classe de base
            
            % Charger les paramètres depuis la structure params
            obj.MaxSpeed = params.MaxSpeed; % Initialiser la vitesse maximale
            obj.MinSpeed = params.MinSpeed; % Initialiser la vitesse minimale
            obj.MaxClimbRate = params.MaxClimbRate; % Initialiser le taux de montée maximal
            obj.MaxDescentRate = params.MaxDescentRate; % Initialiser le taux de descente maximal
            
            % Initialiser le contrôleur d'attitude de base
            obj.Controller = BasicAttitudeController(obj);
            obj.CurrentPos = initialPosition; % Définir la position actuelle
            obj.Destination = initialPosition; % Définir la destination initiale comme la position de départ
        end

        % Méthode pour définir une nouvelle destination pour le drone
        function setDestination(obj, dest)
            obj.Destination = dest; % Mettre à jour la position cible
        end
        
        % Méthode de mise à jour pour recalculer la position du drone
        function update(obj, dt)
            % Obtenir la position actuelle et calculer la vitesse et le taux de montée
            currentPos = obj.Platform.Pose(1:3); % Position actuelle du drone
            [velocity, climbRate] = obj.Controller.computeControlSignal(currentPos, obj.Destination, dt);
            
            % Contraindre le taux de montée pour respecter les limites du drone à voilure fixe
            if climbRate > obj.MaxClimbRate
                climbRate = obj.MaxClimbRate;
            elseif climbRate < obj.MaxDescentRate
                climbRate = obj.MaxDescentRate;
            end
            
            % Mettre à jour la position en appliquant la vitesse et le taux de montée
            newPos = currentPos + [velocity(1:2); climbRate] * dt;
            obj.Platform.updatePose('Position', newPos); % Appliquer la nouvelle position au drone
        end
    end
end
