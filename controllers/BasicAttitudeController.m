% BasicAttitudeController.m
% Contrôleur pour moduler la vitesse et l'attitude du drone, supportant à la fois les drones multirotors et à voilure fixe

classdef BasicAttitudeController
    properties
        Drone           % Référence au drone contrôlé
    end
    
    methods
        function obj = BasicAttitudeController(drone)
            obj.Drone = drone;
        end
        
        function [velocity, climbRate] = computeControlSignal(obj, currentPos, destination, dt)
            % Calculer la direction vers la destination
            direction = destination - currentPos;
            distance = norm(direction);
            
            % Normaliser le vecteur direction si la distance est non nulle
            if distance > 0
                direction = direction / distance;
            end
            
            % Définir la vitesse en fonction du type de drone
            if strcmp(obj.Drone.Type, 'fixedwing')
                % Les drones à voilure fixe ont une contrainte de vitesse minimale et maximale
                speed = max(obj.Drone.MinSpeed, min(obj.Drone.MaxSpeed, distance / dt));
            else
                % Les drones multirotors utilisent la vitesse maximale et de croisière
                speed = min(obj.Drone.MaxSpeed, max(obj.Drone.CruiseSpeed, distance / dt));
            end
            
            % Calculer le vecteur de vitesse
            velocity = direction * speed;
            
            % Contrôler le taux de montée en fonction de la différence d'altitude
            altitudeDifference = destination(3) - currentPos(3);
            if altitudeDifference > 0
                climbRate = min(obj.Drone.MaxVarioUp, altitudeDifference / dt);
            else
                climbRate = max(obj.Drone.MaxVarioDown, altitudeDifference / dt);
            end
        end
    end
end
