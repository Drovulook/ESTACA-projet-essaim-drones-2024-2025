% classes/DroneBase.m
% Classe de base abstraite pour tous les types de drones

classdef (Abstract) DroneBase < handle
    % DroneBase : Classe de base abstraite pour les drones
    
    properties
        ID              % Identifiant unique du drone
        Type            % Type de drone : 'multirotor' ou 'fixedwing' 
        posState        % Vecteur de position à tn (1,3) [x, y, z]
        speedState      % Vecteur de vitesse à tn (1,3) [r, teta, phi]
        flightTime = 0  % Temps de vol en sec
        Target          % Coordonnées de la cible

    end
    
    methods
        % Constructeur pour initialiser un drone avec un ID, un environnement, un type et une position initiale
        function obj = DroneBase(id, type, initialPos)
            obj.ID = id; % Assigner l'identifiant unique
            obj.Type = type; % Assigner le type de drone
            

            % Générer un nom unique en utilisant le type et l'ID du drone
            uniqueName = sprintf('%sDrone%d_%s', upper(type(1)), id, datestr(now, 'HHMMSSFFF'));
        end

        function [distance] = closestDrone(obj, swarm); % Donne la distance au drone le plus proche
            distance = 100 %calcul a implémenter
        end
           
        function [score] = obsScore(obj, target, env); %Donne le score d'observation du drone
            score = 100 %calcul a implémenter
        end

        function [distance] = closestEnv(obj, target, env); %donne la distance à l'objet de l'env le plus proche
            distance = 100 %calcul a implémenter
        end

        function [autonomy] = autonomy(obj); % donne l'autonomie restante en fct de flightTime
            autonomy = 100 %calcul a implémenter
        end

        function [Vx Vy Vz] = velocityNorm(obj); % Normalise la vitesse
            r = obj.speedState(1,1)
            teta = obj.speedState(1,2)
            phi = obj.speedState(1,3)

            Vx = r*sin(phi)*cos(teta)
            Vy = r*sin(phi)*sin(teta)
            Vz = r*cos(phi)
        end
        
    end
    
    methods (Abstract)
        function update_pos(obj, dt) %Update la position du drone en fct de sa vitesse
            dpos = dt*speedState %on peut ajouter du noise
            posState = posState + dpos 

        end
    end
end
