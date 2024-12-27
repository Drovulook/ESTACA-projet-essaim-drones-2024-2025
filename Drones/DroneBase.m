% classes/DroneBase.m
% Classe de base abstraite pour tous les types de drones

classdef (Abstract) DroneBase < handle
    % DroneBase : Classe de base abstraite pour les drones
    
    properties
        ID              % Identifiant unique du drone
        Type            % Type de drone : 'multirotor' ou 'fixedwing' 
        posState        % Vecteur de position à tn (1,3) [x, y, z]
        speedState = [0 0 0]     % Vecteur de vitesse à tn (1,3) [r, teta, phi]
        posLog
        speedLog
        flightTime = 0  % Temps de vol en sec
        Target          % Coordonnées de la cible
        Radius
        IsAlive = true;
        mode_Follow_waypoint = false; % False = follow target / True = follow waypoints
        
        mass
        

        powerLog            % Puissances consommées à chaque pas (1,n)
        mean_consumption    % Puissance moyenne consommée (pour l'optimisation)

        %energie a bord
        maxCapacity         % si batterie
        batteryNominalVoltage       % si batterie
        k_peukert           % si batterie (cte de Peukert)
        tankVolume          % si carburant
        yield=0.9               % pour les deux
        remainingCapacity   % en Wh
        autonomy            % en h
    end
    
    methods
        % Constructeur pour initialiser un drone avec un ID, un environnement, un type et une position initiale
        function obj = DroneBase(id, type, initialPos)
            obj.ID = id; % Assigner l'identifiant unique
            obj.Type = type; % Assigner le type de drone
            
            %dimensionnement drone
            % !! ne pas retirer ces valeurs, elles ne peuvent être nulles
            obj.batteryNominalVoltage=22.2;
            obj.maxCapacity=200;
            obj.remainingCapacity=obj.maxCapacity;
            obj.mass=50;
            obj.k_peukert=1.2;

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

        function [autonomy] = getAutonomy(obj); % donne l'autonomie restante en fct des logs du vol
            autonomy=obj.autonomy;
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
    
    methods
        function update_pos(obj, dt) %Update la position du drone en fct de sa vitesse
            dpos = dt * obj.speedState; %on peut ajouter du noise
            obj.posState = obj.posState + dpos;

            % Logging de l'état du drone
            if isempty(obj.posLog)
                obj.posLog = obj.posState; % Initialise le log si vide
            else
                obj.posLog = [obj.posLog; obj.posState]; % Ajoute la nouvelle position à l'historique
            end
        
            if isempty(obj.speedLog)
                obj.speedLog = obj.speedState; % Initialise le log si vide
            else
                obj.speedLog = [obj.speedLog; obj.speedState]; % Ajoute la nouvelle vitesse à l'historique
            end


        end
    end
end
