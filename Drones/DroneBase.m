% classes/DroneBase.m
% Classe de base abstraite pour tous les types de drones

classdef (Abstract) DroneBase < handle
    % DroneBase : Classe de base abstraite pour les drones
    
    properties
        % ---------------- Existing Properties ----------------
        ID                   % Identifiant unique du drone
        Name
        Model
        Type                 % 'multirotor' ou 'fixedwing'
        posState             % [x, y, z]
        speedState = [0 0 0] % [vx, vy, vz] or [r,theta,phi], etc.
        posLog
        speedLog
        flightTime = 0
        targetGroup = 1
        IsAlive = true
        mode_Follow_waypoint = false
        Waypoints
        CurrentWaypoint = 1
        hasCommunicated = 0
        mass
        powerLog
        mean_consumption
        
        % ---------------- Battery/Power/Autonomy (existing) ---------------
        maxCapacity          % e.g. in Wh (if battery)
        batteryNominalVoltage% e.g. in V
        k_peukert = 1.2
        tankVolume           % e.g. in liters (if fuel)
        yield = 0.9          % generic efficiency factor
        remainingCapacity    % in Wh
        autonomy             % in hours (existing property)
        
        % ---------------- NEW / CSV-derived (added) -----------------------
        % (You can store these so that child classes won't need to declare them)
        AutonomyMins         % e.g. from CSV
        ReloadMins           % e.g. from CSV
        NominalCapacity      % if you want to store CSV's "NominalCapacity" directly
        NominalVoltage       % from CSV's "NominalVoltage"
    end
    
    methods
        % Constructeur pour initialiser un drone avec un ID, un environnement, un type et une position initiale
        function obj = DroneBase(id, type, initialPos)
            obj.ID   = id;
            obj.Type = type;
            
            % Some defaults:
            obj.batteryNominalVoltage = 22.2;  % e.g. 6S LiPo
            obj.maxCapacity           = 200;
            obj.remainingCapacity     = obj.maxCapacity;
            obj.mass                  = 50;
            obj.k_peukert             = 1.2;
            
            % Initialize position
            obj.posState = initialPos;
            
            % Example: if you want "autonomy" to start the same as AutonomyMins,
            % you can do so below (once you assign AutonomyMins). But that’s up to you.
            
            % Generate a unique name if desired (though not stored in a property):
            % uniqueName = sprintf('%sDrone%d_%s', upper(type(1)), id, datestr(now, 'HHMMSSFFF'));
        end
        
        % -------------- Example methods -----------------------------------
        function [distance] = closestDrone(obj, swarm)
            distance = 100; % calcul à implémenter
        end
           
        function [score] = obsScore(obj, target, env)
            score = 100; % calcul à implémenter
        end

        function [distance] = closestEnv(obj, target, env)
            distance = 100; % calcul à implémenter
        end

        function [autonomy] = getAutonomy(obj)
            autonomy = obj.autonomy;  % e.g. in hours
        end

        function [target] = getTarget(obj, swarm)
            target = swarm.targets(obj.targetGroup, :);
        end

        function setTargetGroup(obj, n)
            obj.targetGroup = n;
        end

        function crashDrone(obj)
            obj.IsAlive = false;
        end
        
        function compute_autonomy(obj,dt)
        end
    end
    
    methods
        function logData(obj)
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


        function update_pos(obj, dt) %Update la position du drone en fct de sa vitesse
            if obj.IsAlive == true
                dpos = dt * obj.speedState; %on peut ajouter du noise
                obj.posState = obj.posState + dpos;
                
            else
                obj.posState(3) = 0;
                obj.speedState = [0 0 0];
            end
            obj.logData
            if(size(obj.speedLog,1)>2)
                obj.compute_autonomy(dt); % utilise les logData
            end
        end


        function CycleWaypoint(obj)
            obj.CurrentWaypoint = obj.CurrentWaypoint + 1 ;
            if obj.CurrentWaypoint > size(obj.Waypoints,1)
                obj.CurrentWaypoint = 1;
            end
        end


    end
end
