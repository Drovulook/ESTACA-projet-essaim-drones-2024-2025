% classes/DroneBase.m
% Classe de base abstraite pour tous les types de drones

classdef (Abstract) DroneBase < handle
    % DroneBase : Classe de base abstraite pour les drones
    
    properties
        ID              % Identifiant unique du drone
        Type            % Type de drone : 'multirotor' ou 'fixedwing'
        Platform        % Objet uavPlatform représentant le drone dans l'environnement
        Environment     % Objet uavScenario représentant l'environnement du drone
    end
    
    methods
        % Constructeur pour initialiser un drone avec un ID, un environnement, un type et une position initiale
        function obj = DroneBase(id, env, type, initialPosition)
            obj.ID = id; % Assigner l'identifiant unique
            obj.Type = type; % Assigner le type de drone
            obj.Environment = env; % Associer l'environnement de simulation
            
            % Générer un nom unique pour le uavPlatform en utilisant le type et l'ID du drone
            uniqueName = sprintf('%sDrone%d_%s', upper(type(1)), id, datestr(now, 'HHMMSSFFF'));
            
            % Créer l'objet uavPlatform représentant le drone avec une couleur rouge
            obj.Platform = uavPlatform(uniqueName, env, ...
                "ReferenceFrame", "NED", ...
                "InitialPosition", initialPosition, ...
                "InitialOrientation", quaternion([0, 0, 0], 'eulerd', 'ZYX', 'frame'));
            
            % Ajouter un maillage 3D pour représenter le drone visuellement, avec une couleur rouge
            % Utilise une forme de cylindre pour symboliser le corps du drone
            addMesh(obj.Environment, "cylinder", ...
                {[initialPosition(1) initialPosition(2) 3], [initialPosition(3) initialPosition(3)+3]}, ...
                [1, 0, 0]);  % RGB pour rouge
        end
    end
    
    methods (Abstract)
        % Méthode abstraite de mise à jour du drone, à implémenter dans les sous-classes
        update(obj, dt)
    end
end
