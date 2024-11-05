% classes/DroneBase.m
% Classe de base abstraite pour tous les types de drones

classdef (Abstract) DroneBase < handle
    % DroneBase : Classe de base abstraite pour les drones
    
    properties
        ID              % Identifiant unique
        Type            % 'multirotor' ou 'fixedwing'
        Platform        % Objet uavPlatform
        Environment     % Objet uavScenario
    end
    
    methods
        function obj = DroneBase(id, env, type, initialPosition)
            obj.ID = id;
            obj.Type = type;
            obj.Environment = env;
            
            % Génère un nom unique pour le uavPlatform en utilisant le type et l'ID
            uniqueName = sprintf('%sDrone%d_%s', upper(type(1)), id, datestr(now, 'HHMMSSFFF'));
            
            % Crée le uavPlatform avec une couleur rouge
            obj.Platform = uavPlatform(uniqueName, env, ...
                "ReferenceFrame", "NED", ...
                "InitialPosition", initialPosition, ...
                "InitialOrientation", quaternion([0, 0, 0], 'eulerd', 'ZYX', 'frame'));
            
            % Ajoute un maillage 3D pour représenter le drone, avec une couleur rouge
            addMesh(obj.Environment, "cylinder", {[initialPosition(1) initialPosition(2) 3], [initialPosition(3) initialPosition(3)+3]}, [1, 0, 0]);  % RGB pour rouge
        end
    end
    
    methods (Abstract)
        update(obj, dt) % Méthode abstraite de mise à jour à implémenter dans les sous-classes
    end
end
