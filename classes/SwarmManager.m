% classes/SwarmManager.m
% Gère l'essaim de drones
classdef SwarmManager < handle
    properties
        Drones          % Tableau de cellules contenant les objets DroneBase
        Environment     % Objet uavScenario
    end
    
    methods
        function obj = SwarmManager(env)
            obj.Drones = {};  % Initialiser comme un tableau de cellules vide
            obj.Environment = env;
        end
        
        function addDrone(obj, droneType, initialPosition)
            id = length(obj.Drones) + 1;
            load('data/params.mat', 'multirotorParams', 'fixedWingParams');
            
            switch droneType
                case 'multirotor'
                    drone = MultirotorDrone(id, obj.Environment, initialPosition, multirotorParams);
                case 'fixedwing'
                    drone = FixedWingDrone(id, obj.Environment, initialPosition, fixedWingParams);
                otherwise
                    error('Type de drone inconnu.');
            end
            
            obj.Drones{end+1} = drone;
            disp(['Nombre actuel de drones : ', num2str(length(obj.Drones))]);
        end
        
        function removeDrone(obj, id)
            % Use cellfun to create an array of IDs
            ids = cellfun(@(drone) drone.ID, obj.Drones);
            
            % Find the index of the matching ID
            idx = find(ids == id, 1);

            % Vérifier si le drone est présent dans l'essaim
            if ~isempty(idx)
                obj.Drones(idx) = []; % Supprimer le drone à l'indice spécifié
                disp(['Drone à l''indice ', num2str(id), ' a été supprimé.']);
                env.Platforms = obj.Drones;
            else
                error('Aucun drone n''existe à cet indice.');
            end
        end

        function setDestination(obj, id, destination)
            % Use cellfun to create an array of IDs
            ids = cellfun(@(drone) drone.ID, obj.Drones)
            
            % Find the index of the matching ID
            idx = find(ids == id, 1)
        
            % Vérifier si le drone est présent dans l'essaim
            if ~isempty(idx)
                obj.Drones{idx}.setDestination(destination);
            else
                error('Aucun drone n''existe à cet indice.');
            end
        end

        function update(obj, dt)
            for i = 1:length(obj.Drones)
                obj.Drones{i}.update(dt);
            end
        end
    end
end
