% classes/SwarmManager.m
% Classe pour gérer un essaim de drones

classdef SwarmManager < handle
    properties
        backend             % Référence à l'application (pour afficher un message quand il y a collision par ex)
        Drones          % Tableau de cellules contenant les objets DroneBase (drones de l'essaim)
        AliveDrones
        DeadDrones
        Environment     % Objet uavScenario représentant l'environnement de simulation

        %% Swarm PROPERTIES
        
        target_list %1 ligne par target en coordonées xyz, A changer + tard en classe pour définir niveau d'intérêt (pondération d'attraction) + mouvement
        target_history_matrix
        drones_pos_history_matrix
    end
    
    methods
        % Constructeur pour initialiser le gestionnaire d'essaim avec l'environnement
        function obj = SwarmManager(backend, env, temps) % Bien prendre l'objet env
            obj.backend = backend;
            obj.Drones = {};  % Initialiser le tableau de drones comme vide
            obj.AliveDrones = {};
            obj.DeadDrones = {};
            obj.target_list = {};
            obj.Environment = env; % Assigner l'environnement de simulation

        end
        
        % Méthode pour ajouter un drone à l'essaim
        function addDrone(obj, droneType, initialPosition)
            id = length(obj.Drones) + 1; % Déterminer un ID unique pour le nouveau drone
            load('data/params.mat', 'multirotorParams', 'fixedWingParams'); % Charger les paramètres des drones
            
            %Ici il faut changer pour essayer de faire un quadrillage de
            %départ au sol. Ici je les fait juste commencer espacés
            count = 1;
            while true
                if isempty(obj.Drones)
                    break;
                end

                if initialPosition ~= obj.Drones{count}.posState
                    break;
                end

                initialPosition(1:2) = initialPosition(1:2) + 1;

                if count == length(obj.Drones)
                    break;
                end
                count = count + 1;

            end

            % Créer le drone en fonction de son type (multirotor ou fixedwing)
            switch droneType
                case 'multirotor'
                    drone = MultirotorDrone(id, initialPosition, multirotorParams);
                case 'fixedwing'
                    drone = FixedWingDrone(id, initialPosition, fixedWingParams);
                otherwise
                    error('Type de drone inconnu.'); % Gérer l'erreur si le type n'est pas reconnu
            end
            
            obj.Drones{end+1} = drone; % Ajouter le drone au tableau des drones
            obj.AliveDrones{end+1} = drone;
            %disp(['Nombre actuel de drones : ', num2str(length(obj.Drones))]); % Afficher le nombre total de drones (commenté)
        end
    

        % Méthode pour retirer un drone de l'essaim
        function removeDrone(obj, id)   %!! à modifier pour prendre en compte AliveDrones
            % Utiliser cellfun pour obtenir un tableau des IDs des drones
            ids = cellfun(@(drone) drone.ID, obj.Drones);
            
            % Trouver l'indice correspondant à l'ID
            idx = find(ids == id, 1);

            % Vérifier si le drone est présent dans l'essaim
            if ~isempty(idx)
                obj.Drones(idx) = []; % Supprimer le drone à l'indice spécifié
                %disp(['Drone à l''indice ', num2str(id), ' a été supprimé.']); % Afficher un message de suppression (commenté)
                %env.Platforms = obj.Drones; % Mettre à jour les plateformes dans l'environnement
            else
                error('Aucun drone n''existe à cet indice.'); % Erreur si le drone n'existe pas
            end
        end

        % Méthode pour définir une destination pour un drone spécifique dans l'essaim
        function setDestination(obj, id, destination)
            % Utiliser cellfun pour obtenir un tableau des IDs des drones
            ids = cellfun(@(drone) drone.ID, obj.Drones);
            
            % Trouver l'indice correspondant à l'ID
            idx = find(ids == id, 1);
        
            % Vérifier si le drone est présent dans l'essaim
            if ~isempty(idx)
                obj.Drones{idx}.setDestination(destination); % Mettre à jour la destination du drone
            else
                error('Aucun drone n''existe à cet indice.'); % Erreur si le drone n'existe pas
            end
        end

        function update_target(obj, newTarget)
            obj.target_list = newTarget;
        end

        function Destroy_drone(obj, ind)
            drone = obj.AliveDrones{ind};
            drone.IsAlive = false;
            drone.posState(3) = 0;
            obj.AliveDrones{ind} = [];
            to_remove = cellfun(@isempty, obj.AliveDrones);  % Trouve les indices des éléments vides
            obj.AliveDrones(to_remove) = [];                % Supprime ces éléments
            %obj.AliveDrones = [obj.AliveDrones(1:ind-1), obj.AliveDrones(ind+1:end)] 
            obj.DeadDrones{end+1} = drone;

        end
    
        function check_collisions(obj, drones_pos, zones_list)
            drones_to_remove = [];
            n = length(obj.AliveDrones);
            value_to_avoid = 0;
            for i = 1:n
                i_drone =  obj.AliveDrones{i};
                for k = 1:i-1;
                    k_drone = obj.AliveDrones{k};
                    dist = sqrt((i_drone.posState(1) - k_drone.posState(1))^2 ...
                              + (i_drone.posState(2) - k_drone.posState(2))^2 ...
                              + (i_drone.posState(3) - k_drone.posState(3))^2);
                    if dist < i_drone.Radius + k_drone.Radius;
                        drones_to_remove = [drones_to_remove, i, k];

                        obj.backend.OnDronesCollision(i_drone.ID, k_drone.ID);
                    end
                end
            end
            drones_to_remove = unique(drones_to_remove);
            for i = numel(drones_to_remove):-1:1  % Supprimer de la fin pour éviter les problèmes d'indice
                obj.Destroy_drone(drones_to_remove(i));  % Appel de Destroy_drone pour retirer le drone
            end
            if isempty(drones_to_remove)
                obj.backend.OnDronesNbUpdate();
            end
        end

        function update_speeds(obj, dt, r, swarm_weights, weights, target_weights, sat)
            % Compute le vecteur vitesse t+1 du drone en fonction de l'influence de l'essaim, de sa vitesse, des targets et des zones d'exclusion 
            % Et gère les collisions éventuelles
        
            % r est la liste de rayon (répulsion, évitement, attraction_max), 
            % weights la liste de pondération (répulsion, attraction, target, évitement),
            % répulsion pour les drones, évitement pour le terrain,
            % attraction max, distance d'attraction maximum (bruit de communication)

            zones = obj.Environment.get_zones_pos_weights();

            n = length(obj.Drones);
            posStateMatrix = zeros(n,3);
            speedStateMatrix = zeros (n,3);

            for i = 1:n
                drone = obj.Drones{i};
                if drone.IsAlive;
                    obj.Drones{i}.update_pos(dt); % On update les drones à leur nouvelle position en fonction du dernier vecteur vitesse computé
                end
                posStateMatrix(i,:) = obj.Drones{i}.posState; 
                speedStateMatrix(i,:) = obj.Drones{i}.speedState;
            end

            obj.check_collisions(posStateMatrix, zones);

            obj.drones_pos_history_matrix(:,:,size(obj.drones_pos_history_matrix,3)+1) = posStateMatrix; % historique des positions pour le temps diff
            
            %% CALCUL DES VOISINS
            [neighborI, nnmax] = VoroiNeighbor(posStateMatrix, n); % Utils.Algo
            
            %% SWARM INFLUENCE
            swarmInfluence = swarm_pond(posStateMatrix, speedStateMatrix, neighborI, n, nnmax, swarm_weights, r); % Utils.Algo
            
            %% SPEED INFLUENCE
            speedInfluence = speed_pond(speedStateMatrix); % Utils.Algo

            %% TARGET INFLUENCE
            targetInfluence = target_pond(obj.target_list, posStateMatrix, target_weights); % Utils.Algo
            
            %% Calcul des zones d'évitement 
            zones = obj.Environment.get_zones_pos_weights();
            avoidInfluence = avoid_pond(posStateMatrix, zones);

            %% Maintentant, pour chaque drone, on fait la pondération des influeneces swarm/target/speed et on les somme

            newSpeedMatrix = whole_pond(swarmInfluence, speedInfluence, targetInfluence, avoidInfluence, weights); % Utils.Algo

            %temp
            newSpeedMatrix(newSpeedMatrix < sat(1)) = sat(1);
            newSpeedMatrix(newSpeedMatrix > sat(2)) = sat(2);
            
            %manque système de saturation conique
            for i = 1:length(obj.AliveDrones)
                drone = obj.AliveDrones{i};
                if drone.IsAlive
                    drone.speedState = newSpeedMatrix(i,:);
                end
            end
          
  
        end
    end
end





