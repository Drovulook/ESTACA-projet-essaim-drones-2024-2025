% classes/SwarmManager.m
% Classe pour gérer un essaim de drones

classdef SwarmManager < handle
    properties
        Drones          % Tableau de cellules contenant les objets DroneBase (drones de l'essaim)
        Environment     % Objet uavScenario représentant l'environnement de simulation

        %% Swarm PROPERTIES
        
        target_list %1 ligne par target en coordonées xyz, A changer + tard en classe pour définir niveau d'intérêt (pondération d'attraction) + mouvement
        target_history_matrix
        drones_pos_history_matrix
    end
    
    methods
        % Constructeur pour initialiser le gestionnaire d'essaim avec l'environnement
        function obj = SwarmManager(env, temps) % Bien prendre l'objet env
            obj.Drones = {};  % Initialiser le tableau de drones comme vide
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
            %disp(['Nombre actuel de drones : ', num2str(length(obj.Drones))]); % Afficher le nombre total de drones (commenté)
        end
        
        % Méthode pour retirer un drone de l'essaim
        function removeDrone(obj, id)
            % Utiliser cellfun pour obtenir un tableau des IDs des drones
            ids = cellfun(@(drone) drone.ID, obj.Drones);
            
            % Trouver l'indice correspondant à l'ID
            idx = find(ids == id, 1);

            % Vérifier si le drone est présent dans l'essaim
            if ~isempty(idx)
                obj.Drones(idx) = []; % Supprimer le drone à l'indice spécifié
                %disp(['Drone à l''indice ', num2str(id), ' a été supprimé.']); % Afficher un message de suppression (commenté)
                env.Platforms = obj.Drones; % Mettre à jour les plateformes dans l'environnement
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

    
        function update_speed(obj, dt, r, swarm_weights, weights, target_weights, sat)
        % Compute le vecteur vitesse t+1 du drone en fonction de l'influence de l'essaim, de sa vitesse, des targets et des zones d'exclusion 
        % r est la liste de rayon (répulsion, évitement, attraction_max), 
        % w la liste de pondération (répulsion, attraction, évitement),
        % répulsion pour les drones, évitement pour le terrain,
        % attraction max, distance d'attraction maximum (bruit de communication)

            n = length(obj.Drones);
            posStateMatrix = zeros(n,3);
            speedStateMatrix = zeros (n,3);

            for i = 1:n
                obj.Drones{i}.update_pos(dt); % On update les drones à leur nouvelle position en fonction du dernier vecteur vitesse comupté
                posStateMatrix(i,:) = obj.Drones{i}.posState; 
                speedStateMatrix(i,:) = obj.Drones{i}.speedState;
            end

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
            %%%%%%%%%%%%%%%%%%%%%%
            % WIP WIP WIP WIP WIP
            %%%%%%%%%%%%%%%%%%%%%%

            %Le besoin c'est une matrice qui renvoie coords + diamètre
            %sphère (à mettre dans Environnement)
            zones_object_list = obj.Environment.get_zones_pos_weights();

            for i = 1:length(zones_object_list)
                
                zones_object_list(i).CenterPosition
                zones_object_list(i).Dimensions

            end

            %% Maintentant, pour chaque drone, on fait la pondération des influeneces swarm/target/speed et on les somme

            newSpeedMatrix = whole_pond(swarmInfluence, speedInfluence, targetInfluence, weights); % Utils.Algo

            %temp
            newSpeedMatrix(newSpeedMatrix < sat(1)) = sat(1);
            newSpeedMatrix(newSpeedMatrix > sat(2)) = sat(2);
            
            %manque système de saturation conique
            for i = 1:n
                obj.Drones{i}.speedState = newSpeedMatrix(i,:);
            end
          
  
        end
    end
end



