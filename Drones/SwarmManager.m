% classes/SwarmManager.m
% Classe pour gérer un essaim de drones

classdef SwarmManager < handle
    properties
        backend             % Référence à l'application (pour afficher un message quand il y a collision par ex)
        settings
        Environment     % Objet uavScenario représentant l'environnement de simulation

        %% Swarm PROPERTIES

        Drones % Liste d'objet Drones_Base
        target % target en coordonées xyz
        drones_pos_history_matrix % Matrice 3 dim de l'historique des positions
        communicationFrequency % Hz, le nombre de fois par itération que les drones peuvent communiquer. Si 1 Hz, 1 drone seulement communique par itération
        communicationMatrix = zeros(0,0,0) % La matrice de communication utilisée par les drones pour se repérer entre eux

        %% Comportement essaim
        threshold_radius = 15 % Distance de trigger des waypoint cycliques
        altitude_min = 10 % Altitude min des drones
        dist_target_min = 20 % Distance minimal du drone à la target
        orbit_radius = 60 % Distance des fixedwing à la cible à partir de laquelle ils orbitent

        r = [30 60 100]  % Rayons Répulsion, attraction au sein de l'essaim
        swarm_weights = [1.4 0.8]; % Pondérations répulsion, attraction au sein de l'essaim
        weights = [0.5 1.2 1 10] / 10; % pondération essaim, inertie, target, évitement
        


    end
    
    properties (Dependent)
        AliveDrones
        FixedWing
        MultiRotor
    end

    methods
        % Constructeur pour initialiser le gestionnaire d'essaim avec l'environnement
        function obj = SwarmManager(env, temps) % Bien prendre l'objet env
            
            obj.Drones = {};  % Initialiser le tableau de drones comme vide
            obj.target = zeros(0,3);
            obj.Environment = env; % Assigner l'environnement de simulation

        end

        function update_backend(obj, backend)
            obj.backend = backend;
            obj.settings = backend.settings;
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
    
        function alive = get.AliveDrones(obj) 
            alive = {};
            for idx = 1:length(obj.Drones)
                if obj.Drones{idx}.IsAlive == 1
                    alive{end + 1} = obj.Drones{idx};
                end
            end
        end

        function fixedwing = get.FixedWing(obj) 
            fixedwing = {};
            for idx = 1:length(obj.Drones)
                if strcmp(obj.Drones{idx}.Type,'fixedwing')
                    fixedwing{end + 1} = obj.Drones{idx};
                end
            end
        end

        function multirotor = get.MultiRotor(obj) 
            multirotor = {};
            for idx = 1:length(obj.Drones)
                if strcmp(obj.Drones{idx}.Type,'multirotor')
                    multirotor{end + 1} = obj.Drones{idx};
                end
            end
        end


        function resetCommunications(obj)
            for idx = 1:length(obj.Drones)
                obj.Drones{idx}.hasCommunicated = 0;
            end
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
        
        function update_target(obj, newTarget)
            obj.target = newTarget;
        end


        function Destroy_drone(obj, ind)
            drone = obj.AliveDrones{ind};
            drone.IsAlive = false;
            drone.posState(3) = 0;
            obj.AliveDrones{ind} = [];
            to_remove = cellfun(@isempty, obj.AliveDrones);  % Trouve les indices des éléments vides
            obj.AliveDrones{to_remove} = [];                % Supprime ces éléments
            %obj.AliveDrones = [obj.AliveDrones(1:ind-1), obj.AliveDrones(ind+1:end)] 


        end
    
        function check_collisions(obj, zones_list)
            drones_to_remove = [];
            n = length(obj.AliveDrones);
            for i = 1:n
                i_drone =  obj.AliveDrones{i};
                for k = 1:i-1
                    k_drone = obj.AliveDrones{k};
                    dist = sqrt((i_drone.posState(1) - k_drone.posState(1))^2 ...
                              + (i_drone.posState(2) - k_drone.posState(2))^2 ...
                              + (i_drone.posState(3) - k_drone.posState(3))^2);
                    if dist < i_drone.Radius + k_drone.Radius
                        drones_to_remove = [drones_to_remove, i, k];

                        %obj.backend.OnDronesCollision(i_drone.ID, k_drone.ID);
                    end
                end
            end

            drones_to_remove = unique(drones_to_remove);
            for i = numel(drones_to_remove):-1:1  % Supprimer de la fin pour éviter les problèmes d'indice
                obj.Destroy_drone(drones_to_remove(i));  % Appel de Destroy_drone pour retirer le drone
            end
            if isempty(drones_to_remove)
                %obj.backend.OnDronesNbUpdate();
            end
        end


    

        function update_speeds(obj, dt)

            % Compute le vecteur vitesse t+1 du drone en fonction de l'influence de l'essaim, de sa vitesse, des targets et des zones d'exclusion 
            % Et gère les collisions éventuelles
        
            % r est la liste de rayon (répulsion, évitement, attraction_max), 
            % weights la liste de pondération (répulsion, attraction, target, évitement),
            % répulsion pour les drones, évitement pour le terrain,
            % attraction max, distance d'attraction maximum (bruit de communication)

            zones = obj.Environment.get_zones_pos_weights();
            n = length(obj.AliveDrones);

            posStateMatrix = zeros(n,3);
            speedStateMatrix = zeros (n,3);


            for i = 1:n
                drone = obj.AliveDrones{i};

                drone.update_pos(dt); % On update les drones à leur nouvelle position en fonction du dernier vecteur vitesse computé
                
                posStateMatrix(i,:) = drone.posState; 
                speedStateMatrix(i,:) = drone.speedState;
            end

            %obj.check_collisions(zones);
                
            obj.drones_pos_history_matrix(:,:,size(obj.drones_pos_history_matrix,3)+1) = posStateMatrix; % historique des positions pour le temps diff

            if size(obj.communicationMatrix,3) == 0
                obj.communicationMatrix = obj.drones_pos_history_matrix(:, :, 1);
            end

            %% Simulation de la communication
            % On simule une matrice de position perçue par les drones communicationMatrix
            % On utilisera donc cette matrice dans le computing de l'influence de l'essaim
            % Actuellement, fonctionnement pas exact, tous les drones utilisent cette matrice, plutôt que cette matrice + leur position réelle

            if isempty(obj.communicationFrequency) == 1
                obj.communicationFrequency = length(obj.Drones); % si pas de valeur de freq renseignée, les drones communiquent tous à chaque étape
            end

            obj.communicationMatrix = communication_simulation(obj.communicationMatrix, obj.drones_pos_history_matrix, obj.communicationFrequency, obj);

            %% CALCUL DES VOISINS
            % On utilise les positions connues par les drones pour le calcul
            [neighborI, nnmax] = VoroiNeighbor(obj.communicationMatrix, n); % Utils.Algo + a mod pour granularité info de comm (donner historique pos en entrée)
         
            %% SWARM INFLUENCE
            % On utilise les positions connues par les drones pour le calcul
            swarmInfluence = swarm_pond(obj.communicationMatrix, neighborI, n, nnmax, obj.swarm_weights, obj.r, obj); % Utils.Algo + a mod pour granularité de comm (donner historique pos en entrée) 
                    
            %% SPEED INFLUENCE
            % On utilise la position réelle pour le calcul
            speedInfluence = speed_pond(speedStateMatrix); % Utils.Algo

            %% TARGET INFLUENCE
            % On utilise la position réelle pour le calcul
            targetInfluence = target_pond(obj.target, posStateMatrix, obj); % Utils.Algo (On utilise position réelle)
           
            %% Calcul des zones d'évitement 
            % On utilise la position réelle pour le calcul
            zones = obj.Environment.get_zones_pos_weights();
            avoidInfluence = avoid_pond(posStateMatrix, zones, obj.altitude_min); % Utils.Algo

            %% Maintentant, pour chaque drone, on fait la pondération des influeneces swarm/target/speed et on les somme

            desiredVector = whole_pond(swarmInfluence, speedInfluence, targetInfluence, avoidInfluence, obj.weights); % Utils.Algo
           
            for i = 1:length(obj.AliveDrones)
                drone = obj.AliveDrones{i};
                
                [vX, vY, vZ] = SpeedProcessing(drone, i, desiredVector, dt);
            
                drone.speedState = [vX, vY, vZ];

            end
        end
    end
end






