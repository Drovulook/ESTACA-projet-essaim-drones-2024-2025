% classes/SwarmManager.m
% Classe pour gérer un essaim de drones

classdef SwarmManager < handle
    properties
        backend             % Référence à l'application (pour afficher un message quand il y a collision par ex)
        settings
        env     % Objet uavScenario représentant l'environnement de simulation

        %% Swarm PROPERTIES

        Drones % Liste d'objet Drones_Base 
        drones_pos_history_matrix % Matrice 3 dim de l'historique des positions
        communicationFrequency % Hz, le nombre de fois par itération que les drones peuvent communiquer. Si 1 Hz, 1 drone seulement communique par itération
        communicationMatrix = zeros(0,0,0) % La matrice de communication utilisée par les drones pour se repérer entre eux
        LastSentDroneTimer = 0 %Pas de temps depuis dernier envoi drone
        Drone_sending_schedule = 2 % 1 drone envoyé tous les x pas de temps

        %% Comportement essaim
        threshold_radius = 15 % Distance de trigger des waypoint cycliques
        altitude_min = 10 % Altitude min des drones
        dist_target_min = 20 % Distance minimal du drone à la target
        orbit_radius = 60 % Distance des fixedwing à la cible à partir de laquelle ils orbitent
        min_dist_between_drones = 2 % si <, alors crash (distance en mètres)
        dt_evitement_max = 0 % dt max pour lequel on calcul l'évitement vertical

        r = [30 60 100]  % Rayons Répulsion, attraction au sein de l'essaim
        swarm_weights = [1.4 0.8]; % Pondérations répulsion, attraction au sein de l'essaim
        weights = [0.5 1.2 1 10] / 10; % pondération essaim, inertie, target, évitement

        TO_WP = [70 20 30]
        landing_WP = [-200 -10 40; -140 -5 20; -80 0 10]
        
        observationMatrix % Matrice de score, n_lignes, 1 colonne. Pour l'instant une distance*
        


    end
    
    properties (Dependent)
        AliveDrones
        FixedWing
        MultiRotor
        CrashedDrones
        targets % target en coordonées xyz, une target par ligne
        StandBy
        Phase
    end

    methods
        % Constructeur pour initialiser le gestionnaire d'essaim avec l'environnement
        function obj = SwarmManager(env, temps) % Bien prendre l'objet env
            
            obj.Drones = {};  % Initialiser le tableau de drones comme vide
            obj.env = env; % Assigner l'environnement de simulation

        end


        % Méthode pour ajouter un drone à l'essaim
        function addDrone(obj, dType, droneName, modelData, initialPosition)
            % Method to add a drone to the swarm, using CSV-based parameters
            
            id = length(obj.Drones) + 1;  % Unique ID for the new drone
            
            % If you still want the "grid offset" approach:
            index = id - 1;
            dx = 5; % m
            dy = 5; % m
            side = 5;
            row = floor(index / side);
            col = mod(index, side);
            initialPosition(1) = initialPosition(1) + col*dx;
            initialPosition(2) = initialPosition(2) + row*dy;
            initialPosition(3) = rand;  % some altitude or random offset
            
            switch lower(dType)
                case "multirotor"
                    % Provide the entire modelData to the Drone constructor
                    drone = MultirotorDrone(id, initialPosition, modelData, obj);
                case "fixedwing"
                    drone = FixedWingDrone(id, initialPosition, modelData, obj);
                otherwise
                    error('Unknown drone type: %s', dType);
            end

            drone.setPhase('stand-by')

            drone.Name = droneName;    
            drone.Model = modelData.Model;  
            obj.Drones{end+1} = drone;
        end
    
        function targets = get.targets(obj)
            targets = obj.env.targets;
        end

        function alive = get.AliveDrones(obj) 
            alive = {};
            for idx = 1:length(obj.Drones)
                if obj.Drones{idx}.IsAlive == 1
                    alive{end + 1} = obj.Drones{idx};
                end
            end
        end

        function crashed = get.CrashedDrones(obj)
            crashed = {};
            for idx = 1:length(obj.Drones)
                if obj.Drones{idx}.IsAlive == 0
                    crashed{end + 1} = obj.Drones{idx};
                end
            end
        end

        function checkCrash(obj, rhon, altitude) % fonction utilisée dans swarm_pond
            crashList = any(rhon < obj.min_dist_between_drones, 2);
            liste = obj.AliveDrones;

            for idx = 1:length(liste)
                if crashList(idx, 1) == 1  && (~contains(liste{idx}.phase, 'stand-by') && ~contains(liste{idx}.phase, 'take-off') && ~contains(liste{idx}.phase, 'landing') && ~contains(liste{idx}.phase, 'return') && ~contains(liste{idx}.phase, 'reload'))
                    liste{idx}.crashDrone;
                    liste{idx}.phase
                    disp([num2str(liste{idx}.ID) ' crashed' ]);
                elseif altitude(idx) < 0
                    liste{idx}.crashDrone;
                    disp([num2str(liste{idx}.ID) ' crashed on the ground' ]);
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

        function standby = get.StandBy(obj)
            standby = {};
            for idx = 1:length(obj.AliveDrones)
                if contains(obj.AliveDrones{idx}.phase, 'stand-by')
                    standby{end+1} = obj.AliveDrones{idx};
                end
            end
        end

        function ph = get.Phase(obj)
            ph = {};
            for idx = 1:length(obj.AliveDrones)
                ph = [ph ; obj.AliveDrones{idx}.phase];
            end
        end
      
        function resetCommunications(obj)
            for idx = 1:length(obj.Drones)
                obj.Drones{idx}.hasCommunicated = 0;
            end
        end

        % Méthode pour retirer un drone de l'essaim
        function removeDrone(obj, id) 
            for idx = 1:length(obj.Drones)
                if obj.Drones{idx}.ID == id
                    if obj.Drones{idx}.IsAlive == 1
                        obj.Drones{idx} = [];
                        return
                    end
                end
            end
            %Past this point, no candidate was found
            error('No drone at this ID found')
        end



        function observationScore(obj, T_eucli)
            obj.observationMatrix = T_eucli; % Fonction a modifier pour comportement du score d'observation
        end

        function update_speeds(obj, dt)

            n = length(obj.Drones);
            %Check les stand by

            % Compute le vecteur vitesse t+1 du drone en fonction de l'influence de l'essaim, de sa vitesse, des targets et des zones d'exclusion 
            % Et gère les collisions éventuelles
        
            % r est la liste de rayon (répulsion, évitement, attraction_max), 
            % weights la liste de pondération (répulsion, attraction, target, évitement),
            % répulsion pour les drones, évitement pour le terrain,
            % attraction max, distance d'attraction maximum (bruit de communication)
            

            % Séquenceur de lancement
            if length(obj.StandBy) > 0 & obj.LastSentDroneTimer > obj.Drone_sending_schedule
                obj.StandBy{1}.setPhase('take-off')
                obj.LastSentDroneTimer = 0;
            end

            obj.LastSentDroneTimer = obj.LastSentDroneTimer + dt;
            

            n = length(obj.Drones);

            posStateMatrix = zeros(n,3);
            speedStateMatrix = zeros (n,3);


            for i = 1:n
                drone = obj.Drones{i};

                drone.update_pos(dt); % On update les drones à leur nouvelle position en fonction du dernier vecteur vitesse computé
                posStateMatrix(i,:) = drone.posState; 
                speedStateMatrix(i,:) = drone.speedState;
            end

                
            obj.drones_pos_history_matrix(:,:,size(obj.drones_pos_history_matrix,3)+1) = posStateMatrix; % historique des positions pour le temps diff

            if size(obj.communicationMatrix,3) == 0
                obj.communicationMatrix = obj.drones_pos_history_matrix(:, :, 1);
            end

            %% Simulation de la communication
            % On simule une matrice de position perçue par les drones communicationMatrix
            % On utilisera donc cette matrice dans le computing de l'influence de l'essaim
            % Actuellement, fonctionnement pas exact, tous les drones utilisent cette matrice, plutôt que cette matrice + leur position réelle

            if isempty(obj.communicationFrequency) == 1
                obj.communicationFrequency = length(obj.AliveDrones); % si pas de valeur de freq renseignée, les drones communiquent tous à chaque étape
            end

            obj.communicationMatrix = communication_simulation(obj.communicationMatrix, obj.drones_pos_history_matrix, obj.communicationFrequency, obj);

            %% CALCUL DES VOISINS
            % On utilise les positions connues par les drones pour le calcul
            [neighborI, nnmax] = VoroiNeighbor(obj.communicationMatrix, n); % Utils.Algo + a mod pour granularité info de comm (donner historique pos en entrée)
            
            %% SWARM INFLUENCE
            % On utilise les positions connues par les drones pour le calcul
            swarmInfluence = swarm_pond(obj.communicationMatrix, neighborI, n, nnmax, obj); % Utils.Algo + a mod pour granularité de comm (donner historique pos en entrée) 
                    
            %% SPEED INFLUENCE
            % On utilise la position réelle pour le calcul
            speedInfluence = speed_pond(speedStateMatrix); % Utils.Algo

            %% TARGET INFLUENCE
            % On utilise la position réelle pour le calcul
            [targetInfluence, T_eucli] = target_pond(posStateMatrix, obj); % Utils.Algo (On utilise position réelle)

            obj.observationScore(T_eucli); % Update de la matrice de score d'observations

            %% Calcul des zones d'évitement 
            % On utilise la position réelle pour le calcul
            zones = obj.env.get_zones_pos_weights();
            avoidInfluence = avoid_pond(posStateMatrix, speedStateMatrix, dt, zones, obj.altitude_min, obj.dt_evitement_max, obj); % Utils.Algo

            %% Maintentant, pour chaque drone, on fait la pondération des influeneces swarm/target/speed et on les somme

            desiredVector = whole_pond(swarmInfluence, speedInfluence, targetInfluence, avoidInfluence, obj.weights); % Utils.Algo
           
            for i = 1:length(obj.Drones)
                drone = obj.Drones{i};
                
                [vX, vY, vZ] = SpeedProcessing(drone, i, desiredVector, dt);
            
                drone.speedState = [vX, vY, vZ];

            end

        end
    end
end






