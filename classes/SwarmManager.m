% classes/SwarmManager.m
% Classe pour gérer un essaim de drones

classdef SwarmManager < handle
    properties
        Drones          % Tableau de cellules contenant les objets DroneBase (drones de l'essaim)
        Environment     % Objet uavScenario représentant l'environnement de simulation

        %% Swarm PROPERTIES
        
        target_list %1 ligne par target en coordonées xyz, A changer + tard en classe pour définir niveau d'intérêt (pondération d'attraction) + mouvement
        instant_trimesh
        instant_allpos
    end
    
    methods
        % Constructeur pour initialiser le gestionnaire d'essaim avec l'environnement
        function obj = SwarmManager(env, target_list)
            obj.Drones = {};  % Initialiser le tableau de drones comme vide
            obj.Environment = env; % Assigner l'environnement de simulation
            obj.target_list = target_list
        end
        
        % Méthode pour ajouter un drone à l'essaim
        function addDrone(obj, droneType, initialPosition)
            id = length(obj.Drones) + 1; % Déterminer un ID unique pour le nouveau drone
            load('data/params.mat', 'multirotorParams', 'fixedWingParams'); % Charger les paramètres des drones
            
            %Ici il faut changer pour essayer de faire un quadrillage de
            %départ au sol. Ici je les fait juste commencer espacés
            count = 1;
            while true
                if length(obj.Drones) == 0 
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


        function update_target(obj, target_list)
            obj.target_list = target_list;
        end

        % Méthode pour mettre à jour la vitesse de chaque drone dans l'essaim
        
        function [posStateMatrix, speedStateMatrix, neighborI] = Get_neighborList(obj, dt, r, swarm_weights, weights, pondeTarg, sat) %r est la liste de rayon (répulsion, évitement, attraction_max), w la liste de pondération (répulsion, orientation, attraction, évitement)
            %répulsion pour les drones, évitement pour le terrain ;
            %attraction max, distance d'attraction maximum
            n = length(obj.Drones);
            
            posStateMatrix = zeros(n,3);
            speedStateMatrix = zeros (n,3);

            for i = 1:n
                obj.Drones{i}.update_pos(dt); %On update la position avec la dernière vitesse et la dernière position
                posStateMatrix(i,:) = obj.Drones{i}.posState; % Crée matrice de positions
                speedStateMatrix(i,:) = obj.Drones{i}.speedState; % Crée matrice de vitesses
            end
            obj.instant_allpos = posStateMatrix;

            % Calcul des voisins de voronoi avec une triangulation de chaque drone
            dTri = delaunay(posStateMatrix); %Création de la matrice de triangulation
            vn = sparse(dTri, dTri(:,[2 3 4 1]),1); % décalage des indices et on crée la matrice des voisins de voroi (vn)
            vn = vn | vn'; %On rend vn symétrique pour s'assurer que la relation de voisinage ets bijective
            obj.instant_trimesh = dTri ;

            listI = repmat(1:n,1,n);
            ns = listI(vn);
            nn = sum(vn);
            neighborI = full(sparse(nn+1,1:n,n+1));
            neighborI = cumsum(neighborI(1:end-1,:),1); 
            neighborI(neighborI==0) = ns;
            neighborI = neighborI';    

        end

        function update_speeds(obj, dt, r, swarm_weights, weights, pondeTarg, sat)
            [posStateMatrix, speedStateMatrix, neighborList] = Get_neighborList(obj, dt, r, swarm_weights, weights, pondeTarg, sat);
            %La matrice NeighborI permet, lorsque parsée avec stateA, de
            %donner une matrice ou sur la ligne, on a le drone, et sur
            %chaque colonne, le numéro de ligne de ses voisins dans un
            %tétraèdre
            % Actuellement la partie du dessus fonctionne en tétraèdre, si
            % on veut la changer pour utiliser TOUS les voisins
            % (fonctionnement omniscient télépathique), il faut boucler
            % pour chaque drone, tous les autres drones, et computer leur
            % distances comme fait dessous, je trouve que c'est plus
            % élégant au-dessus
            %La pb avec ça, c'est le comportement dans un cas
            %coplanaire/colinéaire, ça marche pas
            n_drones = length(obj.Drones);
            parfor i = 1:n_drones %exécution du code en parallèle, sur plusieurs threads (un pour chaque drone)
                neighbors = neighborList(i, neighborList(i,:) ~= n_drones+1);
                neighborPos = posStateMatrix(neighbors, :);
                neighborSpeed = speedStateMatrix(neighbors, :);
                obj.Drones{i}.Set_NextVelVector(dt, r, swarm_weights, weights, pondeTarg, sat, neighborPos, neighborSpeed, obj.target_list);
                %calcule le prochain vecteur vitesse pour chaque drone
                %!!à terme, prendre en compte les contraintes liées à la communication
            end
        end
    end
end
