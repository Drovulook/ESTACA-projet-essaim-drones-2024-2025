% classes/SwarmManager.m
% Classe pour gérer un essaim de drones

classdef SwarmManager < handle
    properties
        Drones          % Tableau de cellules contenant les objets DroneBase (drones de l'essaim)
        Environment     % Objet uavScenario représentant l'environnement de simulation

        %% Swarm PROPERTIES
        
        Target %1 ligne par target en coordonées xyz, A changer + tard en classe pour définir niveau d'intérêt (pondération d'attraction) + mouvement
        target_history_matrix
        drones_pos_history_matrix
    end
    
    methods
        % Constructeur pour initialiser le gestionnaire d'essaim avec l'environnement
        function obj = SwarmManager(env, temps)
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
            obj.Target = newTarget;
        end

        % Méthode pour mettre à jour la vitesse de chaque drone dans l'essaim
        
        function update_speed(obj, dt, r, swarm_weights, weights, pondeTarg, sat) %r est la liste de rayon (répulsion, évitement, attraction_max), w la liste de pondération (répulsion, attraction, évitement)
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

            obj.drones_pos_history_matrix(:,:,size(obj.drones_pos_history_matrix,3)+1) = posStateMatrix;

            % Calcul des voisins de voronoi avec une triangulation de chaque drone
            dTri = delaunay(posStateMatrix); %Création de la matrice de triangulation
            vn = sparse(dTri, dTri(:,[2 3 4 1]),1); % décalage des indices et on crée la matrice des voisins de voroi (vn)
            vn = vn | vn'; %On rend vn symétrique pour s'assurer que la relation de voisinage ets bijective
            
            listI = repmat(1:n,1,n);
            ns = listI(vn);
            nn = sum(vn);
            nnmax = max(nn);
            neighborI = full(sparse(nn+1,1:n,n+1));
            neighborI = cumsum(neighborI(1:end-1,:),1); 
            neighborI(neighborI==0) = ns;
            neighborI = neighborI';

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

            
            %% SWARM INFLUENCE

            % On fusionne les matrices + on gère l'indice de fin
            stateA = [posStateMatrix speedStateMatrix ; nan(1,6)];

            % Différence de position entre les drones et leurs voisins
            rho_x = reshape(stateA(neighborI,1),n,nnmax) - posStateMatrix(:,1); % Différence axe x
            rho_y = reshape(stateA(neighborI,2),n,nnmax) - posStateMatrix(:,2); % Différence axe y
            rho_z = reshape(stateA(neighborI,3),n,nnmax) - posStateMatrix(:,3); % Différence axe z
            rhon = sqrt(rho_x.^2 + rho_y.^2 + rho_z.^2); % Distance euclidienne en 3D

          
            %Chaque matrice a le drone par ligne, et ses contacts par
            %dépendance sur la ligne. Ainsi, on peut, en fonction de rhon,
            %qui définit la distance euclidienne, définir si le drone de la
            %case est dans le cercle d'attraction, de répulsion,
            %d'orientation ou d'évitement

            %Règles de pondération
            weight_matrix = zeros(size(rhon));
            weight_matrix(rhon < r(1)) = -swarm_weights(1); % Cercle de répulsion
            weight_matrix(rhon >= r(1)) = swarm_weights(2); % Cercle d'attraction
        
            swarminfluence_x = (sum(weight_matrix.*rho_x./rhon, 2, 'omitnan'));
            swarminfluence_y = (sum(weight_matrix.*rho_y./rhon, 2, 'omitnan'));
            swarminfluence_z = (sum(weight_matrix.*rho_z./rhon, 2, 'omitnan'));
         
            %sum(weight_matrix,2) poids par ligne à diviser pour pondérer de la somme
            %On multiplie la projection sur un axe par le poids, qu'on
            %normalise par la norme euclidienne, pour obtenir un nouveau
            %vecteur d'influence

            %% SPEED INFLUENCE
            
            speedNorm = sqrt(sum(speedStateMatrix.^2,2));
            speedinfluence_x = speedStateMatrix(:,1)./speedNorm;
            speedinfluence_y = speedStateMatrix(:,2)./speedNorm;
            speedinfluence_z = speedStateMatrix(:,3)./speedNorm;

            speedinfluence_x(isnan(speedinfluence_x)) = 0;
            speedinfluence_y(isnan(speedinfluence_y)) = 0;
            speedinfluence_z(isnan(speedinfluence_z)) = 0;
            

            %% Target

            %Différence de position aux targets, en ligne, les drones, en
            %colonne la diff à chaque target
            %Rajouter un IF si pas de target + Comportement retour maison
           

            T_x = obj.Target(:,1)' - posStateMatrix(:,1);
            T_y = obj.Target(:,2)' - posStateMatrix(:,2);
            T_z = obj.Target(:,3)' - posStateMatrix(:,3);

            T_eucli = sqrt(T_x.^2 + T_y.^2 + T_z.^2); % On peut y ajouter de la pondération de cible en fct de la distance ; distance d'attraction max à ajouter (r(3)/w(3))
            T_x_pond = T_x./T_eucli;
            T_y_pond = T_y./T_eucli;
            T_z_pond = T_z./T_eucli;

            %Pondération target de test
            T_x_pond = T_x_pond .* pondeTarg;
            T_y_pond = T_y_pond .* pondeTarg;
            T_z_pond = T_z_pond .* pondeTarg;
   
            T_x_pond = sum(T_x_pond,2)/sum(pondeTarg);
            T_y_pond = sum(T_y_pond,2)/sum(pondeTarg);
            T_z_pond = sum(T_z_pond,2)/sum(pondeTarg);


            %% Maintentant, pour chaque drone, on fait la pondération des influeneces swarm/target/speed et on les sommes
            % Il ne faut pas oublier de pondérer les influences avec son propre vecteur
            % vitesse, weighté en fonction du type de drone, pour simuler l'inertie.
            % GROS POINT BLOQUANT, je ne vois pas comment intégrer les
            % valeurs d'angles max, de G, de vario, etc. Attendu qu'on l'intègre en
            % pondération, axe de travail à pousser en second temps

            %distances pondérées et normalisées = matrices d'attraction
            swarm_weight = weights(1);
            speed_weight = weights(2);
            target_weight = weights(3); 
            
            Pond_x = (swarminfluence_x*swarm_weight + speedinfluence_x*speed_weight + T_x_pond*target_weight);
            Pond_y = (swarminfluence_y*swarm_weight + speedinfluence_y*speed_weight + T_y_pond*target_weight);
            Pond_z = (swarminfluence_z*swarm_weight + speedinfluence_z*speed_weight + T_z_pond*target_weight);

            %Concrètement, on pondère une fois les cercles de répulsion,
            %orientation des drones, puis on repondère avec la vitesse
            %actuelle + L'attractivité de la target

            %A voir si on ne rajoute pas le troisième cercle concentrique
            %d'attraction avec les autres drones
            
            newSpeedMatrix = [Pond_x Pond_y Pond_z];

            newSpeedMatrix(newSpeedMatrix < sat(1)) = sat(1);
            newSpeedMatrix(newSpeedMatrix > sat(2)) = sat(2);
            
            for i = 1:n
                %On réinjecte la nouvelle vitesse
                obj.Drones{i}.speedState = newSpeedMatrix(i,:);
            end
          

        end

    end
end
