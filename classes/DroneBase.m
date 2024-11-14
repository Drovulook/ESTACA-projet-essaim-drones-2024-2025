% classes/DroneBase.m
% Classe de base abstraite pour tous les types de drones

classdef (Abstract) DroneBase < handle
    % DroneBase : Classe de base abstraite pour les drones
    
    properties
        ID              % Identifiant unique du drone
        Type            % Type de drone : 'multirotor' ou 'fixedwing' 
        posState        % Vecteur de position à tn (1,3) [x, y, z]
        speedState = [0 0 0]     % Vecteur de vitesse à tn (1,3) [r, teta, phi]
        flightTime = 0  % Temps de vol en sec
        Target          % Coordonnées de la cible

    end
    
    methods
        % Constructeur pour initialiser un drone avec un ID, un environnement, un type et une position initiale
        function obj = DroneBase(id, type, initialPos)
            obj.ID = id; % Assigner l'identifiant unique
            obj.Type = type; % Assigner le type de drone
            

            % Générer un nom unique en utilisant le type et l'ID du drone
            uniqueName = sprintf('%sDrone%d_%s', upper(type(1)), id, datestr(now, 'HHMMSSFFF'));
        end

        function [distance] = closestDrone(obj, swarm); % Donne la distance au drone le plus proche
            distance = 100 %calcul a implémenter
        end
           
        function [score] = obsScore(obj, target, env); %Donne le score d'observation du drone
            score = 100 %calcul a implémenter
        end

        function [distance] = closestEnv(obj, target, env); %donne la distance à l'objet de l'env le plus proche
            distance = 100 %calcul a implémenter
        end

        function [autonomy] = autonomy(obj); % donne l'autonomie restante en fct de flightTime
            autonomy = 100 %calcul a implémenter
        end

        function [Vx Vy Vz] = velocityNorm(obj); % Normalise la vitesse
            r = obj.speedState(1,1)
            teta = obj.speedState(1,2)
            phi = obj.speedState(1,3)

            Vx = r*sin(phi)*cos(teta)
            Vy = r*sin(phi)*sin(teta)
            Vz = r*cos(phi)
        end
        
    end
    
    methods
        function update_pos(obj, dt) %Update la position du drone en fct de sa vitesse
            dpos = dt*obj.speedState; %on peut ajouter du noise
            obj.posState = obj.posState + dpos;

        end

        function Set_NextVelVector(obj, dt, r, swarm_weights, weights, pondeTarg, sat, neighborPos, neighborSpeed, target_list)
            
            % Différence de position avec les voisins
            rho_x = neighborPos(:,1) - obj.posState(1);
            rho_y = neighborPos(:,2) - obj.posState(2);
            rho_z = neighborPos(:,3) - obj.posState(3);
            rhon = sqrt(rho_x.^2 + rho_y.^2 + rho_z.^2);

            % Matrice de poids selon les distances
            weight_matrix = zeros(size(rhon));
            weight_matrix(rhon < r(1)) = -swarm_weights(1);
            weight_matrix(rhon >= r(1)) = swarm_weights(2);

            swarminfluence_x = (sum(weight_matrix.*rho_x./rhon, 1, 'omitnan'));
            swarminfluence_y = (sum(weight_matrix.*rho_y./rhon, 1, 'omitnan'));
            swarminfluence_z = (sum(weight_matrix.*rho_z./rhon, 1, 'omitnan'));

            %sum(weight_matrix,2) poids par ligne à diviser pour pondérer de la somme
            %On multiplie la projection sur un axe par le poids, qu'on
            %normalise par la norme euclidienne, pour obtenir un nouveau
            %vecteur d'influence

            %% SPEED INFLUENCE
            speedNorm = sqrt(sum(neighborSpeed.^2,2));
            speedinfluence_x = sum(neighborSpeed(:,1)./speedNorm);
            speedinfluence_y = sum(neighborSpeed(:,2)./speedNorm);
            speedinfluence_z = sum(neighborSpeed(:,3)./speedNorm);

            speedinfluence_x(isnan(speedinfluence_x)) = 0;
            speedinfluence_y(isnan(speedinfluence_y)) = 0;
            speedinfluence_z(isnan(speedinfluence_z)) = 0;
            
            %% Target

            %Différence de position aux targets, en ligne, les drones, en
            %colonne la diff à chaque target
            %Rajouter un IF si pas de target + Comportement retour maison

            T_x = target_list(:,1)' - obj.posState(1);
            T_y = target_list(:,2)' - obj.posState(2);
            T_z = target_list(:,3)' - obj.posState(3);

            T_eucli = sqrt(T_x.^2 + T_y.^2 + T_z.^2); % On peut y ajouter de la pondération de cible en fct de la distance ; distance d'attraction max à ajouter (r(3)/w(3))
            T_x_pond = T_x./T_eucli;
            T_y_pond = T_y./T_eucli;
            T_z_pond = T_z./T_eucli;

            % Appliquer la pondération de chaque cible aux coordonnées (x, y, z)
            T_x_pond = T_x_pond .* pondeTarg(:)';
            T_y_pond = T_y_pond .* pondeTarg(:)';
            T_z_pond = T_z_pond .* pondeTarg(:)';

            T_x_pond = sum(T_x_pond)/sum(pondeTarg);
            T_y_pond = sum(T_y_pond)/sum(pondeTarg);
            T_z_pond = sum(T_z_pond)/sum(pondeTarg);

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
            
            newSpeedVec = [Pond_x Pond_y Pond_z];
            newSpeedVec(newSpeedVec < sat(1)) = sat(1);
            newSpeedVec(newSpeedVec > sat(2)) = sat(2);

            obj.SetSpeedWithConstraints(dt, newSpeedVec);
            %obj.speedState = newSpeedVec;
            end
    end
end
