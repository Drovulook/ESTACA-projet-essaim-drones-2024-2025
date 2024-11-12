% classes/SwarmManager.m
% Classe pour gérer un essaim de drones

classdef SwarmManager < handle
    properties
        Drones          % Tableau de cellules contenant les objets DroneBase (drones de l'essaim)
        Environment     % Objet uavScenario représentant l'environnement de simulation

        %% Swarm PROPERTIES
        
        Ip = 1.5; % 0-10 vision control power
        In = 0.3; % 0-1 vision control noise

    end
    
    methods
        % Constructeur pour initialiser le gestionnaire d'essaim avec l'environnement
        function obj = SwarmManager(env)
            obj.Drones = {};  % Initialiser le tableau de drones comme vide
            obj.Environment = env; % Assigner l'environnement de simulation
        end
        
        % Méthode pour ajouter un drone à l'essaim
        function addDrone(obj, droneType, initialPosition)
            id = length(obj.Drones) + 1; % Déterminer un ID unique pour le nouveau drone
            load('data/params.mat', 'multirotorParams', 'fixedWingParams'); % Charger les paramètres des drones
            
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



        % Méthode pour mettre à jour la position de chaque drone dans l'essaim

        
        function update_speed(obj, dt)
            n = length(obj.Drones)

            for i = 1:n
                obj.Drones{i}.update_pos
                posStateMatrix = [posStateMatrix ; obj.Drones{i}.posState]; % Crée matrice de positions
                speedStateMatrix = [speedStateMatrix ; obj.Drones{i}.speedState] % Crée matrice de vitesses
            
            % Calcul des voisins de voronoi avec une triangulation de chaque drone
            dTri = delaunay(posState); %Création de la matrice de triangulation
            vn = sparse(dTri, dTri(:,[2 3 4 1]),1); % décalage des indices et on crée la matrice des voisins de voroi (vn)
            vn = vn | vn'; %On rend vn symétrique pour s'assurer que la relation de voisinage ets bijective
            
            % A partir de la je comprends plus rien, mais ça marche
            listI = repmat(1:n,1,n)
            ns = listI(vn)
            nn = sum(vn)
            nnmax = max(nn)

            neighborI = full(sparse(nn+1,1:n,n+1));
            neighborI = cumsum(neighborI(1:end-1,:),1); 
            neighborI(neighborI==0) = ns;
            neighborI = neighborI';

            
            
            % vision control induced angular velocity
            stateA = [posStateMatrix speedStateMatrix; nan(1,6)];
            
            phi = reshape(stateA(neighborI,3),n,nnmax) - posStateMatrix(:,3);
            
            % Différence de position entre les drones et leurs voisins
            rho1 = reshape(stateA(neighborI,1),n,nnmax) - posStateMatrix(:,1); % Différence axe x
            rho2 = reshape(stateA(neighborI,2),n,nnmax) - posStateMatrix(:,2); % Différence axe y
            rho3 = reshape(stateA(neighborI,3),n,nnmax) - posStateMatrix(:,3); % Différence axe z

            rhon = sqrt(rho1.^2 + rho2.^2 + rho3.^2); % Distance euclidienne en 3D
            theta = getangle(state(:,3),rho1,rho2,rhon);
            minDist = nanmin(rhon,[],2);
            
            w_vision = nansum((Ip*sin(phi) + rhon.*sin(theta)).*(1 + cos(theta)),2)./nansum(1 + cos(theta),2);
  


            end

        end

        function [theta2 phi2] = getangle(teta, phi, rhox, rhoy, rhoz,rhom) % renvoie les projection d'angles sur xy et xz de l'angle entre un vecteur définit par 3 coordonnées et un vecteur défini par 2 angles
            
            if nargin < 4, rhom = sqrt(rhox.^2 + rhoy.^2 + rhoz.^2); end
            rhox = rhox./rhom;
            rhoy = rhoy./rhom;
            rhoz = rhoz./rhom;

            ex = sin(teta) * cos(phi);
            ey = sin(teta) * sin(phi);
            ez = cos(teta);
            
            prod_scal = ex .* rx + ey .* ry + ez .* rz; %produit scalaire des 2 vecteurs

            angle_polaire = acos(prod_scal); % angle à transposer en 2 angles
            %J'EN SUIS LA ET J'ARRIVE PAS PTN DE MERDE

            sgn = sign(ex.*rhoy - ey.*rhox);
            sgn(sgn == 0) = 1;
            
            theta = sgn.*acos(ex.*rhox + ey.*rhoy);
            
        end

    end
end
