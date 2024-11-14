classdef AppBackend < handle
    properties
        env
        zones;
        targetList
        swarm
        numMultirotor
    end
    methods
        function Init(obj);
            clear all;
            close all;
            clc;

            obj.env = Environment(10, 200, [-150, 150, 150, -150], [-150, -150, 150, 150], [0, 0, 0, 0]);
            obj.zones = Zones(obj); % Configuration des zones de l'espace aérien

            obj.targetList = [100 50 50 ; 50 100 50];

            obj.swarm = SwarmManager(obj.env.Env, obj.targetList); % Initialiser le gestionnaire d'essaim avec l'environnement
            obj.numMultirotor = 10 % Nombre de drones multirotors

            for i = 1:obj.numMultirotor
                obj.swarm.addDrone('multirotor', obj.zones.homeBaseCoord);
            end

            spawn_size=50;
            min_distance=0.5;

            for i = 1:obj.numMultirotor
                % Position aléatoire dans la zone définie (ici entre 0 et 'zone_size' sur x et y)
                valid_position = false;
                while ~valid_position
                    % Génère une position aléatoire dans la zone
                    new_pos = [rand*spawn_size, rand*spawn_size, rand*spawn_size];
                    
                    % Vérifier si la position est trop proche d'un autre drone
                    valid_position = true;
                    for j = 1:i-1
                        % Calculer la distance entre le drone actuel et les drones précédents
                        dist = norm(new_pos - obj.swarm.Drones{j}.posState);
                        if dist < min_distance
                            valid_position = false;
                            break;  % Si trop proche, on quitte la boucle et on génère une nouvelle position
                        end
                    end
                end
                % Assigner la position valide au drone i
                obj.swarm.Drones{i}.posState = new_pos;
            end
                
            % figure;
            % testplot(swarm)
            r = [10 50 100]/2; %Répulsion, évitement, attraction max (rayons)
            swarm_weights = [1.4 1 1.2 2]; %Pondérations répulsion, alignement, attraction drone, évitement obstacle
            weights = [0.5 1.2 0.8]/2; %Influence sur le vecteur vitesse de : l'environnement ; la vitesse du drone a t-1 (maniabilité) ; la target
            pondeTarg = [10 15]; %Pondération de la value des 2 targets
            %pondDistTarg = 0.5; %accorde une importance variable aux cibles en fonction de la distance
            satextrem = 10; %Saturation de vitesse projetée
            sat = [-satextrem satextrem];
            temps = 1000;
            dt = 0.3;
        end

    end

end