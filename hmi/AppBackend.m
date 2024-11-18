classdef AppBackend < handle
    %cette classe gère tout ce qui ne concerne pas l'affichage
    properties
        App; 
        env;
        zones;
        targetList;
        swarm;
        settings;
    end
    methods
        
        function Init(obj, app)
            %clear all; 
            %close all;
            clc;

            obj.App = app;

            obj.settings = Settings();

            obj.env = Environment(10, 200, [-150, 150, 150, -150], [-150, -150, 150, 150], [0, 0, 0, 0]);
            obj.zones = Zones(obj); % Configuration des zones de l'espace aérien

            obj.targetList = obj.settings.targetListInit;

            obj.swarm = SwarmManager(obj, obj.env, obj.targetList); % Initialiser le gestionnaire d'essaim avec l'environnement
            obj.swarm.update_target(obj.targetList);
        
            for i = 1:obj.settings.numMultirotorInit
                obj.swarm.addDrone('multirotor', obj.zones.homeBaseCoord);
            end

            spawn_size=obj.settings.spawn_size;
            min_distance=obj.settings.min_distance;

            for i = 1:obj.settings.numMultirotorInit
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
        end
    
        function update(obj)
            obj.swarm.update_speeds(obj.settings.dt, obj.settings.r, obj.settings.swarm_weights, obj.settings.weights, obj.settings.pondeTarg, obj.settings.sat)
        end

        function OnDronesCollision(obj, ID1, ID2)
            obj.App.OnDronesCollision(ID1, ID2);
        end

        function OnDronesNbUpdate(obj)
            obj.App.OnDronesNbUpdate();
        end

    end

end