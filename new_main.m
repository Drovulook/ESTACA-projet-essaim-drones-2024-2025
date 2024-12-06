clear all;
close all;
clc;

tic

env = Environment(10, 200, [-150, 150, 150, -150], [-150, -150, 150, 150], [0, 0, 0, 0]);
%setupZones;
homeBaseCoord = [0, 0, 0];
temps = 100000;


swarm = SwarmManager(env, temps); % Initialiser le gestionnaire d'essaim avec l'environnement
numMultirotor = 0; % Nombre de drones multirotors

% Ajouter les drones multirotors à l'essaim, placés à la coordonnée de la base
for i = 1:numMultirotor
    swarm.addDrone('multirotor', homeBaseCoord);
end

%assignation des pos init, du à l'algo de delaunay, si coplanaire ou
%colinéaire, kaput

zone_size=50;
min_distance=0.5;
for i = 1:numMultirotor
    % Position aléatoire dans la zone définie (ici entre 0 et 'zone_size' sur x et y)
    valid_position = false;
    while ~valid_position
        % Génère une position aléatoire dans la zone
        new_pos = [rand*zone_size, rand*zone_size, rand*zone_size];
        
        % Vérifier si la position est trop proche d'un autre drone
        valid_position = true;
        for j = 1:i-1
            % Calculer la distance entre le drone actuel et les drones précédents
            dist = norm(new_pos - swarm.Drones{j}.posState);
            if dist < min_distance
                valid_position = false;
                break;  % Si trop proche, on quitte la boucle et on génère une nouvelle position
            end
        end
    end
    % Assigner la position valide au drone i
    swarm.Drones{i}.posState = new_pos;
end


% Ajouter les drones multirotors à l'essaim, placés à la coordonnée de la base
swarm.addDrone('fixedwing', homeBaseCoord);


%assignation des pos init, du à l'algo de delaunay, si coplanaire ou
%colinéaire, kaput

Target = [100 50 50];
swarm.update_target(Target); 

Waypoints = [100 50 50 ; 50 50 50 ; 50 100 50 ; 100 100 50]; 
swarm.waypoints = Waypoints; 
swarm.FixedWing{1}.Waypoints = Waypoints;

r = [30 60 100]; %Répulsion, évitement, attraction max (rayons)
swarm_weights = [1.4 0.8 1.6]; %Pondérations répulsion, alignement, attraction drone, évitement obstacle

weights = [0.5 1.2 1 10]/10; %(répulsion entre drones, attraction entre drones, target, évitement)
%J'ai remarqué qu'en divisant tout par 10, on réduit les comportements HF,
%et les drones sont plus posés

pondeTarg = [10]; %Pondération de la value des 2 targets
satextrem = 1.5; %Saturation de vitesse projetée
sat = [-satextrem satextrem];
dt = 1;

RTPlot2(env, swarm, dt, temps, r, swarm_weights, weights, pondeTarg, sat, Target);

toc

