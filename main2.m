clear all;
close all;
clc;

env = Environment(10, 200, [-150, 150, 150, -150], [-150, -150, 150, 150], [0, 0, 0, 0]);
setupZones; % Configuration des zones de l'espace aérien

swarm = SwarmManager(env.Env); % Initialiser le gestionnaire d'essaim avec l'environnement
numMultirotor = 10; % Nombre de drones multirotors


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
    
% figure;
% testplot(swarm)
<<<<<<< Updated upstream
Target = [100 50 50 ; 50 100 50];
swarm.Target = Target; % 2 targets pour l'instant, sinon c'est cassé mdr, pas plus, pas moins
r = [10 60 100]; %Répulsion, évitement, attraction max (rayons)
swarm_weights = [1.4 1.2 2]; %Pondérations répulsion, attraction drone, évitement obstacle
weights = [0.5 1.2 0.8]/4; %Influence sur le vecteur vitesse de : l'environnement ; la vitesse du drone a t-1 (maniabilité) ; la target
%J'ai remarqué qu'en divisant tout par 10, on réduit les comportements HF,
%et les drones sont plus posés
pondeTarg = [10 0]; %Pondération de la value des 2 targets
satextrem = 2; %Saturation de vitesse projetée
sat = [-satextrem satextrem];
temps = 1000;
dt = 1;
<<<<<<< Updated upstream
rand;
RTPlot2(env, swarm, dt, temps, r, swarm_weights, weights, pondeTarg, sat, Target);
=======

RTPlot2(env, swarm, dt, temps, r, swarm_weights, weights, pondeTarg, sat, Target)
>>>>>>> Stashed changes
