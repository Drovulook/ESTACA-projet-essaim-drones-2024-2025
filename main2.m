clear all;
close all;
clc;

env = Environment(10, 200, [-10, 10, 10, -10], [-10, -10, 10, 10], [0, 0, 0, 0]);
setupZones; % Configuration des zones de l'espace aérien

swarm = SwarmManager(env.Env); % Initialiser le gestionnaire d'essaim avec l'environnement
numMultirotor = 4; % Nombre de drones multirotors


% Ajouter les drones multirotors à l'essaim, placés à la coordonnée de la base
for i = 1:numMultirotor
    swarm.addDrone('multirotor', homeBaseCoord);
end

%assignation des pos init, du à l'algo de delaunay, si coplanaire ou
%colinéaire, kaput
swarm.Drones{1}.posState = [0 1 1];
swarm.Drones{2}.posState = [0 0 0];
swarm.Drones{3}.posState = [1 0 0];
swarm.Drones{4}.posState = [1 1 0];

% figure;
% testplot(swarm)

swarm.Target = [10 10 10 ; -5 -7 30]; % 2 targets pour l'instant, sinon c'est cassé mdr, pas plus, pas moins
r = [5 10 50]; %Répulsion, évitement, attraction max (rayons)
w = [-10 2 50 30]; %Pondérations répulsion, orientation, attraction, évitement
ABC = [1 1 1] %Influence sur le vecteur vitesse de : l'environnement ; la vitesse du drone a t-1 (maniabilité) ; la target
pondeTarg = [1000 1] %Pondération de la value des 2 targets

temps = 1000

RTPlot2(env, swarm,temps, r, w, ABC, pondeTarg)
