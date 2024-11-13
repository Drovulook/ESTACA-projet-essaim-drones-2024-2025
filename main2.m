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

swarm.Drones{1}.posState = [0 1 1];
swarm.Drones{2}.posState = [0 0 0];
swarm.Drones{3}.posState = [1 0 0];
swarm.Drones{4}.posState = [1 1 0];

% figure;
% testplot(swarm)

swarm.Target = [10 10 10 ; -5 -7 30];
r = [2 4 50];
w = [-10 2 50 30];
temps = 1000
ABC = [1 1 1] %Influence sur le vecteur vitesse de : l'environnement ; la vitesse du drone a t-1 (maniabilité) ; la target
pondeTarg = [1000 1] %Pondération des 2 targets

RTPlot2(env, swarm,temps, r, w, ABC, pondeTarg)
