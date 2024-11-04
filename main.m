% main.m
% Script principal pour initialiser l'environnement, créer l'essaim et exécuter la simulation

clear; close all; clc;

%% Configuration de l'environnement
env = setupEnvironment();
baseCoordinates = [0, 0, 0];

% Création du gestionnaire d'essaim et des paramètres de l'essaim
swarm = SwarmManager(env);
numMultirotor = 3;
numFixedwing = 4;

%% Chargement des paramètres des drones
%load('data/params.mat'); % Contient des paramètres comme la masse, l'inertie, etc.

%% Ajouter des drones à l'essaim
% Ajouter un drone multirotor
for i = 1:numMultirotor
    swarm.addDrone('multirotor', baseCoordinates);
end
% Ajouter un drone à voilure fixe
for i = 1:numFixedwing
    swarm.addDrone('fixedwing', baseCoordinates);
end
% Essai de retrait de drone (arbitrairement 1 et 5)
swarm.removeDrone(1);
swarm.removeDrone(5);

% Affichage de la situation
show3D(env);

%% EN COURS DE DEBUG
swarm.setDestination(2, [50, 50, 50]); 

%% ATTENTE DE DEBUG PRECEDENT
% %% Définir la zone de surveillance et générer des points de passage
% area = [0, 100, 0, 100]; % [xmin, xmax, ymin, ymax]
% altitude = -10; % Système de coordonnées NED (altitude négative)
% spacing = 10; % Espacement entre les trajectoires
% waypoints = generateLawnmowerPattern(area, altitude, spacing);
% 
% %% Assigner les points de passage aux drones
% % Distribuer les points de passage entre les drones
% swarm.distributeWaypoints(waypoints);
% 
% %% Paramètres de simulation
% dt = 0.1; % Pas de temps
% simulationTime = 200; % Temps total de simulation
% steps = simulationTime / dt;
% 
% %% Exécuter la simulation
% for t = 1:steps
%     % Mettre à jour l'essaim
%     swarm.update(dt);
% 
%     % Comportement dynamique (optionnel)
%     if t == 500
%         % Changer la destination ou les points de passage
%         newDestination = [50, 50, -10];
%         swarm.setDestination(1, newDestination); % Pour le drone multirotor
%         % Pour le drone à voilure fixe, réassigner des points de passage si nécessaire
%     end
% 
%     % Avancer l'environnement
%     advance(env);
% end
% 
% %% Visualisation
% swarm.plotTrajectories();
