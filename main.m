% main.m
% Script principal pour initialiser l'environnement, créer l'essaim et exécuter la simulation

clear; close all; clc; % Nettoyer l'espace de travail, fermer les figures et effacer la console

%% Configuration de l'environnement
env = Environment(10, 200, [-150, 150, 150, -150], [-150, -150, 150, 150], [0, 0, 0, 0]);
setupZones; % Configuration des zones de l'espace aérien

%% Configuration du gestionnaire d'essaim et des paramètres de l'essaim
swarm = SwarmManager(env.Env); % Initialiser le gestionnaire d'essaim avec l'environnement
numMultirotor = 3; % Nombre de drones multirotors
numFixedwing = 4; % Nombre de drones à voilure fixe

%% Ajout des drones à l'essaim
% Ajouter les drones multirotors à l'essaim, placés à la coordonnée de la base
for i = 1:numMultirotor
    swarm.addDrone('multirotor', homeBaseCoord);
end
% Ajouter les drones à voilure fixe à l'essaim
for i = 1:numFixedwing
    swarm.addDrone('fixedwing', homeBaseCoord);
end

% Essai de retrait de drone (arbitrairement drones 1 et 5)
swarm.removeDrone(1);
swarm.removeDrone(5);

% Essai de réallocation de destination pour un drone spécifique
swarm.setDestination(2, [50, 50, 50]);

%% Affichage 
% Affichage de la situation de simulation en temps réel
realTimePlot(env, swarm);

%% En cours de développement
% %% Définir la zone de surveillance et générer des points de passage
% area = [0, 100, 0, 100]; % Zone de balayage [xmin, xmax, ymin, ymax]
% altitude = -10; % Altitude dans le système NED (altitude négative)
% spacing = 10; % Espacement entre les trajectoires
% waypoints = generateLawnmowerPattern(area, altitude, spacing); % Générer un modèle en "lawnmower"
% 
% %% Assigner les points de passage aux drones
% % Distribuer les points de passage entre les drones de l'essaim
% swarm.distributeWaypoints(waypoints);
% 
% %% Paramètres de simulation
% dt = 0.1; % Pas de temps
% simulationTime = 200; % Temps total de simulation en secondes
% steps = simulationTime / dt; % Nombre d'itérations
% 
% %% Exécuter la simulation
% for t = 1:steps
%     % Mettre à jour l'essaim en fonction du pas de temps
%     swarm.update(dt);
% 
%     % Comportement dynamique (optionnel)
%     if t == 500
%         % Changer la destination ou les points de passage
%         newDestination = [50, 50, -10];
%         swarm.setDestination(1, newDestination); % Redéfinir la destination pour le drone multirotor
%         % Pour le drone à voilure fixe, réassigner des points de passage si nécessaire
%     end
% 
%     % Avancer l'environnement
%     advance(env);
% end
% 
% %% Visualisation des trajectoires
% swarm.plotTrajectories(); % Tracer les trajectoires des drones
