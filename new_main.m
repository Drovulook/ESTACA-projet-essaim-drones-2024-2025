clear all;
close all;
clc;


env = Environment(10, 200, [-200, 200, 200, -200], [-200, -200, 200, 200], [0, 0, 0, 0]);
setupZones;
homeBaseCoord = [0, 0, 0];
temps = 100000;
traceSize = 20;

swarm = SwarmManager(env, temps);
numMultirotor = 10; % Nombre de drones multirotors
numFixedwing = 5; % Nombre de drones fixedwing

for i = 1:numMultirotor
    swarm.addDrone('multirotor', homeBaseCoord);
    swarm.MultiRotor{i}.mode_Follow_waypoint = false; % Follow des wp ou de la target
end


% Ajouter les drones multirotors à l'essaim, placés à la coordonnée de la base
for i = 1:numFixedwing
    swarm.addDrone('fixedwing', homeBaseCoord);
    swarm.FixedWing{i}.mode_Follow_waypoint = false; % Follow des wp ou de la target
end



Waypoints = [0 50 100; 0 0 50; 100 100 50 ; 100 -100 100 ; -100 -100 50 ; -100 100 100 ; -100 -10 10]; 
for i = 1:length(swarm.Drones)
    swarm.Drones{i}.Waypoints = Waypoints;
end


% POUR BAPTISTE, switch des targets et groupes d'observation
swarm.targets = [1 0 0; 140 140 30; 6 8 7]; % 3 groupes de target
Target = [0 0 75];
swarm.update_target(Target, 1); % Pour modifier la taget du pool n°1
swarm.Drones{end}.setTargetGroup(2); % Pour modifier le pool du dernier drone (il passe en pool 2)
swarm.Drones{1}.mode_Follow_waypoint = true; % Le premier drone passe en follow waypoint
swarm.AliveDrones; % Liste des drones vivants


swarm.r = [30 60 100]; % Rayons Répulsion, attraction au sein de l'essaim
swarm.swarm_weights = [1.4 0.8]; % Pondérations répulsion, attraction au sein de l'essaim
swarm.weights = [0.5 1.2 1 10] / 10; % pondération essaim, inertie, target, évitement


dt = 0.1;
RTPlot2(env, swarm, dt, temps, Target, traceSize);