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
end


zone_size = 50;
min_distance = 0.5;
for i = 1:numMultirotor
    valid_position = false;
    while ~valid_position
        new_pos = [rand*zone_size, rand*zone_size, rand*zone_size];
        
        valid_position = true;
        for j = 1:i-1
            dist = norm(new_pos - swarm.Drones{j}.posState);
            if dist < min_distance
                valid_position = false;
                break; 
            end
        end
    end
    swarm.MultiRotor{i}.posState = new_pos;
    swarm.MultiRotor{i}.mode_Follow_waypoint = false; % Follow des wp ou de la target
end


% Ajouter les drones multirotors à l'essaim, placés à la coordonnée de la base
for i = 1:numFixedwing
    swarm.addDrone('fixedwing', homeBaseCoord);
end
for i = 1:numFixedwing
    valid_position = false;
    while ~valid_position
        new_pos = [rand*zone_size, rand*zone_size, rand*zone_size];
        valid_position = true;
        for j = 1:i-1
  
            dist = norm(new_pos - swarm.Drones{j}.posState);
            if dist < min_distance
                valid_position = false;
                break; 
            end
        end
    end
    swarm.FixedWing{i}.posState = new_pos;
    swarm.FixedWing{i}.mode_Follow_waypoint = false; % Follow des wp ou de la target
end

Waypoints = [0 50 100; 0 0 50; 100 100 50 ; 100 -100 100 ; -100 -100 50 ; -100 100 100 ; -100 -10 10]; 
for i = 1:length(swarm.Drones)
    swarm.Drones{i}.Waypoints = Waypoints;
end


Target = [0 0 75];
swarm.update_target(Target); 


swarm.r = [30 60 100]; % Rayons Répulsion, attraction au sein de l'essaim
swarm.swarm_weights = [1.4 0.8]; % Pondérations répulsion, attraction au sein de l'essaim
swarm.weights = [0.5 1.2 1 10] / 10; % pondération essaim, inertie, target, évitement


dt = 0.1;
RTPlot2(env, swarm, dt, temps, Target, traceSize);

