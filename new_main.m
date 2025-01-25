clear all;
close all;
clc;

env = Environment(10, 200, [-200, 200, 200, -200], ...
                 [-200, -200, 200, 200], [0, 0, 0, 0]);
setupZones;
homeBaseCoord = [0, 0, 0];
temps = 100000;
traceSize = 50;

swarm = SwarmManager(env, temps);


% 1) Read the drone models from CSV
dronemodels = readtable('dronemodels.csv', 'Delimiter', ',', 'VariableNamingRule', 'preserve');

% 2) Read the initial fleet from CSV (has columns e.g. Name, Model, etc.)
fleet = readtable('fleet.csv', 'Delimiter', ',', 'VariableNamingRule', 'preserve');

% 3) For each row in fleet, find the matching model in dronemodels,
%    get the 'Type' (multirotor or fixedwing), then add the drone.
for iRow = 1:height(fleet)
    currentModel = fleet.Model{iRow};
    idx = strcmp(dronemodels.Model, currentModel);
    if ~any(idx)
        error('No matching model found in dronemodels.csv for: %s', currentModel);
    end
    
    modelRow = dronemodels(idx,:);          % The row of parameters
    modelName = fleet.Name{iRow};
    dType    = modelRow.Type{1};            % 'multirotor' or 'fixedwing'
    
    swarm.addDrone(dType, modelName, modelRow, homeBaseCoord);   % Same signature as before
end

for i = 1:length(swarm.Drones)
    swarm.Drones{i}.mode_Follow_waypoint = false; 
end


Waypoints = [0 50 100; 0 0 50; 100 100 50 ; 100 -100 100 ; ...
            -100 -100 50 ; -100 100 100 ; -100 -10 10]; 
for i = 1:length(swarm.Drones)
    swarm.Drones{i}.Waypoints = Waypoints;
end

% POUR BAPTISTE, switch des targets et groupes d'observation
swarm.targets = [1 0 0; 140 140 30; 6 8 7]; % 3 groupes de target
Target = [0 0 75];
swarm.update_target(Target, 1);
swarm.Drones{end}.setTargetGroup(2);
swarm.Drones{1}.mode_Follow_waypoint = true;
swarm.AliveDrones;

swarm.r = [30 60 100]; 
swarm.swarm_weights = [1.4 0.8]; 
swarm.weights = [0.5 1.2 1 10] / 10;

dt = 0.1;
RTPlot2(env, swarm, dt, temps, Target, traceSize);
