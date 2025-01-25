close all
clear all

tic

env = Environment(10, 200, [-1000, 1000, 1000, -1000], ...
    [-1000, -1000, 1000, 1000], [0, 0, 0, 0]);
setupZones;
homeBaseCoord = [0, 0, 0];
temps = 10000;

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

% % If you still want to use numMultirotor and numFixedwing below,
% % you can keep them OR redefine them from the CSV if needed:
% numMultirotor = sum(strcmpi(dronemodels.Type(fleet.Model == dronemodels.Model), 'multirotor'));
% numFixedwing  = sum(strcmpi(dronemodels.Type(fleet.Model == dronemodels.Model), 'fixedwing'));
%
% % Set mode_Follow_waypoint = false for each existing multirotor/fixed wing
% % (keeping the same lines from your original code)
% for i = 1:numMultirotor
%     swarm.MultiRotor{i}.mode_Follow_waypoint = false;
% end
%
% for i = 1:numFixedwing
%     swarm.FixedWing{i}.mode_Follow_waypoint = false;
% end

% =====================================================================
%            EVERYTHING ELSE BELOW IS UNCHANGED
% =====================================================================

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

%% Boucle de simulation

k = 0;
n_drone = size(swarm.Drones, 2);
while true
    % tic
    swarm.update_speeds(dt);

    drone_pos = zeros(n_drone, 3);
    drone_speed = zeros(n_drone, 3);

    for i = 1:n_drone
        drone_pos(i, :) = swarm.Drones{i}.posState;
        drone_speed(i, :) = swarm.Drones{i}.speedState;
    end

    k = k + 1;
    if k == temps
        break;
    end
end
swarm.drones_pos_history_matrix = cat(3, zeros(n_drone, 3, 15), swarm.drones_pos_history_matrix);



elapsedtime=toc





figure
subplot(3,1,1)
hold on
for i=1:size(swarm.Drones,2)
    if swarm.Drones{i}.Type=="fixedwing"
        plot(vecnorm(swarm.Drones{i}.speedLog'))
    end
end
title("Vitesses")

subplot(3,1,2)
hold on
for i=1:size(swarm.Drones,2)
    if swarm.Drones{i}.Type=="fixedwing"
        plot(swarm.Drones{i}.powerLog)
    end
end
title("Puissance")
drones_capacity=[];
subplot(3,1,3)
hold on
capacity=zeros(size(swarm.Drones{1}.powerLog,2),size(swarm.Drones,2));
for i=1:size(swarm.Drones,2)
    if swarm.Drones{i}.Type=="fixedwing"
        drones_capacity=[drones_capacity swarm.Drones{i}.remainingCapacity];
        capacity_consumption=power(swarm.Drones{i}.powerLog/swarm.Drones{i}.NominalVoltage, swarm.Drones{i}.k_peukert)*dt/3600*swarm.Drones{i}.NominalVoltage;
        % Wh
        autonomy=zeros(size(capacity_consumption,2),1);
        for j=1:size(capacity_consumption,2)
            capacity(j,i)=sum(capacity_consumption(1:j));
            % I=mean(capacity_consumption(1:j))*3600/swarm.Drones{i}.batteryNominalVoltage/swarm.Drones{i}.yield;
            % autonomy(j)=(swarm.Drones{i}.maxCapacity-capacity(i,j))/(I^swarm.Drones{i}.k_peukert);
            % autonomy(j)=(swarm.Drones{i}.maxCapacity-capacity(i,j))/(mean(capacity_consumption(1:j))*3600);
        end
        plot(swarm.Drones{i}.NominalCapacity-capacity(:,i))
        % plot(autonomy);
    end
end
title("Capacité consommée (Wh)");

% phases=[];
% for i=1:size(swarm.Drones,2)
%     [swarm.Drones{i}.phase ' ' swarm.Drones{i}.Type]
% end