tic

clear all;
close all;
clc;

%% 1) Create the Environment
env = Environment(10, 200, ...
    [-200, 200, 200, -200], ...
    [-200, -200, 200, 200], ...
    [0, 0, 0, 0]);

%% 2) Load Obstacles, Targets, and Zones from CSV
defineTargetsFromFile(env, "targets_test.csv");
defineZonesFromFile(env,  "zones_test.csv");

runwayHeading = 1;
homeBaseCoord = [0, 0, 0];
temps = 20000;
traceSize = 70;

swarm = SwarmManager(env, temps);

swarm.runwayHeading = runwayHeading;
defineWaypointsDecollageFromFile(swarm, runwayHeading, "waypointsdecollage.csv");
defineWaypointsAtterrissageFromFile(swarm, runwayHeading, "waypointsatterrissage.csv");


%% 3) Load DroneModels and Fleet, then build the swarm
dronemodels = readtable('dronemodels.csv', ...
    'Delimiter', ',', 'VariableNamingRule', 'preserve');

fleet = readtable('fleet_test.csv', ...
    'Delimiter', ',', 'VariableNamingRule', 'preserve');

for iRow = 1:height(fleet)
    currentModel = fleet.Model{iRow};
    idx = strcmp(dronemodels.Model, currentModel);
    if ~any(idx)
        error('No matching model found in dronemodels.csv for: %s', currentModel);
    end

    modelRow = dronemodels(idx, :);
    modelName = fleet.Name{iRow};
    dType = modelRow.Type{1};  % e.g. 'multirotor' or 'fixedwing'

    % Add the drone to the swarm
    swarm.addDrone(dType, modelName, modelRow, homeBaseCoord);
end

% Example: disable waypoint-following by default
for i = 1:length(swarm.Drones)
    swarm.Drones{i}.mode_Follow_waypoint = false;
    swarm.Drones{i}.TOheading = runwayHeading;
end

%% 4) Example Waypoints and Target Setup
% Waypoints = [0 50 100;
%              0 0 50;
%              100 100 50;
%              100 -100 100;
%              -100 -100 50;
%              -100 100 100;
%              -100 -10 10];

for i = 1:length(swarm.Drones)
    % swarm.Drones{i}.setWP(Waypoints);
    swarm.Drones{1}.mode_Follow_waypoint = true;
    swarm.Drones{1}.setPhase('stand-by');
end

% Example: assign targets and groups
Target = [0 0 75];
swarm.Drones{end}.setTargetGroup(2);

swarm.AliveDrones;

% Swarm parameters
swarm.r = [30 60 100];
swarm.swarm_weights = [1.4 0.8];
swarm.weights = [0.5 1.2 1 10] / 10;

%% 5) Run the RTPlot2 simulation
dt = 0.1;

%bat initiale
for i=1:size(swarm.Drones,2)
    swarm.Drones{i}.setActualCapacity(25); %capacité en pourcents
end

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
    if swarm.Drones{i}.Type=="multirotor"
        plot(vecnorm(swarm.Drones{i}.speedLog'))
    end
end
title("Vitesses")

subplot(3,1,2)
hold on
for i=1:size(swarm.Drones,2)
    if swarm.Drones{i}.Type=="multirotor"
        plot(swarm.Drones{i}.powerLog)
    end
end
title("Puissance")
drones_capacity=[];
subplot(3,1,3)
hold on
capacity=zeros(size(swarm.Drones{1}.powerLog,2),size(swarm.Drones,2));
for i=1:size(swarm.Drones,2)
    if swarm.Drones{i}.Type=="multirotor"
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



%% ------------------------------------------------------------------------
%% Helper functions for reading CSV files: Obstacles, Targets, Zones
%% ------------------------------------------------------------------------
function defineZonesFromFile(env, filename)
    % Load zones from CSV and add them to env.ZonesList
    T = readtable(filename, 'Delimiter', ',', 'VariableNamingRule','preserve');

    % For example, columns: Name,Type,Category,CenterX,CenterY,CenterZ,
    %                       Dim1,Dim2,Dim3,Tilt,Status
    needed = ["Name","Type","Category","CenterX","CenterY","CenterZ", ...
              "Dim1","Tilt","Status"];
    for c = 1:numel(needed)
        if ~ismember(needed(c), T.Properties.VariableNames)
            error('Missing column "%s" in "%s".', needed(c), filename);
        end
    end

    for i = 1:height(T)
        name   = T.Name{i};
        ztype  = T.Type{i};
        cat    = T.Category{i};
        center = [T.CenterX(i), T.CenterY(i), T.CenterZ(i)];
        dims   = T.Dim1(i);
        tilt   = T.Tilt(i);
        stat   = T.Status{i};

        newZone = Zone(name, ztype, cat, center, dims, tilt, stat, env);
        env.addZone(newZone);
    end
    fprintf('Loaded %d zones from "%s".\n', height(T), filename);
end

function defineWaypointsDecollageFromFile(swarm, heading, filename)
    T = readtable(filename, 'Delimiter', ',', 'VariableNamingRule', 'preserve');
    requiredCols = ["X","Y","Z"];
    for c = 1:numel(requiredCols)
        if ~ismember(requiredCols(c), T.Properties.VariableNames)
            error('Missing column "%s" in "%s".', requiredCols(c), filename);
        end
    end
    
    computedX = T.X * cosd(heading) - T.Y * sind(heading);
    computedY = T.X * sind(heading) + T.Y * cosd(heading);
    swarm.TO_WP = [computedX, computedY, T.Z];
    fprintf('%d waypoints de décollage chargés depuis "%s".\n', height(T), filename);
end

function defineWaypointsAtterrissageFromFile(swarm, heading, filename)
    T = readtable(filename, 'Delimiter', ',', 'VariableNamingRule', 'preserve');
    requiredCols = ["X","Y","Z"];
    for c = 1:numel(requiredCols)
        if ~ismember(requiredCols(c), T.Properties.VariableNames)
            error('Missing column "%s" in "%s".', requiredCols(c), filename);
        end
    end

    computedX = T.X * cosd(heading) - T.Y * sind(heading);
    computedY = T.X * sind(heading) + T.Y * cosd(heading);
    swarm.landing_WP = [computedX, computedY, T.Z];
    fprintf('%d waypoints d''atterrissage chargés depuis "%s".\n', height(T), filename);
end
