clear all;
close all;
clc;

%% 1) Create the Environment
env = Environment(10, 200, ...
    [-200, 200, 200, -200], ...
    [-200, -200, 200, 200], ...
    [0, 0, 0, 0]);

%% 2) Load Obstacles, Targets, and Zones from CSV
defineTargetsFromFile(env, "scenar1_targets.csv");
defineZonesFromFile(env,  "scenar1_zones.csv");

homeBaseCoord = [0, 0, 0];
temps = 100000;
traceSize = 50;

swarm = SwarmManager(env, temps);


%% 3) Load DroneModels and Fleet, then build the swarm
dronemodels = readtable('dronemodels.csv', ...
    'Delimiter', ',', 'VariableNamingRule', 'preserve');

fleet = readtable('scenar1_fleet.csv', ...
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
end

%% 4) Example Waypoints and Target Setup
Waypoints = [0 50 100;
             0 0 50;
             100 100 50;
             100 -100 100;
             -100 -100 50;
             -100 100 100;
             -100 -10 10];

for i = 1:length(swarm.Drones)
    swarm.Drones{i}.setWP(Waypoints);
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
RTPlot2(env, swarm, dt, temps, Target, traceSize);







%% ------------------------------------------------------------------------
%% Helper functions for reading CSV files: Obstacles, Targets, Zones
%% ------------------------------------------------------------------------
function defineObstaclesFromFile(env, filename)
    % Load obstacles from CSV and add them to env.ObstaclesList
    T = readtable(filename, 'Delimiter', ',', 'VariableNamingRule','preserve');
    
    % For example, we assume columns: Name,Type,Category,CenterX,CenterY,CenterZ,
    %                                Dim1,Dim2,Dim3,TiltAngle,Status
    needed = ["Name","Type","Category","CenterX","CenterY","CenterZ", ...
              "Dim1","Dim2","Dim3","TiltAngle","Status"];
    for c = 1:numel(needed)
        if ~ismember(needed(c), T.Properties.VariableNames)
            error('Missing column "%s" in "%s".', needed(c), filename);
        end
    end

    for i = 1:height(T)
        name   = T.Name{i};
        otype  = T.Type{i};
        cat    = T.Category{i};
        center = [T.CenterX(i), T.CenterY(i), T.CenterZ(i)];
        dims   = [T.Dim1(i),    T.Dim2(i),    T.Dim3(i)];
        tilt   = T.TiltAngle(i);
        stat   = T.Status{i};

        newObstacle = Obstacle(name, otype, cat, center, dims, tilt, stat, env);
        env.addObstacle(newObstacle);
    end
    fprintf('Loaded %d obstacles from "%s".\n', height(T), filename);
end

function defineTargetsFromFile(env, filename)
    % Load targets from CSV and add them to env.TargetsList
    T = readtable(filename, 'Delimiter', ',', 'VariableNamingRule','preserve');
    
    % For example, columns: Name,CenterX,CenterY,CenterZ,Status,ObservabilityScore
    needed = ["Name","CenterX","CenterY","CenterZ","Status","ObservabilityScore"];
    for c = 1:numel(needed)
        if ~ismember(needed(c), T.Properties.VariableNames)
            error('Missing column "%s" in "%s".', needed(c), filename);
        end
    end

    for i = 1:height(T)
        name  = T.Name{i};
        pos   = [T.CenterX(i), T.CenterY(i), T.CenterZ(i)];
        stat  = T.Status{i};
        score = T.ObservabilityScore(i);

        newTarget = Target(name, pos, stat, score, env);
        env.addTarget(newTarget);
    end
    fprintf('Loaded %d targets from "%s".\n', height(T), filename);
end

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
