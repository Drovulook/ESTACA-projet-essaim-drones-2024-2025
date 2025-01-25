classdef IHM_FINAL < matlab.apps.AppBase

    % Properties that correspond to app components
    properties (Access = public)
        UIFigure                       matlab.ui.Figure
        DviewMenu                      matlab.ui.container.Menu
        InfoMenu                       matlab.ui.container.Menu
        Menu                           matlab.ui.container.Menu
        Menu2                          matlab.ui.container.Menu
        Panel                          matlab.ui.container.Panel
        Lamp                           matlab.ui.control.Lamp
        StartButton                    matlab.ui.control.Button
        PauseButton                    matlab.ui.control.Button
        ResetButton                    matlab.ui.control.Button
        DronesperdusEditField          matlab.ui.control.NumericEditField
        DronesperdusEditFieldLabel     matlab.ui.control.Label
        EventLoggingLabel              matlab.ui.control.Label
        TempscoulLabel                 matlab.ui.control.Label
        DronesnondploysEditField       matlab.ui.control.NumericEditField
        DronesnondploysEditFieldLabel  matlab.ui.control.Label
        DronesdploysEditField          matlab.ui.control.NumericEditField
        DronesdploysEditFieldLabel     matlab.ui.control.Label
        dtSlider                       matlab.ui.control.Slider
        dtSliderLabel                  matlab.ui.control.Label
        tempscouleEditField            matlab.ui.control.EditField
        Console                        matlab.ui.control.TextArea
        TabGroup                       matlab.ui.container.TabGroup
        ObservationTab                 matlab.ui.container.Tab
        PanDNButton                    matlab.ui.control.Button
        PanUPButton                    matlab.ui.control.Button
        RotationGAButton               matlab.ui.control.Button
        RotationDRButton               matlab.ui.control.Button
        TranslationGAButton            matlab.ui.control.Button
        TranslationARButton            matlab.ui.control.Button
        TranslationDRButton            matlab.ui.control.Button
        TranslationAVButton            matlab.ui.control.Button
        FlotteTab                      matlab.ui.container.Tab
        SuppressionDroneButton         matlab.ui.control.Button
        ModificationDroneButton        matlab.ui.control.Button
        AjoutDroneButton               matlab.ui.control.Button
        UIFlotteTable                  matlab.ui.control.Table
        CiblesTab                      matlab.ui.container.Tab
        SuppressionCibleButton         matlab.ui.control.Button
        ModificationCibleButton        matlab.ui.control.Button
        AjoutCibleButton               matlab.ui.control.Button
        UICibleTable                   matlab.ui.control.Table
        ZonesTab                       matlab.ui.container.Tab
        SuppressionZoneButton          matlab.ui.control.Button
        ModificationZoneButton         matlab.ui.control.Button
        AjoutZoneButton                matlab.ui.control.Button
        UIZoneTable                    matlab.ui.control.Table
        ObstaclesTab                   matlab.ui.container.Tab
        SuppressionObstacleButton      matlab.ui.control.Button
        ModificationObstacleButton     matlab.ui.control.Button
        AjoutObstacleButton            matlab.ui.control.Button
        UIObstacleTable                matlab.ui.control.Table
        ComportementTab                matlab.ui.container.Tab
        UIAxes3D                       matlab.ui.control.UIAxes
        UIAxes2D                       matlab.ui.control.UIAxes
    end

    
    properties (Access = private)
        plotHandles  % struct to store scatter, quiver, trace objects
        simTimer     % MATLAB timer for repeated updates

        env                  % Environment object
        swarm                % SwarmManager object
        DroneModels
        %simTimer             % Timer for the simulation loop
        simulationRunning    logical = false
        simulationPaused     logical = false
    
        % For real-time plotting
        dronePlotHandles     = []   % Store handles to drone plots (2D/3D)
        traceSize            = 20
        simulationTime       double = 0
        maxSimulationTime    double = 100000
    end
    
    methods (Access = public)
    
        function setupSimulation(app)
            clc
        
            % 1) Simulation parameters
            app.traceSize = 200;
            app.maxSimulationTime = 100000;
        
            % 2) Reset UI elements
            app.Console.Value         = {''};
            app.tempscouleEditField.Value = "0";
            app.simulationTime        = 0;
            app.DronesdploysEditField.Value    = 0;
            app.DronesnondploysEditField.Value = 0;
            app.DronesperdusEditField.Value    = 0;
        
            % 3) Create Environment & obstacles
            app.env = Environment(10, 200, ...
                [-200, 200, 200, -200], ...
                [-200, -200, 200, 200], ...
                [0,0,0,0]);
        
            app.defineObstaclesFromFile("obstaclelist.csv");
        
            % 4) Load Drone Models from CSV
            app.defineDroneModelsFromFile("dronemodels.csv");
        
            % 5) Create Swarm
            app.swarm = SwarmManager(app.env, app.maxSimulationTime);
        
            % Load the fleet from a CSV instead of manually adding drones
            homeBaseCoord = [0 0 0];  % or wherever your "base" is
            app.defineFleetFromFile("fleet.csv", homeBaseCoord);
        
            % (Optionally remove or comment out any old code like: 
            %   for i=1:10, app.swarm.addDrone('multirotor', [0 0 0]); end 
            %   etc.)
        
            % If you still want to set custom properties (like follow_waypoint):
            %   for i=1:numel(app.swarm.Drones)
            %       if strcmpi(app.swarm.Drones{i}.type, 'multirotor')
            %           app.swarm.Drones{i}.mode_Follow_waypoint = false;
            %       end
            %   end
        
            % 6) Possibly define default Waypoints or Targets,
            %    or do so after reading extra columns from your CSV
        
            % 7) Clear tables, reset axes, and initialize the real-time plot
            app.UIFlotteTable.Data = {};
            app.UIZoneTable.Data   = {};
            app.UICibleTable.Data  = {};
        
            cla(app.UIAxes3D, "reset");
        
            title(app.UIAxes3D, 'Affichage 3D');
            xlabel(app.UIAxes3D, 'X'); ylabel(app.UIAxes3D, 'Y'); zlabel(app.UIAxes3D, 'Z');
        
            title(app.UIAxes2D, 'Affichage 2D');
            xlabel(app.UIAxes2D, 'X'); ylabel(app.UIAxes2D, 'Y');
            zticks(app.UIAxes2D, []);
        
            % 8) Suppose you have a default target
            Target = [0 0 75];
            app.plotHandles = RTPlot2_init(app, app.UIAxes3D, app.env, app.swarm, Target, app.traceSize);
        
            % done
        end

        function startSimulationTimer(app)
            if isempty(app.simTimer) || ~isvalid(app.simTimer)
                % Create a new timer
                app.simTimer = timer( ...
                    'ExecutionMode','fixedRate', ...
                    'Period', app.dtSlider.Value, ...
                    'BusyMode','drop', ...
                    'TimerFcn', @(~,~)updateSimulation(app));
            else
                % If it already exists and is running, stop it first
                if strcmpi(app.simTimer.Running, 'on')
                    stop(app.simTimer);
                end
                app.simTimer.Period = app.dtSlider.Value;
                app.simTimer.TimerFcn = @(~,~)updateSimulation(app);
            end
        
            start(app.simTimer);
            app.simulationRunning = true;
            app.simulationPaused  = false;
            app.Lamp.Color = [0 1 0];  % Green
            addConsoleMsg(app, "Simulation started.");
        end

        function updateSimulation(app)
            if ~app.simulationPaused
                dt = 0.1; %app.dtSlider.Value
                app.simulationTime = app.simulationTime + dt;
        
                % Advance swarm by dt
                app.swarm.update_speeds(dt);
        
                % Update the visuals (no 'Target' argument here)
                RTPlot2_update(app, app.plotHandles, app.swarm);
                % drawnow limitrate;
        
                % Update UI fields
                app.tempscouleEditField.Value = sprintf("%.1f", app.simulationTime);
        
                % Optional checks / updates ...
                if app.simulationTime >= app.maxSimulationTime
                    stopSimulationTimer(app);
                end
            end
        end



        
        function defineDroneModelsFromFile(app, filename)
            % DEFINEDRONEMODELSFROMFILE  Load drone models from a CSV file
            % and store them in 'app.DroneModels'.
            %
            %   defineDroneModelsFromFile(app, "dronemodels.csv")
            %   or pass the file path of your choice.
        
            % 1) Read the CSV file as a table
            T = readtable(filename, 'Delimiter', ',');
        
            % 2) Check that the needed columns exist (optional but safer)
            requiredCols = {'Model', 'Type', 'Autonomy', 'RechargeTime'};
            for c = 1:numel(requiredCols)
                if ~ismember(requiredCols{c}, T.Properties.VariableNames)
                    error('CSV file must contain a column named "%s".', requiredCols{c});
                end
            end
        
            % 3) Validate the data types of the columns (optional but recommended)
            if ~iscellstr(T.Model)
                error('The "Model" column must contain text values.');
            end
            if ~iscellstr(T.Type)
                error('The "Type" column must contain text values (e.g., "Fixed Wing" or "Multirotor").');
            end
            if ~isnumeric(T.Autonomy) || any(T.Autonomy <= 0)
                error('The "Autonomy" column must contain positive numeric values.');
            end
            if ~isnumeric(T.RechargeTime) || any(T.RechargeTime <= 0)
                error('The "RechargeTime" column must contain positive numeric values.');
            end
        
            % 4) Store the table in the app's properties
            app.DroneModels = T;
        
            fprintf('Loaded %d drone models from "%s".\n', height(T), filename);
        end

        function defineObstaclesFromFile(app, filename)
            % DEFINEOBSTACLESFROMFILE  Load obstacles from a CSV file
            % and add them to 'app.env'.
            %
            %   defineObstaclesFromFile(app, "obstaclelist.csv")
            %   or pass the file path of your choice.
            
            % 1) Read the CSV file as a table
            T = readtable(filename, 'Delimiter', ',');
            
            % 2) Check that the needed columns exist (optional but safer)
            requiredCols = {'ZoneType','Category','CenterX','CenterY','CenterZ','DimX','DimY','DimZ','TiltAngle'};
            for c = 1:numel(requiredCols)
                if ~ismember(requiredCols{c}, T.Properties.VariableNames)
                    error('CSV file must contain a column named "%s".', requiredCols{c});
                end
            end
            
            % 3) Loop over rows in the table
            nRows = height(T);
            for iRow = 1:nRows
                % Extract each parameter from the table row
                zoneType = T.ZoneType{iRow};      % string or char
                category = T.Category{iRow};      % 'A', 'P', 'M', etc.
                
                center   = [ T.CenterX(iRow), T.CenterY(iRow), T.CenterZ(iRow) ];
                dims     = [ T.DimX(iRow),   T.DimY(iRow),   T.DimZ(iRow) ];
                tilt     = T.TiltAngle(iRow);
                
                % 4) Construct a new Zone
                newZone = Zone(zoneType, category, center, dims, tilt, app.env);
                
                % 5) Add to the environment
                app.env.addZone(newZone);
            end
            
            fprintf('Loaded %d obstacles from "%s".\n', nRows, filename);
        end

        function defineFleetFromFile(app, filename, homeBaseCoord)
            % DEFINEFLEETFROMFILE  Load a list of drones from a CSV file,
            %   ignoring any coordinates in the file and using the same
            %   homeBaseCoord for all.
            %
            %   Required CSV column(s):
            %       - 'DroneType': e.g. 'multirotor', 'fixedwing'
            %
            %   Example CSV:
            %       DroneType
            %       multirotor
            %       fixedwing
            %       multirotor
            %
            %   Then we add each drone at the same (homeBaseCoord).
            
            % 1) Read the CSV file as a table
            T = readtable(filename, 'Delimiter', ',');
    
            % 2) Check for required column(s)
            requiredCols = {'DroneType'};
            for c = 1:numel(requiredCols)
                if ~ismember(requiredCols{c}, T.Properties.VariableNames)
                    error('CSV file must contain a column named "%s".', requiredCols{c});
                end
            end
            
            % 3) Loop over each row in the table
            nRows = height(T);
            for iRow = 1:nRows
                dType = T.DroneType{iRow};  % e.g. 'multirotor' or 'fixedwing'
                
                % Add the drone to the swarm with the same initial coordinates
                app.swarm.addDrone(dType, homeBaseCoord);
            end
            
            fprintf('Loaded %d drones (all at homeBaseCoord) from "%s".\n', ...
                nRows, filename);
        end




        function handles = RTPlot2_init(app, ax, env, swarm, Target, traceSize)
            % RTPlot2_init  Initialize 3D plots for environment, zones, waypoints, and drones.
            %
            %   (Only the relevant changed lines are shown.)
        
            %== 1) Basic Axes Setup =======================================
            rotate3d(ax, 'on'); 
            grid(ax, 'on');
            hold(ax, 'on');
            
            % Instead of "axis(ax, 'equal')", let's leave it free. 
            % If you want a uniform scale, you can keep or remove 'equal'.
            % axis(ax, 'equal'); 
            
            % Choose an initial camera view
            view(ax, -75, 15);
        
            % -- MANUAL axis limits (so new objects do NOT auto-rescale) --
            set(ax, ...
                'XLim',      [-700  700], ...
                'YLim',      [-700  700], ...
                'ZLim',      [   0  700], ...
                'XLimMode',  'manual', ...
                'YLimMode',  'manual', ...
                'ZLimMode',  'manual');
        
            xlabel(ax, 'X (Nord - Sud)');
            ylabel(ax, 'Y (Ouest - Est)');
            zlabel(ax, 'Z (Altitude)');
            title(ax, 'Rendu de Simulation en Temps Réel');
        
            %== 2) Draw Ground Surface =====================================
            fill3(ax, ...
                env.GroundCoordinates.x, ...
                env.GroundCoordinates.y, ...
                env.GroundCoordinates.z, ...
                [0.5 0.5 0.5], 'FaceAlpha', 0.3);
        
            %== 2b) Draw Zones (initially) =================================
            nZones = length(env.ZonesList);
            zoneHandles = gobjects(1, nZones);
            for iZone = 1:nZones
                zone = env.ZonesList{iZone};
                zoneColor = app.pickZoneColor(zone.Category);
        
                shapeHandle = create_shape(zone.Type, zoneColor, ...
                    zone.CenterPosition, zone.Dimensions, ...
                    zone.TiltAngle, ax);
                zoneHandles(iZone) = shapeHandle;
            end
        
            %== 3) Plot the Target(s) ======================================
            nTargets = size(Target, 1);
            targetHandles = gobjects(1, nTargets);
            for k = 1:nTargets
                targetHandles(k) = scatter3(ax, ...
                    Target(k,1), Target(k,2), Target(k,3), ...
                    50, 'filled', 'MarkerFaceColor','r');
            end
        
            %== 4) Plot Each Drone (head + quiver + trace) ==================
            nDrones = numel(swarm.Drones);
            droneHeads   = gobjects(1, nDrones);
            droneQuivers = gobjects(1, nDrones);
            droneTraces  = gobjects(1, nDrones);
        
            for i = 1:nDrones
                pos = swarm.Drones{i}.posState;  % [x, y, z]
                droneHeads(i) = scatter3(ax, ...
                    pos(1), pos(2), pos(3), 50, ...
                    'filled', 'MarkerFaceColor','b');
        
                droneQuivers(i) = quiver3(ax, ...
                    pos(1), pos(2), pos(3), ...
                    0, 0, 0, 'r', 'LineWidth', 1.5, 'MaxHeadSize', 0.5);
        
                droneTraces(i) = plot3(ax, nan, nan, nan, ...
                    'Color', [0 0 1], 'LineStyle', '--');
            end
        
            %== 5) Plot the Active Waypoints (FixedWing) ====================
            nFixedWing = numel(swarm.FixedWing);
            waypointHandles = gobjects(1, nFixedWing);
            for i = 1:nFixedWing
                fw = swarm.FixedWing{i};
                if ~isempty(fw.Waypoints)
                    cwp = fw.CurrentWaypoint;
                    if cwp <= size(fw.Waypoints, 1)
                        wpPos = fw.Waypoints(cwp, :);
                        waypointHandles(i) = scatter3(ax, ...
                            wpPos(1), wpPos(2), wpPos(3), ...
                            50, 'filled', 'MarkerFaceColor','g');
                    else
                        waypointHandles(i) = scatter3(ax, NaN, NaN, NaN, ...
                            50, 'filled', 'MarkerFaceColor','g');
                    end
                else
                    waypointHandles(i) = scatter3(ax, NaN, NaN, NaN, ...
                        50, 'filled', 'MarkerFaceColor','g');
                end
            end
        
            %== 6) Package into a "handles" struct ==========================
            handles.ax               = ax;
            handles.zoneHandles      = zoneHandles;
            handles.targetHandles    = targetHandles;
            handles.droneHeads       = droneHeads;
            handles.droneQuivers     = droneQuivers;
            handles.droneTraces      = droneTraces;
            handles.waypointHandles  = waypointHandles;
        
            handles.traceSize        = traceSize;
            handles.speedVectorSize  = 2;  % adjust as needed
        
            handles.nZonesLastUpdate = nZones;
        end

        function RTPlot2_update(app, handles, swarm)
        % RTPlot2_update  Update positions (and possibly zone changes) for real-time display
        %                 WITHOUT auto-adjusting the axes.
        
            ax = handles.ax;
            speedVectorSz = handles.speedVectorSize;
            traceSize     = handles.traceSize;
        
            % --- 1) Update Drones' positions & traces ---
            nDrones = numel(swarm.Drones);
        
            for i = 1:nDrones
                pos = swarm.Drones{i}.posState;     % [x, y, z]
                spd = swarm.Drones{i}.speedState;
        
                % Update drone head (scatter3 marker)
                set(handles.droneHeads(i), ...
                    'XData', pos(1), 'YData', pos(2), 'ZData', pos(3));
        
                % Update quiver (velocity vector)
                set(handles.droneQuivers(i), ...
                    'XData', pos(1), 'YData', pos(2), 'ZData', pos(3), ...
                    'UData', spd(1)*speedVectorSz, ...
                    'VData', spd(2)*speedVectorSz, ...
                    'WData', spd(3)*speedVectorSz);
        
                % Update position trace
                if isprop(swarm.Drones{i}, 'posLog')
                    lastPositions = swarm.Drones{i}.posLog;  % Nx3 log of past positions
                    if ~isempty(lastPositions)
                        idx = max(1, size(lastPositions,1) - traceSize + 1) : size(lastPositions,1);
                        recentPositions = lastPositions(idx, :);
                        set(handles.droneTraces(i), ...
                            'XData', recentPositions(:,1), ...
                            'YData', recentPositions(:,2), ...
                            'ZData', recentPositions(:,3));
                    end
                end
            end
        
            % --- 2) Update Waypoints (FixedWing) ---
            nFixedWing = numel(swarm.FixedWing);
            for i = 1:nFixedWing
                fw = swarm.FixedWing{i};
        
                % Ensure CurrentWaypoint is at least 1 AND within the # of rows in fw.Waypoints
                if ~isempty(fw.Waypoints) ...
                        && fw.CurrentWaypoint >= 1 ...
                        && fw.CurrentWaypoint <= size(fw.Waypoints, 1)
        
                    % Safe to index Waypoints
                    cwp   = fw.CurrentWaypoint;
                    wpPos = fw.Waypoints(cwp, :);
                    set(handles.waypointHandles(i), ...
                        'XData', wpPos(1), 'YData', wpPos(2), 'ZData', wpPos(3));
                else
                    % No valid waypoint => hide the waypoint marker
                    set(handles.waypointHandles(i), ...
                        'XData', NaN, 'YData', NaN, 'ZData', NaN);
                end
            end
        
            % --- 3) Check if the set of zones changed (in case new obstacles added/removed) ---
            newZones = app.env.ZonesList;
            newZoneCount = length(newZones);
            oldZoneCount = handles.nZonesLastUpdate;
        
            if newZoneCount ~= oldZoneCount
                % Delete old zone graphics
                oldHandles = handles.zoneHandles;
                for hh = 1:length(oldHandles)
                    if isvalid(oldHandles(hh))
                        delete(oldHandles(hh));
                    end
                end
        
                % Re-draw current zones
                newZoneHandles = gobjects(1, newZoneCount);
                for iZone = 1:newZoneCount
                    zone = newZones{iZone};
                    zoneColor = app.pickZoneColor(zone.Category);
                    shapeHandle = create_shape(zone.Type, zoneColor, ...
                                               zone.CenterPosition, zone.Dimensions, ...
                                               zone.TiltAngle, ax);
                    newZoneHandles(iZone) = shapeHandle;
                end
        
                % Update stored handles and zone count
                handles.zoneHandles      = newZoneHandles;
                handles.nZonesLastUpdate = newZoneCount;
            end
        
            % --- 4) Render ---
            % No bounding box changes, so no axis adjustments here.
            drawnow limitrate;
        end

        function updateDroneTable (app)
        end
        
        function shiftCamera(app, dx, dy)
            % shiftCamera  Shift the camera left/right/up/down by dx, dy in the
            %              plane of the camera's view (i.e. sideways and up).
            %
            %  dx, dy are in data units (like meters). Positive dx => shift right,
            %  positive dy => shift up. We figure out "right" & "up" vectors by
            %  using camtarget, campos, camup.
    
            ax = app.UIAxes3D;
            
            % Current camera position & target
            pos = campos(ax);
            tgt = camtarget(ax);
    
            % Camera direction (from pos to target)
            dir = tgt - pos;
            dir = dir / norm(dir);
    
            % Camera's "up" vector
            upv = camup(ax);
            upv = upv / norm(upv);
    
            % The camera's "right" vector is cross(dir, up)
            rightv = cross(dir, upv);
            rightv = rightv / norm(rightv);
    
            % shiftVec is dx in 'right' + dy in 'up'
            shiftVec = dx * rightv + dy * upv;
    
            % Move both campos and camtarget
            newPos = pos + shiftVec;
            newTgt = tgt + shiftVec;
    
            campos(ax, newPos);
            camtarget(ax, newTgt);
        end

        % Helper for zone color
        function c = pickZoneColor(app, category)
            % PICKZONECOLOR  Return a color char or RGB triplet 
            % based on the category (e.g. 'A', 'P', 'M', etc.)
            
            switch category
                case 'A'
                    c = 'g';   % green
                case 'P'
                    c = 'r';   % red
                case 'M'
                    c = 'b';   % blue
                otherwise
                    c = 'y';   % yellow (default)
            end
        end
   
        function stopSimulationTimer(app)
            if ~isempty(app.simTimer) && isvalid(app.simTimer)
                stop(app.simTimer);
            end
            app.simulationRunning = false;
            app.Lamp.Color = [1 0 0];  % Red
            addConsoleMsg(app, "Simulation stopped.");
        end

        % Method to add a message to the Console
        function addConsoleMsg(app, message)
            % Get the current timestamp
            timestamp = datetime("now");
            
            % Format the message with timestamp
            formattedMsg = sprintf('[%s] %s', timestamp, message);
            
            % Append the message to the Console
            currentConsoleValue = app.Console.Value;
            if ischar(currentConsoleValue)
                % Convert single string to cell array if needed
                currentConsoleValue = {currentConsoleValue};
            end
            app.Console.Value = [currentConsoleValue; {formattedMsg}];
            
            % Ensure UI updates immediately
            %drawnow;
        end




        % Callback to update drone details based on selection
        function updateDroneDetails(app, dropdown, modelLabel, typeLabel, autonomyLabel, rechargeLabel)
            selectedName = dropdown.Value;
            idx = strcmp(app.UIFlotteTable.Data(:, 2), selectedName); % Updated to column 2 for "Nom"
            if any(idx)
                modelLabel.Text = app.UIFlotteTable.Data{idx, 8}; % "Modèle"
                typeLabel.Text = app.UIFlotteTable.Data{idx, 7}; % "Type"
                autonomyLabel.Text = num2str(app.UIFlotteTable.Data{idx, 9}); % "Autonomie Nominale (min)"
                rechargeLabel.Text = num2str(app.UIFlotteTable.Data{idx, 11}); % "Recharge Nominale (min)"
            end
        end
        
        function updateModelDetails(app, dropdown, typeLabel, autonomyLabel, rechargeLabel)
            selectedModel = dropdown.Value;
            idx = strcmp(app.DroneModels.Model, selectedModel);
            if any(idx)
                typeLabel.Text = app.DroneModels.Type{idx};
                autonomyLabel.Text = num2str(app.DroneModels.Autonomy(idx));
                rechargeLabel.Text = num2str(app.DroneModels.RechargeTime(idx));
            else
                typeLabel.Text = '';
                autonomyLabel.Text = '';
                rechargeLabel.Text = '';
            end
        end
        
        % Callback to delete a drone
        function deleteDroneCallback(app, fig, dropdown)
            selectedName = dropdown.Value;
            idx = strcmp(app.UIFlotteTable.Data(:, 2), selectedName); % Updated to column 2 for "Nom"
            if any(idx)
                app.UIFlotteTable.Data(idx, :) = [];
                close(fig);
            end
        end
        
        % Callback to modify a drone
        function modifyDroneCallback(app, fig, dropdown, nameField)
            selectedName = dropdown.Value;
            newName = nameField.Value;
            if isempty(newName) || any(strcmp(app.UIFlotteTable.Data(:, 2), newName)) % Updated to column 2 for "Nom"
                uialert(fig, 'Nom déjà existant ou invalide.', 'Erreur');
                return;
            end
            idx = strcmp(app.UIFlotteTable.Data(:, 2), selectedName); % Updated to column 2 for "Nom"
            if any(idx)
                app.UIFlotteTable.Data{idx, 2} = newName; % Updated to column 2 for "Nom"
                close(fig);
            end
        end
        
        % Callback to add a drone
        function addDroneCallback(app, fig, nameField, modelDropdown, typeLabel, autonomyLabel, rechargeLabel)
            % Retrieve data from input fields
            newName = nameField.Value;
            selectedModel = modelDropdown.Value;
            selectedType = typeLabel.Text;
            autonomy = str2double(autonomyLabel.Text);
            rechargeTime = str2double(rechargeLabel.Text);
        
            % % Validate inputs
            % if isempty(newName) || isempty(selectedModel) || any(strcmp(app.UIFlotteTable.Data(:, 2), newName))
            %     uialert(fig, 'Nom déjà existant ou invalide.', 'Erreur');
            %     return;
            % end
        
            % Add new drone to the table
            newRow = {size(app.UIFlotteTable.Data, 1) + 1, newName, [], [], [], [], selectedType, selectedModel, autonomy, 100, rechargeTime, 0};
            app.UIFlotteTable.Data = [app.UIFlotteTable.Data; newRow];
        
            % Close the window
            close(fig);
        end

        function updateTargetDetails(app, dropdown, coordLabel, statusLabel)
            selectedName = dropdown.Value;
            idx = strcmp(app.UICibleTable.Data(:, 2), selectedName);
            if any(idx)
                coordLabel.Text = app.UICibleTable.Data{idx, 3};
                statusLabel.Text = app.UICibleTable.Data{idx, 4};
            else
                coordLabel.Text = '';
                statusLabel.Text = '';
            end
        end

        function updateTargetFields(app, dropdown, newNameField, coordField, statusDropdown)
            selectedName = dropdown.Value;
            idx = strcmp(app.UICibleTable.Data(:, 2), selectedName);
            if any(idx)
                newNameField.Value = app.UICibleTable.Data{idx, 2};
                coordField.Value = app.UICibleTable.Data{idx, 3};
                statusDropdown.Value = app.UICibleTable.Data{idx, 4};
            else
                newNameField.Value = '';
                coordField.Value = '';
                statusDropdown.Value = '';
            end
        end

        function deleteTargetCallback(app, fig, dropdown)
            selectedName = dropdown.Value;
            idx = strcmp(app.UICibleTable.Data(:, 2), selectedName);
            if any(idx)
                app.UICibleTable.Data(idx, :) = [];
                close(fig);
            end
        end

        function modifyTargetCallback(app, fig, nomDropdown, newNameField, coordField, statusDropdown)
            selectedName = nomDropdown.Value;
            newName = newNameField.Value;
            newCoord = coordField.Value;
            newStatus = statusDropdown.Value;
        
            % Validate new name
            if ~isempty(newName) && any(strcmp(app.UICibleTable.Data(:, 2), newName)) && ~strcmp(selectedName, newName)
                uialert(fig, 'Nom déjà existant ou invalide.', 'Erreur');
                return;
            end
        
            % Update the table
            idx = strcmp(app.UICibleTable.Data(:, 2), selectedName);
            if any(idx)
                app.UICibleTable.Data{idx, 2} = newName;
                app.UICibleTable.Data{idx, 3} = newCoord;
                app.UICibleTable.Data{idx, 4} = newStatus;
                close(fig);
            end
        end

        function addTargetCallback(app, fig, nameField, coordField, statusDropdown)
            % Retrieve data from input fields
            newName = nameField.Value;
            coords = coordField.Value;
            status = statusDropdown.Value;
        
            % % Validate inputs
            % if isempty(newName) || isempty(coords) || any(strcmp(app.UICibleTable.Data(:, 2), newName))
            %     uialert(fig, 'Nom déjà existant ou invalide.', 'Erreur');
            %     return;
            % end
        
            % Add new target to the table
            newRow = {size(app.UICibleTable.Data, 1) + 1, newName, coords, status, '', 0};
            app.UICibleTable.Data = [app.UICibleTable.Data; newRow];
        
            % Close the window
            close(fig);
        end

        function updateZoneDetails(app, dropdown, typeLabel, coordLabel, sizeLabel, tiltLabel, statusLabel)
            selectedName = dropdown.Value;
            idx = strcmp(app.UIZoneTable.Data(:, 2), selectedName);
            if any(idx)
                typeLabel.Text = app.UIZoneTable.Data{idx, 3};
                coordLabel.Text = app.UIZoneTable.Data{idx, 4};
                sizeLabel.Text = num2str(app.UIZoneTable.Data{idx, 5});
                tiltLabel.Text = num2str(app.UIZoneTable.Data{idx, 6});
                statusLabel.Text = app.UIZoneTable.Data{idx, 7};
            else
                typeLabel.Text = '';
                coordLabel.Text = '';
                sizeLabel.Text = '';
                tiltLabel.Text = '';
                statusLabel.Text = '';
            end
        end

        function updateZoneFields(app, dropdown, newNameField, typeDropdown, coordField, sizeField, tiltField, statusDropdown)
            selectedName = dropdown.Value;
            idx = strcmp(app.UIZoneTable.Data(:, 2), selectedName);
            if any(idx)
                newNameField.Value = app.UIZoneTable.Data{idx, 2};
                typeDropdown.Value = app.UIZoneTable.Data{idx, 3};
                coordField.Value = app.UIZoneTable.Data{idx, 4};
                sizeField.Value = app.UIZoneTable.Data{idx, 5};
                tiltField.Value = app.UIZoneTable.Data{idx, 6};
                statusDropdown.Value = app.UIZoneTable.Data{idx, 7};
            else
                newNameField.Value = '';
                typeDropdown.Value = '';
                coordField.Value = '';
                sizeField.Value = '';
                tiltField.Value = '';
                statusDropdown.Value = '';
            end
        end

        function deleteZoneCallback(app, fig, nameDropdown)
            selectedName = nameDropdown.Value;
            idx = strcmp(app.UIZoneTable.Data(:, 2), selectedName);
            if any(idx)
                app.UIZoneTable.Data(idx, :) = [];
                close(fig);
            end
        end

        function modifyZoneCallback(app, fig, nameDropdown, newNameField, typeDropdown, coordField, sizeField, tiltField, statusDropdown)
            selectedName = nameDropdown.Value;
            newName = newNameField.Value;
            newType = typeDropdown.Value;
            newCoord = coordField.Value;
            newSize = sizeField.Value;
            newTilt = tiltField.Value;
            newStatus = statusDropdown.Value;
        
            % Validate new name
            if ~isempty(newName) && any(strcmp(app.UIZoneTable.Data(:, 2), newName)) && ~strcmp(selectedName, newName)
                uialert(fig, 'Nom déjà existant ou invalide.', 'Erreur');
                return;
            end
        
            % Update the table
            idx = strcmp(app.UIZoneTable.Data(:, 2), selectedName);
            if any(idx)
                app.UIZoneTable.Data{idx, 2} = newName;
                app.UIZoneTable.Data{idx, 3} = newType;
                app.UIZoneTable.Data{idx, 4} = newCoord;
                app.UIZoneTable.Data{idx, 5} = newSize;
                app.UIZoneTable.Data{idx, 6} = newTilt;
                app.UIZoneTable.Data{idx, 7} = newStatus;
                close(fig);
            end
        end

        function addZoneCallback(app, fig, nameField, typeDropdown, coordField, sizeField, tiltField, statusDropdown)
            % Retrieve data from input fields
            newName = nameField.Value;
            newType = typeDropdown.Value;
            coordinates = coordField.Value;
            zoneSize = sizeField.Value;
            tilt = tiltField.Value;
            status = statusDropdown.Value;
        
            % % Validate inputs
            % if isempty(newName) || any(strcmp(app.UIZoneTable.Data(:, 2), newName))
            %     uialert(fig, 'Nom déjà existant ou invalide.', 'Erreur');
            %     return;
            % end
        
            % Validate coordinates (must be in X,Y,Z format)
            try
                coordArray = str2num(coordinates); %#ok<ST2NM>
                if numel(coordArray) ~= 3
                    error('Invalid format');
                end
            catch
                uialert(fig, 'Coordonnées invalides. Utilisez le format X,Y,Z.', 'Erreur');
                return;
            end
        
            % Add new zone to the table
            newRow = {size(app.UIZoneTable.Data, 1) + 1, newName, newType, coordinates, zoneSize, tilt, status, 0};
            app.UIZoneTable.Data = [app.UIZoneTable.Data; newRow];
        
            % Close the window
            close(fig);
        end

        function updateObstacleDetails(app, dropdown, coordLabel, sizeLabel, tiltLabel)
            selectedNum = dropdown.Value;
            idx = strcmp(app.UIObstacleTable.Data(:, 1), selectedNum);
            if any(idx)
                coordLabel.Text = app.UIObstacleTable.Data{idx, 2};
                sizeLabel.Text = num2str(app.UIObstacleTable.Data{idx, 3});
                tiltLabel.Text = num2str(app.UIObstacleTable.Data{idx, 4});
            else
                coordLabel.Text = '';
                sizeLabel.Text = '';
                tiltLabel.Text = '';
            end
        end

        function updateObstacleFields(app, dropdown, coordField, sizeField, tiltField, statusDropdown)
            % Ensure dropdown has a valid value
            if isempty(dropdown.Value) || isempty(app.UIObstacleTable.Data)
                coordField.Value = '';
                sizeField.Value = '';
                tiltField.Value = '';
                statusDropdown.Value = '';
                return;
            end
        
            selectedNum = dropdown.Value;
            idx = strcmp(app.UIObstacleTable.Data(:, 1), selectedNum);
        
            % Update fields if the obstacle exists
            if any(idx)
                coordField.Value = app.UIObstacleTable.Data{idx, 2};
                sizeField.Value = app.UIObstacleTable.Data{idx, 3};
                tiltField.Value = app.UIObstacleTable.Data{idx, 4};
                statusDropdown.Value = app.UIObstacleTable.Data{idx, 5};
            else
                coordField.Value = '';
                sizeField.Value = '';
                tiltField.Value = '';
                statusDropdown.Value = '';
            end
        end

        function deleteObstacleCallback(app, fig, numDropdown)
            selectedNum = numDropdown.Value;
            idx = strcmp(app.UIObstacleTable.Data(:, 1), selectedNum);
            if any(idx)
                app.UIObstacleTable.Data(idx, :) = [];
                close(fig);
            end
        end

        function modifyObstacleCallback(app, fig, numDropdown, coordField, sizeField, tiltField, statusDropdown)
            selectedNum = numDropdown.Value;
            newCoord = coordField.Value;
            newSize = sizeField.Value;
            newTilt = tiltField.Value;
            newStatus = statusDropdown.Value;
        
            % Validate inputs
            if isempty(newCoord) || isempty(newSize) || isempty(newTilt) || isempty(newStatus)
                uialert(fig, 'Données invalides ou incomplètes.', 'Erreur');
                return;
            end
        
            % Update the table
            idx = strcmp(app.UIObstacleTable.Data(:, 1), selectedNum);
            if any(idx)
                app.UIObstacleTable.Data{idx, 2} = newCoord;
                app.UIObstacleTable.Data{idx, 3} = newSize;
                app.UIObstacleTable.Data{idx, 4} = newTilt;
                app.UIObstacleTable.Data{idx, 5} = newStatus;
                close(fig);
            else
                uialert(fig, 'Obstacle non trouvé.', 'Erreur');
            end
        end

        function addObstacleCallback(app, fig, coordField, sizeField, tiltField, statusDropdown)
            % Retrieve data from input fields
            coordinates = coordField.Value;
            size = sizeField.Value;
            tilt = tiltField.Value;
            status = statusDropdown.Value;
        
            % Validate inputs
            if isempty(coordinates) || isempty(size) || isempty(tilt) || isempty(status)
                uialert(fig, 'Données invalides ou incomplètes.', 'Erreur');
                return;
            end
        
            % Determine the next obstacle number
            if isempty(app.UIObstacleTable.Data)
                nextNum = 1; % First obstacle
            else
                existingNums = cell2mat(app.UIObstacleTable.Data(:, 1));
                nextNum = max(existingNums) + 1; % Increment the highest number
            end
        
            % Add new obstacle to the table
            newRow = {nextNum, coordinates, size, tilt, status};
            app.UIObstacleTable.Data = [app.UIObstacleTable.Data; newRow];
        
            % Close the window
            close(fig);
        end


        

        % % Callback function for the "Suppression" button
        % function onSuppressionButtonPushed(app, ~)
        %     % Create a new figure for deleting data
        %     supprFig = uifigure('Name', 'Supprimer', 'Position', [100, 100, 300, 150]);
        % 
        %     % Add dropdown menu to select existing drone
        %     uilabel(supprFig, 'Position', [20, 70, 100, 22], 'Text', 'Nom:');
        %     nomDropdown = uidropdown(supprFig, 'Position', [130, 70, 150, 22]);
        %     nomDropdown.Items = app.UITable.Data(:, 1); % Populate dropdown with existing names
        % 
        %     % OK button
        %     okButton = uibutton(supprFig, 'Text', 'OK', 'Position', [100, 30, 100, 30]);
        %     okButton.ButtonPushedFcn = @(~, ~) deleteDataCallback(app, supprFig, nomDropdown);
        % end

        % % Callback to add data
        % function addDataCallback(app, fig, nomField, coordField)
        %     % Retrieve data from fields
        %     nom = nomField.Value;
        %     coord = coordField.Value;
        % 
        %     % Add new row to the table
        %     newRow = {nom, coord};
        %     app.UIFlotteTable.Data = [app.UIFlotteTable.Data; newRow];
        % 
        %     % Close the window
        %     delete(fig);
        % end

        % % Callback to modify data
        % function modifyDataCallback(app, fig, nomDropdown, coordField)
        %     % Retrieve the selected drone name and the new coordinates
        %     nom       = nomDropdown.Value;
        %     newCoord  = coordField.Value;  % e.g. '100,200,50' if it's a string
        % 
        %     % Load the existing data from the correct table
        %     data = app.UIFlotteTable.Data;  % <--- fix here
        % 
        %     % Find matching name and update
        %     for i = 1:size(data, 1)
        %         if strcmp(data{i, 1}, nom)
        %             data{i, 2} = newCoord;  % Assuming column #2 is for coordinates
        %             break;
        %         end
        %     end
        % 
        %     % Write the table back
        %     app.UIFlotteTable.Data = data;
        % 
        %     % Close the window
        %     delete(fig);
        % end

        % % Callback to delete data
        % function deleteDataCallback(app, fig, nomDropdown)
        %     % Retrieve data from dropdown
        %     nom = nomDropdown.Value;
        % 
        %     % Find the row to delete
        %     data = app.UIFlotteTable.Data;
        %     rowToDelete = false(size(data, 1), 1);
        %     for i = 1:size(data, 1)
        %         if strcmp(data{i, 1}, nom)
        %             rowToDelete(i) = true;
        %             break;
        %         end
        %     end
        % 
        %     % Remove the row and update the table
        %     data(rowToDelete, :) = [];
        %     app.UIFlotteTable.Data = data;
        % 
        %     % Close the window
        %     delete(fig);
        % end
    end
    

    % Callbacks that handle component events
    methods (Access = private)

        % Button pushed function: StartButton
        function StartButtonPushed(app, event)
            if ~app.simulationRunning
                % Initialize or re-initialize the simulation
                setupSimulation(app);
                startSimulationTimer(app);
            elseif app.simulationPaused
                % If it's paused, just unpause it
                app.simulationPaused = false;
                app.Lamp.Color = [0 1 0];
                addConsoleMsg(app, "Simulation resumed.");
            end
        end

        % Button pushed function: ResetButton
        function ResetButtonPushed(app, event)
            if app.simulationRunning
                stopSimulationTimer(app);
            end
            % Re-initialize everything
            setupSimulation(app);
            app.Lamp.Color = [1 0 0];
            cla(app.UIAxes3D,"reset");
            addConsoleMsg(app, "Simulation reset.");
        end

        % Button pushed function: PauseButton
        function PauseButtonPushed(app, event)
            if app.simulationRunning && ~app.simulationPaused
                app.simulationPaused = true;
                app.Lamp.Color = [1 1 0]; % Yellow
                addConsoleMsg(app, "Simulation paused.");
            elseif app.simulationRunning && app.simulationPaused
                % If paused, pressing Pause again could resume or do nothing
                app.simulationPaused = false;
                app.Lamp.Color = [0 1 0];
                addConsoleMsg(app, "Simulation resumed.");
            end
        end

        % Button pushed function: SuppressionDroneButton
        function SuppressionDroneButtonPushed(app, event)
            % Create a new figure for deleting a drone
            supprFig = uifigure('Name', 'Suppression Drone', 'Position', [100, 100, 400, 250]);
        
            % Dropdown menu to select existing drone
            uilabel(supprFig, 'Position', [20, 190, 100, 22], 'Text', 'Nom:');
            nomDropdown = uidropdown(supprFig, 'Position', [130, 190, 200, 22], 'Items', app.UIFlotteTable.Data(:, 2));
        
            % Display drone details
            uilabel(supprFig, 'Position', [20, 140, 100, 22], 'Text', 'Modèle:');
            modelLabel = uilabel(supprFig, 'Position', [130, 140, 200, 22]);
        
            uilabel(supprFig, 'Position', [20, 100, 150, 22], 'Text', 'Type:');
            typeLabel = uilabel(supprFig, 'Position', [130, 100, 200, 22]);
        
            uilabel(supprFig, 'Position', [20, 60, 200, 22], 'Text', 'Autonomie (minutes):');
            autonomyLabel = uilabel(supprFig, 'Position', [230, 60, 100, 22]);
        
            uilabel(supprFig, 'Position', [20, 30, 200, 22], 'Text', 'Recharge (minutes):');
            rechargeLabel = uilabel(supprFig, 'Position', [230, 30, 100, 22]);
        
            % Update drone details when dropdown value changes
            nomDropdown.ValueChangedFcn = @(~, ~) updateDroneDetails(app, nomDropdown, modelLabel, typeLabel, autonomyLabel, rechargeLabel);
            updateDroneDetails(app, nomDropdown, modelLabel, typeLabel, autonomyLabel, rechargeLabel);
        
            % OK button to confirm deletion
            okButton = uibutton(supprFig, 'Text', 'OK', 'Position', [150, 10, 100, 30]);
            okButton.ButtonPushedFcn = @(~, ~) deleteDroneCallback(app, supprFig, nomDropdown);
        end

        % Button pushed function: ModificationDroneButton
        function ModificationDroneButtonPushed(app, event)
            % Create a new figure for modifying a drone
            modifFig = uifigure('Name', 'Modification Drone', 'Position', [100, 100, 400, 300]);
        
            % Dropdown menu to select existing drone
            uilabel(modifFig, 'Position', [20, 230, 100, 22], 'Text', 'Nom:');
            nomDropdown = uidropdown(modifFig, 'Position', [130, 230, 200, 22], 'Items', app.UIFlotteTable.Data(:, 2));
        
            % Display drone details
            uilabel(modifFig, 'Position', [20, 190, 100, 22], 'Text', 'Modèle:');
            modelLabel = uilabel(modifFig, 'Position', [130, 190, 200, 22]);
        
            uilabel(modifFig, 'Position', [20, 150, 150, 22], 'Text', 'Type:');
            typeLabel = uilabel(modifFig, 'Position', [130, 150, 200, 22]);
        
            uilabel(modifFig, 'Position', [20, 110, 200, 22], 'Text', 'Autonomie (minutes):');
            autonomyLabel = uilabel(modifFig, 'Position', [230, 110, 100, 22]);
        
            uilabel(modifFig, 'Position', [20, 70, 200, 22], 'Text', 'Recharge (minutes):');
            rechargeLabel = uilabel(modifFig, 'Position', [230, 70, 100, 22]);
        
            % Text field for new name
            uilabel(modifFig, 'Position', [20, 30, 150, 22], 'Text', 'Nouveau Nom:');
            newNameField = uieditfield(modifFig, 'Position', [130, 30, 200, 22]);
        
            % Update drone details when dropdown value changes
            nomDropdown.ValueChangedFcn = @(~, ~) updateDroneDetails(app, nomDropdown, modelLabel, typeLabel, autonomyLabel, rechargeLabel);
            updateDroneDetails(app, nomDropdown, modelLabel, typeLabel, autonomyLabel, rechargeLabel);
        
            % OK button to confirm modification
            okButton = uibutton(modifFig, 'Text', 'OK', 'Position', [150, 10, 100, 30]);
            okButton.ButtonPushedFcn = @(~, ~) modifyDroneCallback(app, modifFig, nomDropdown, newNameField);
        end

        % Button pushed function: AjoutDroneButton
        function AjoutDroneButtonPushed(app, event)
            % Create a new figure for adding a drone
            models = app.DroneModels;
            numDrones = size(app.UIFlotteTable.Data, 1);
            ajoutFig = uifigure('Name', ['Ajout Drone n°', num2str(numDrones + 1)], 'Position', [100, 100, 400, 300]);
        
            % Input field for drone name
            uilabel(ajoutFig, 'Position', [20, 230, 150, 22], 'Text', 'Nom:');
            nameField = uieditfield(ajoutFig, 'Position', [130, 230, 200, 22]);
        
            % Dropdown for drone models
            uilabel(ajoutFig, 'Position', [20, 190, 150, 22], 'Text', 'Modèle:');
            modelDropdown = uidropdown(ajoutFig, 'Position', [130, 190, 200, 22], 'Items', models.Model);
        
            % Labels for additional details
            uilabel(ajoutFig, 'Position', [20, 150, 100, 22], 'Text', 'Type:');
            typeLabel = uilabel(ajoutFig, 'Position', [130, 150, 200, 22]);
        
            uilabel(ajoutFig, 'Position', [20, 110, 200, 22], 'Text', 'Autonomie (minutes):');
            autonomyLabel = uilabel(ajoutFig, 'Position', [230, 110, 100, 22]);
        
            uilabel(ajoutFig, 'Position', [20, 70, 200, 22], 'Text', 'Recharge (minutes):');
            rechargeLabel = uilabel(ajoutFig, 'Position', [230, 70, 100, 22]);
        
            % Update details when the model changes
            modelDropdown.ValueChangedFcn = @(src, ~) updateModelDetails(app, src, typeLabel, autonomyLabel, rechargeLabel);
        
            % Initialize details with the first model
            if ~isempty(models.Model)
                updateModelDetails(app, modelDropdown, typeLabel, autonomyLabel, rechargeLabel);
            end
        
            % OK button to confirm addition
            okButton = uibutton(ajoutFig, 'Text', 'OK', 'Position', [150, 10, 100, 30]);
            okButton.ButtonPushedFcn = @(~, ~) addDroneCallback(app, ajoutFig, nameField, modelDropdown, typeLabel, autonomyLabel, rechargeLabel);
        end

        % Value changed function: dtSlider
        function dtSliderValueChanged(app, event)
            % Get the new value of dt from the slider
            newDt = app.dtSlider.Value;
            
            % Update the timer's period if the simulation is running
            if ~isempty(app.simTimer) && isvalid(app.simTimer)
                if strcmp(app.simTimer.Running, 'on')
                    stop(app.simTimer); % Stop the timer before changing
                    app.simTimer.Period = newDt; % Update the timer's period
                    start(app.simTimer); % Restart the timer
                else
                    app.simTimer.Period = newDt; % Update the timer's period
                end
            end
            
            % Log the change to the console
            addConsoleMsg(app, sprintf('Time step (dt) updated to %.2f seconds.', newDt));            
        end

        % Window scroll wheel function: UIFigure
        function onScroll(app, event)
            % onScroll  Zoom in/out with mouse wheel in a UIAxes
            %   Positive evt.VerticalScrollCount => wheel scrolled "down" => zoom out
            %   Negative => wheel scrolled "up" => zoom in
            %
            % We'll adjust the camera view angle (camva) in perspective mode.
    
            ax = app.UIAxes3D;
            scrollSteps = evt.VerticalScrollCount;  % e.g. +1 or -1 per notch
            zoomFactor  = 1.05;  % ~5% zoom steps
    
            currentVA = camva(ax);  % current view angle in degrees
            if scrollSteps > 0
                % Scroll down => Zoom OUT => Increase the view angle
                newVA = currentVA * zoomFactor;
            else
                % Scroll up => Zoom IN => Decrease the view angle
                newVA = currentVA / zoomFactor;
            end
    
            % Limit how wide or narrow the angle can get
            newVA = max(1, min(179, newVA));
            camva(ax, newVA);
        end

        % Window key press function: UIFigure
        function onKeyPress(app, event)
            % onKeyPress  Pan (translate) the camera with arrow keys in UIAxes.
            %             We shift both camera position and camera target.
            %
            % Arrow keys => left/right/up/down in the "screen plane" of the camera view.
            
            ax = app.UIAxes3D;
            moveAmount = 10;  % how many "data units" to shift per press
    
            switch evt.Key
                case 'leftarrow'
                    % Move left
                    shiftCamera(app, -moveAmount, 0);
                case 'rightarrow'
                    % Move right
                    shiftCamera(app, +moveAmount, 0);
                case 'uparrow'
                    % Move up
                    shiftCamera(app, 0, +moveAmount);
                case 'downarrow'
                    % Move down
                    shiftCamera(app, 0, -moveAmount);
            end
        end

        % Button pushed function: AjoutCibleButton
        function AjoutCibleButtonPushed(app, event)
            % Create a new figure for adding a target
            numTargets = size(app.UICibleTable.Data, 1);
            ajoutFig = uifigure('Name', ['Ajout Cible n°', num2str(numTargets + 1)], 'Position', [100, 100, 400, 300]);
        
            % Input field for target name
            uilabel(ajoutFig, 'Position', [20, 230, 150, 22], 'Text', 'Nom:');
            nameField = uieditfield(ajoutFig, 'Position', [130, 230, 200, 22]);
        
            % Input field for target coordinates
            uilabel(ajoutFig, 'Position', [20, 190, 150, 22], 'Text', 'Coordonnées (X,Y,Z):');
            coordField = uieditfield(ajoutFig, 'Position', [130, 190, 200, 22]);
        
            % Dropdown for target status
            uilabel(ajoutFig, 'Position', [20, 150, 150, 22], 'Text', 'Statut:');
            statusDropdown = uidropdown(ajoutFig, 'Position', [130, 150, 200, 22], 'Items', {'actif', 'inactif'});
        
            % OK button to confirm addition
            okButton = uibutton(ajoutFig, 'Text', 'OK', 'Position', [150, 10, 100, 30]);
            okButton.ButtonPushedFcn = @(~, ~) addTargetCallback(app, ajoutFig, nameField, coordField, statusDropdown);
        end

        % Button pushed function: ModificationCibleButton
        function ModificationCibleButtonPushed(app, event)
            % Create a new figure for modifying a target
            modifFig = uifigure('Name', 'Modification Cible', 'Position', [100, 100, 400, 400]);
        
            % Dropdown to select existing target
            uilabel(modifFig, 'Position', [20, 330, 150, 22], 'Text', 'Nom:');
            nomDropdown = uidropdown(modifFig, 'Position', [130, 330, 200, 22], 'Items', app.UICibleTable.Data(:, 2));
        
            % Input fields for modifying data
            uilabel(modifFig, 'Position', [20, 290, 150, 22], 'Text', 'Nouveau Nom:');
            newNameField = uieditfield(modifFig, 'Position', [130, 290, 200, 22]);
        
            uilabel(modifFig, 'Position', [20, 250, 150, 22], 'Text', 'Coordonnées (X,Y,Z):');
            coordField = uieditfield(modifFig, 'Position', [130, 250, 200, 22]);
        
            uilabel(modifFig, 'Position', [20, 210, 150, 22], 'Text', 'Statut:');
            statusDropdown = uidropdown(modifFig, 'Position', [130, 210, 200, 22], 'Items', {'actif', 'inactif'});
        
            % OK button to confirm modification
            okButton = uibutton(modifFig, 'Text', 'OK', 'Position', [150, 50, 100, 30]);
            okButton.ButtonPushedFcn = @(~, ~) modifyTargetCallback(app, modifFig, nomDropdown, newNameField, coordField, statusDropdown);
        
            % Update displayed details when a target is selected
            nomDropdown.ValueChangedFcn = @(~, ~) updateTargetFields(app, nomDropdown, newNameField, coordField, statusDropdown);
            updateTargetFields(app, nomDropdown, newNameField, coordField, statusDropdown);
        end

        % Button pushed function: SuppressionCibleButton
        function SuppressionCibleButtonPushed(app, event)
            % Create a new figure for deleting a target
            supprFig = uifigure('Name', 'Suppression Cible', 'Position', [100, 100, 400, 250]);
        
            % Dropdown menu to select existing target
            uilabel(supprFig, 'Position', [20, 190, 150, 22], 'Text', 'Nom:');
            nomDropdown = uidropdown(supprFig, 'Position', [130, 190, 200, 22], 'Items', app.UICibleTable.Data(:, 2));
        
            % Display target details
            uilabel(supprFig, 'Position', [20, 150, 150, 22], 'Text', 'Coordonnées:');
            coordLabel = uilabel(supprFig, 'Position', [130, 150, 200, 22]);
        
            uilabel(supprFig, 'Position', [20, 110, 150, 22], 'Text', 'Statut:');
            statusLabel = uilabel(supprFig, 'Position', [130, 110, 200, 22]);
        
            % Update target details when dropdown value changes
            nomDropdown.ValueChangedFcn = @(~, ~) updateTargetDetails(app, nomDropdown, coordLabel, statusLabel);
            updateTargetDetails(app, nomDropdown, coordLabel, statusLabel);
        
            % OK button to confirm deletion
            okButton = uibutton(supprFig, 'Text', 'OK', 'Position', [150, 10, 100, 30]);
            okButton.ButtonPushedFcn = @(~, ~) deleteTargetCallback(app, supprFig, nomDropdown);
        end

        % Button pushed function: AjoutZoneButton
        function AjoutZoneButtonPushed(app, event)
            % Create a new figure for adding a zone
            numZones = size(app.UIZoneTable.Data, 1);
            ajoutFig = uifigure('Name', ['Ajout Zone n°', num2str(numZones + 1)], 'Position', [100, 100, 400, 400]);
        
            % Input field for zone name
            uilabel(ajoutFig, 'Position', [20, 330, 150, 22], 'Text', 'Nom:');
            nameField = uieditfield(ajoutFig, 'Position', [130, 330, 200, 22]);
        
            % Dropdown for zone type
            uilabel(ajoutFig, 'Position', [20, 290, 150, 22], 'Text', 'Type:');
            typeDropdown = uidropdown(ajoutFig, 'Position', [130, 290, 200, 22], 'Items', {'allied', 'prohibited', 'other'});
        
            % Input field for coordinates
            uilabel(ajoutFig, 'Position', [20, 250, 150, 22], 'Text', 'Coordonnées (X,Y,Z):');
            coordField = uieditfield(ajoutFig, 'Position', [130, 250, 200, 22]);
        
            % Input field for size
            uilabel(ajoutFig, 'Position', [20, 210, 150, 22], 'Text', 'Taille (m):');
            sizeField = uieditfield(ajoutFig, 'numeric', 'Position', [130, 210, 200, 22]);
        
            % Input field for tilt angle
            uilabel(ajoutFig, 'Position', [20, 170, 150, 22], 'Text', 'Orientation (°):');
            tiltField = uieditfield(ajoutFig, 'numeric', 'Position', [130, 170, 200, 22]);
        
            % Dropdown for status
            uilabel(ajoutFig, 'Position', [20, 130, 150, 22], 'Text', 'Statut:');
            statusDropdown = uidropdown(ajoutFig, 'Position', [130, 130, 200, 22], 'Items', {'actif', 'inactif'});
        
            % OK button to confirm addition
            okButton = uibutton(ajoutFig, 'Text', 'OK', 'Position', [150, 50, 100, 30]);
            okButton.ButtonPushedFcn = @(~, ~) addZoneCallback(app, ajoutFig, nameField, typeDropdown, coordField, sizeField, tiltField, statusDropdown);
        end

        % Button pushed function: ModificationZoneButton
        function ModificationZoneButtonPushed(app, event)
            % Create a new figure for modifying a zone
            modifFig = uifigure('Name', 'Modification Zone', 'Position', [100, 100, 400, 400]);
        
            % Dropdown menu to select existing zone
            uilabel(modifFig, 'Position', [20, 330, 150, 22], 'Text', 'Nom:');
            nameDropdown = uidropdown(modifFig, 'Position', [130, 330, 200, 22], 'Items', app.UIZoneTable.Data(:, 2));
        
            % Input fields for modifying data
            uilabel(modifFig, 'Position', [20, 290, 150, 22], 'Text', 'Nouveau Nom:');
            newNameField = uieditfield(modifFig, 'Position', [130, 290, 200, 22]);
        
            uilabel(modifFig, 'Position', [20, 250, 150, 22], 'Text', 'Type:');
            typeDropdown = uidropdown(modifFig, 'Position', [130, 250, 200, 22], 'Items', {'allied', 'prohibited', 'other'});
        
            uilabel(modifFig, 'Position', [20, 210, 150, 22], 'Text', 'Coordonnées (X,Y,Z):');
            coordField = uieditfield(modifFig, 'Position', [130, 210, 200, 22]);
        
            uilabel(modifFig, 'Position', [20, 170, 150, 22], 'Text', 'Taille (m):');
            sizeField = uieditfield(modifFig, 'numeric', 'Position', [130, 170, 200, 22]);
        
            uilabel(modifFig, 'Position', [20, 130, 150, 22], 'Text', 'Orientation (°):');
            tiltField = uieditfield(modifFig, 'numeric', 'Position', [130, 130, 200, 22]);
        
            uilabel(modifFig, 'Position', [20, 90, 150, 22], 'Text', 'Statut:');
            statusDropdown = uidropdown(modifFig, 'Position', [130, 90, 200, 22], 'Items', {'actif', 'inactif'});
        
            % OK button to confirm modification
            okButton = uibutton(modifFig, 'Text', 'OK', 'Position', [150, 10, 100, 30]);
            okButton.ButtonPushedFcn = @(~, ~) modifyZoneCallback(app, modifFig, nameDropdown, newNameField, typeDropdown, coordField, sizeField, tiltField, statusDropdown);
        
            % Update displayed details when a zone is selected
            nameDropdown.ValueChangedFcn = @(~, ~) updateZoneFields(app, nameDropdown, newNameField, typeDropdown, coordField, sizeField, tiltField, statusDropdown);
            updateZoneFields(app, nameDropdown, newNameField, typeDropdown, coordField, sizeField, tiltField, statusDropdown);
        end

        % Button pushed function: SuppressionZoneButton
        function SuppressionZoneButtonPushed(app, event)
            % Create a new figure for deleting a zone
            supprFig = uifigure('Name', 'Suppression Zone', 'Position', [100, 100, 400, 300]);
        
            % Dropdown menu to select existing zone
            uilabel(supprFig, 'Position', [20, 230, 150, 22], 'Text', 'Nom:');
            nameDropdown = uidropdown(supprFig, 'Position', [130, 230, 200, 22], 'Items', app.UIZoneTable.Data(:, 2));
        
            % Display zone details
            uilabel(supprFig, 'Position', [20, 190, 150, 22], 'Text', 'Type:');
            typeLabel = uilabel(supprFig, 'Position', [130, 190, 200, 22]);
        
            uilabel(supprFig, 'Position', [20, 150, 150, 22], 'Text', 'Coordonnées:');
            coordLabel = uilabel(supprFig, 'Position', [130, 150, 200, 22]);
        
            uilabel(supprFig, 'Position', [20, 110, 150, 22], 'Text', 'Taille (m):');
            sizeLabel = uilabel(supprFig, 'Position', [130, 110, 200, 22]);
        
            uilabel(supprFig, 'Position', [20, 70, 150, 22], 'Text', 'Orientation (°):');
            tiltLabel = uilabel(supprFig, 'Position', [130, 70, 200, 22]);
        
            % OK button to confirm deletion
            okButton = uibutton(supprFig, 'Text', 'OK', 'Position', [150, 10, 100, 30]);
            okButton.ButtonPushedFcn = @(~, ~) deleteZoneCallback(app, supprFig, nameDropdown);
        
            % Update displayed details when a zone is selected
            nameDropdown.ValueChangedFcn = @(~, ~) updateZoneDetails(app, nameDropdown, typeLabel, coordLabel, sizeLabel, tiltLabel, []);
            updateZoneDetails(app, nameDropdown, typeLabel, coordLabel, sizeLabel, tiltLabel, []);
        end

        % Button pushed function: AjoutObstacleButton
        function AjoutObstacleButtonPushed(app, event)
            % Create a new figure for adding an obstacle
            numObstacles = size(app.UIObstacleTable.Data, 1);
            ajoutFig = uifigure('Name', ['Ajout Obstacle n°', num2str(numObstacles + 1)], 'Position', [100, 100, 400, 300]);
        
            % Input field for obstacle coordinates
            uilabel(ajoutFig, 'Position', [20, 230, 150, 22], 'Text', 'Coordonnées (X,Y,Z):');
            coordField = uieditfield(ajoutFig, 'Position', [180, 230, 200, 22]);
        
            % Input field for obstacle size
            uilabel(ajoutFig, 'Position', [20, 190, 150, 22], 'Text', 'Taille (m):');
            sizeField = uieditfield(ajoutFig, 'numeric', 'Position', [180, 190, 200, 22]);
        
            % Input field for obstacle orientation
            uilabel(ajoutFig, 'Position', [20, 150, 150, 22], 'Text', 'Orientation (°):');
            tiltField = uieditfield(ajoutFig, 'numeric', 'Position', [180, 150, 200, 22]);
        
            % Dropdown for obstacle status
            uilabel(ajoutFig, 'Position', [20, 110, 150, 22], 'Text', 'Statut:');
            statusDropdown = uidropdown(ajoutFig, 'Position', [180, 110, 200, 22], 'Items', {'actif', 'inactif'});
        
            % OK button to confirm addition
            okButton = uibutton(ajoutFig, 'Text', 'OK', 'Position', [150, 50, 100, 30]);
            okButton.ButtonPushedFcn = @(~, ~) addObstacleCallback(app, ajoutFig, coordField, sizeField, tiltField, statusDropdown);
        end

        % Button pushed function: ModificationObstacleButton
        function ModificationObstacleButtonPushed(app, event)
            % Create a new figure for modifying an obstacle
            modifFig = uifigure('Name', 'Modification Obstacle', 'Position', [100, 100, 400, 400]);
        
            % Check if there are obstacles in the table
            if isempty(app.UIObstacleTable.Data)
                uialert(modifFig, 'Aucun obstacle à modifier.', 'Erreur');
                return;
            end
        
            % Dropdown menu to select existing obstacle
            uilabel(modifFig, 'Position', [20, 330, 150, 22], 'Text', 'N°:');
            obstacleNumbers = string(app.UIObstacleTable.Data(:, 1));
            numDropdown = uidropdown(modifFig, ...
                'Position', [130, 330, 200, 22], ...
                'Items', obstacleNumbers);
        
            % Set the dropdown value to the first item if not empty
            if ~isempty(obstacleNumbers)
                numDropdown.Value = obstacleNumbers(1);
            end
        
            % Input fields for modifying data
            uilabel(modifFig, 'Position', [20, 290, 150, 22], 'Text', 'Coordonnées (X,Y,Z):');
            coordField = uieditfield(modifFig, 'Position', [130, 290, 200, 22]);
        
            uilabel(modifFig, 'Position', [20, 250, 150, 22], 'Text', 'Taille (m):');
            sizeField = uieditfield(modifFig, 'numeric', 'Position', [130, 250, 200, 22]);
        
            uilabel(modifFig, 'Position', [20, 210, 150, 22], 'Text', 'Orientation (°):');
            tiltField = uieditfield(modifFig, 'numeric', 'Position', [130, 210, 200, 22]);
        
            uilabel(modifFig, 'Position', [20, 170, 150, 22], 'Text', 'Statut:');
            statusDropdown = uidropdown(modifFig, 'Position', [130, 170, 200, 22], 'Items', {'actif', 'inactif'});
        
            % OK button to confirm modification
            okButton = uibutton(modifFig, 'Text', 'OK', 'Position', [150, 50, 100, 30]);
            okButton.ButtonPushedFcn = @(~, ~) modifyObstacleCallback(app, modifFig, numDropdown, coordField, sizeField, tiltField, statusDropdown);
        
            % Update displayed details when an obstacle is selected
            numDropdown.ValueChangedFcn = @(~, ~) updateObstacleFields(app, numDropdown, coordField, sizeField, tiltField, statusDropdown);
            updateObstacleFields(app, numDropdown, coordField, sizeField, tiltField, statusDropdown);
        end

        % Button pushed function: SuppressionObstacleButton
        function SuppressionObstacleButtonPushed(app, event)
            % Create a new figure for deleting an obstacle
            supprFig = uifigure('Name', 'Suppression Obstacle', 'Position', [100, 100, 400, 250]);
        
            % Dropdown menu to select existing obstacle
            uilabel(supprFig, 'Position', [20, 150, 150, 22], 'Text', 'N°:');
            numDropdown = uidropdown(supprFig, 'Position', [180, 150, 200, 22], 'Items', string(app.UIObstacleTable.Data(:, 1)));
        
            % Display obstacle details
            uilabel(supprFig, 'Position', [20, 110, 150, 22], 'Text', 'Coordonnées:');
            coordLabel = uilabel(supprFig, 'Position', [180, 110, 200, 22]);
        
            uilabel(supprFig, 'Position', [20, 70, 150, 22], 'Text', 'Taille (m):');
            sizeLabel = uilabel(supprFig, 'Position', [180, 70, 200, 22]);
        
            uilabel(supprFig, 'Position', [20, 30, 150, 22], 'Text', 'Orientation (°):');
            tiltLabel = uilabel(supprFig, 'Position', [180, 30, 200, 22]);
        
            % OK button to confirm deletion
            okButton = uibutton(supprFig, 'Text', 'OK', 'Position', [150, 10, 100, 30]);
            okButton.ButtonPushedFcn = @(~, ~) deleteObstacleCallback(app, supprFig, numDropdown);
        
            % Update displayed details when an obstacle is selected
            numDropdown.ValueChangedFcn = @(~, ~) updateObstacleDetails(app, numDropdown, coordLabel, sizeLabel, tiltLabel);
            updateObstacleDetails(app, numDropdown, coordLabel, sizeLabel, tiltLabel);
        end
    end

    % Component initialization
    methods (Access = private)

        % Create UIFigure and components
        function createComponents(app)

            % Create UIFigure and hide until all components are created
            app.UIFigure = uifigure('Visible', 'off');
            app.UIFigure.Position = [100 100 1920 1080];
            app.UIFigure.Name = 'MATLAB App';
            app.UIFigure.WindowScrollWheelFcn = createCallbackFcn(app, @onScroll, true);
            app.UIFigure.WindowKeyPressFcn = createCallbackFcn(app, @onKeyPress, true);

            % Create DviewMenu
            app.DviewMenu = uimenu(app.UIFigure);
            app.DviewMenu.Text = '3Dview';

            % Create InfoMenu
            app.InfoMenu = uimenu(app.UIFigure);
            app.InfoMenu.Text = 'Info';

            % Create Menu
            app.Menu = uimenu(app.InfoMenu);
            app.Menu.Text = 'Menu';

            % Create Menu2
            app.Menu2 = uimenu(app.InfoMenu);
            app.Menu2.Text = 'Menu2';

            % Create UIAxes2D
            app.UIAxes2D = uiaxes(app.UIFigure);
            title(app.UIAxes2D, 'Affichage 2D')
            xlabel(app.UIAxes2D, 'X')
            ylabel(app.UIAxes2D, 'Y')
            app.UIAxes2D.ZTick = [];
            app.UIAxes2D.XGrid = 'on';
            app.UIAxes2D.YGrid = 'on';
            app.UIAxes2D.Position = [9 1 921 546];

            % Create UIAxes3D
            app.UIAxes3D = uiaxes(app.UIFigure);
            title(app.UIAxes3D, 'Affichage 3D')
            xlabel(app.UIAxes3D, 'X')
            ylabel(app.UIAxes3D, 'Y')
            zlabel(app.UIAxes3D, 'Z')
            app.UIAxes3D.Projection = 'perspective';
            app.UIAxes3D.XGrid = 'on';
            app.UIAxes3D.YGrid = 'on';
            app.UIAxes3D.ZGrid = 'on';
            app.UIAxes3D.Position = [1 548 982 533];

            % Create TabGroup
            app.TabGroup = uitabgroup(app.UIFigure);
            app.TabGroup.Position = [982 1 940 869];

            % Create ObservationTab
            app.ObservationTab = uitab(app.TabGroup);
            app.ObservationTab.Title = 'Observation';

            % Create TranslationAVButton
            app.TranslationAVButton = uibutton(app.ObservationTab, 'push');
            app.TranslationAVButton.Position = [91 798 100 23];
            app.TranslationAVButton.Text = 'Translation AV';

            % Create TranslationDRButton
            app.TranslationDRButton = uibutton(app.ObservationTab, 'push');
            app.TranslationDRButton.Position = [154 766 100 23];
            app.TranslationDRButton.Text = 'Translation DR';

            % Create TranslationARButton
            app.TranslationARButton = uibutton(app.ObservationTab, 'push');
            app.TranslationARButton.Position = [91 734 100 23];
            app.TranslationARButton.Text = 'Translation AR';

            % Create TranslationGAButton
            app.TranslationGAButton = uibutton(app.ObservationTab, 'push');
            app.TranslationGAButton.Position = [28 766 100 23];
            app.TranslationGAButton.Text = 'Translation GA';

            % Create RotationDRButton
            app.RotationDRButton = uibutton(app.ObservationTab, 'push');
            app.RotationDRButton.Position = [154 688 100 23];
            app.RotationDRButton.Text = 'Rotation DR';

            % Create RotationGAButton
            app.RotationGAButton = uibutton(app.ObservationTab, 'push');
            app.RotationGAButton.Position = [28 688 100 23];
            app.RotationGAButton.Text = 'Rotation GA';

            % Create PanUPButton
            app.PanUPButton = uibutton(app.ObservationTab, 'push');
            app.PanUPButton.Position = [284 703 100 23];
            app.PanUPButton.Text = 'Pan UP';

            % Create PanDNButton
            app.PanDNButton = uibutton(app.ObservationTab, 'push');
            app.PanDNButton.Position = [284 675 100 23];
            app.PanDNButton.Text = 'Pan DN';

            % Create FlotteTab
            app.FlotteTab = uitab(app.TabGroup);
            app.FlotteTab.Title = 'Flotte';

            % Create UIFlotteTable
            app.UIFlotteTable = uitable(app.FlotteTab);
            app.UIFlotteTable.ColumnName = {'N°'; 'Nom'; 'Coordonnées'; 'Cible'; 'Statut'; 'Phase'; 'Type'; 'Modèle'; 'Autonomie Nominale (min)'; 'Autonomie Restante (%)'; 'Recharge Nominale (min)'; 'Recharge Restante (min)'};
            app.UIFlotteTable.RowName = {};
            app.UIFlotteTable.Position = [1 54 939 790];

            % Create AjoutDroneButton
            app.AjoutDroneButton = uibutton(app.FlotteTab, 'push');
            app.AjoutDroneButton.ButtonPushedFcn = createCallbackFcn(app, @AjoutDroneButtonPushed, true);
            app.AjoutDroneButton.Position = [28 16 100 23];
            app.AjoutDroneButton.Text = 'Ajout';

            % Create ModificationDroneButton
            app.ModificationDroneButton = uibutton(app.FlotteTab, 'push');
            app.ModificationDroneButton.ButtonPushedFcn = createCallbackFcn(app, @ModificationDroneButtonPushed, true);
            app.ModificationDroneButton.Position = [154 16 100 23];
            app.ModificationDroneButton.Text = 'Modification';

            % Create SuppressionDroneButton
            app.SuppressionDroneButton = uibutton(app.FlotteTab, 'push');
            app.SuppressionDroneButton.ButtonPushedFcn = createCallbackFcn(app, @SuppressionDroneButtonPushed, true);
            app.SuppressionDroneButton.Position = [283 16 100 23];
            app.SuppressionDroneButton.Text = 'Suppression';

            % Create CiblesTab
            app.CiblesTab = uitab(app.TabGroup);
            app.CiblesTab.Title = 'Cibles';

            % Create UICibleTable
            app.UICibleTable = uitable(app.CiblesTab);
            app.UICibleTable.ColumnName = {'N°'; 'Nom'; 'Coordonnées'; 'Statut'; 'Flotte Allouée'; 'Score d''Observabilité'};
            app.UICibleTable.RowName = {};
            app.UICibleTable.Position = [1 54 939 790];

            % Create AjoutCibleButton
            app.AjoutCibleButton = uibutton(app.CiblesTab, 'push');
            app.AjoutCibleButton.ButtonPushedFcn = createCallbackFcn(app, @AjoutCibleButtonPushed, true);
            app.AjoutCibleButton.Position = [28 16 100 23];
            app.AjoutCibleButton.Text = 'Ajout';

            % Create ModificationCibleButton
            app.ModificationCibleButton = uibutton(app.CiblesTab, 'push');
            app.ModificationCibleButton.ButtonPushedFcn = createCallbackFcn(app, @ModificationCibleButtonPushed, true);
            app.ModificationCibleButton.Position = [154 16 100 23];
            app.ModificationCibleButton.Text = 'Modification';

            % Create SuppressionCibleButton
            app.SuppressionCibleButton = uibutton(app.CiblesTab, 'push');
            app.SuppressionCibleButton.ButtonPushedFcn = createCallbackFcn(app, @SuppressionCibleButtonPushed, true);
            app.SuppressionCibleButton.Position = [283 16 100 23];
            app.SuppressionCibleButton.Text = 'Suppression';

            % Create ZonesTab
            app.ZonesTab = uitab(app.TabGroup);
            app.ZonesTab.Title = 'Zones';

            % Create UIZoneTable
            app.UIZoneTable = uitable(app.ZonesTab);
            app.UIZoneTable.ColumnName = {'N°'; 'Nom'; 'Type'; 'Coordonnées'; 'Taille'; 'Orientation'; 'Statut'; 'Temps de Violation'};
            app.UIZoneTable.RowName = {};
            app.UIZoneTable.Position = [1 54 939 790];

            % Create AjoutZoneButton
            app.AjoutZoneButton = uibutton(app.ZonesTab, 'push');
            app.AjoutZoneButton.ButtonPushedFcn = createCallbackFcn(app, @AjoutZoneButtonPushed, true);
            app.AjoutZoneButton.Position = [28 16 100 23];
            app.AjoutZoneButton.Text = 'Ajout';

            % Create ModificationZoneButton
            app.ModificationZoneButton = uibutton(app.ZonesTab, 'push');
            app.ModificationZoneButton.ButtonPushedFcn = createCallbackFcn(app, @ModificationZoneButtonPushed, true);
            app.ModificationZoneButton.Position = [154 16 100 23];
            app.ModificationZoneButton.Text = 'Modification';

            % Create SuppressionZoneButton
            app.SuppressionZoneButton = uibutton(app.ZonesTab, 'push');
            app.SuppressionZoneButton.ButtonPushedFcn = createCallbackFcn(app, @SuppressionZoneButtonPushed, true);
            app.SuppressionZoneButton.Position = [283 16 100 23];
            app.SuppressionZoneButton.Text = 'Suppression';

            % Create ObstaclesTab
            app.ObstaclesTab = uitab(app.TabGroup);
            app.ObstaclesTab.Title = 'Obstacles';

            % Create UIObstacleTable
            app.UIObstacleTable = uitable(app.ObstaclesTab);
            app.UIObstacleTable.ColumnName = {'N°'; 'Coordonnées'; 'Taille'; 'Orientation'; 'Statut'};
            app.UIObstacleTable.RowName = {};
            app.UIObstacleTable.Position = [1 54 939 790];

            % Create AjoutObstacleButton
            app.AjoutObstacleButton = uibutton(app.ObstaclesTab, 'push');
            app.AjoutObstacleButton.ButtonPushedFcn = createCallbackFcn(app, @AjoutObstacleButtonPushed, true);
            app.AjoutObstacleButton.Position = [28 16 100 23];
            app.AjoutObstacleButton.Text = 'Ajout';

            % Create ModificationObstacleButton
            app.ModificationObstacleButton = uibutton(app.ObstaclesTab, 'push');
            app.ModificationObstacleButton.ButtonPushedFcn = createCallbackFcn(app, @ModificationObstacleButtonPushed, true);
            app.ModificationObstacleButton.Position = [154 16 100 23];
            app.ModificationObstacleButton.Text = 'Modification';

            % Create SuppressionObstacleButton
            app.SuppressionObstacleButton = uibutton(app.ObstaclesTab, 'push');
            app.SuppressionObstacleButton.ButtonPushedFcn = createCallbackFcn(app, @SuppressionObstacleButtonPushed, true);
            app.SuppressionObstacleButton.Position = [283 16 100 23];
            app.SuppressionObstacleButton.Text = 'Suppression';

            % Create ComportementTab
            app.ComportementTab = uitab(app.TabGroup);
            app.ComportementTab.Title = 'Comportement';

            % Create Panel
            app.Panel = uipanel(app.UIFigure);
            app.Panel.Position = [982 869 939 212];

            % Create Console
            app.Console = uitextarea(app.Panel);
            app.Console.Position = [442 8 487 174];

            % Create tempscouleEditField
            app.tempscouleEditField = uieditfield(app.Panel, 'text');
            app.tempscouleEditField.Position = [108 83 63 22];

            % Create dtSliderLabel
            app.dtSliderLabel = uilabel(app.Panel);
            app.dtSliderLabel.HorizontalAlignment = 'center';
            app.dtSliderLabel.Position = [200 79 25 22];
            app.dtSliderLabel.Text = 'dt';

            % Create dtSlider
            app.dtSlider = uislider(app.Panel);
            app.dtSlider.Limits = [0.03 10];
            app.dtSlider.ValueChangedFcn = createCallbackFcn(app, @dtSliderValueChanged, true);
            app.dtSlider.Position = [237 88 133 3];
            app.dtSlider.Value = 0.03;

            % Create DronesdploysEditFieldLabel
            app.DronesdploysEditFieldLabel = uilabel(app.Panel);
            app.DronesdploysEditFieldLabel.HorizontalAlignment = 'right';
            app.DronesdploysEditFieldLabel.Position = [201 179 95 22];
            app.DronesdploysEditFieldLabel.Text = 'Drones déployés';

            % Create DronesdploysEditField
            app.DronesdploysEditField = uieditfield(app.Panel, 'numeric');
            app.DronesdploysEditField.HorizontalAlignment = 'center';
            app.DronesdploysEditField.Position = [331 179 50 22];

            % Create DronesnondploysEditFieldLabel
            app.DronesnondploysEditFieldLabel = uilabel(app.Panel);
            app.DronesnondploysEditFieldLabel.HorizontalAlignment = 'right';
            app.DronesnondploysEditFieldLabel.Position = [201 149 118 22];
            app.DronesnondploysEditFieldLabel.Text = 'Drones non déployés';

            % Create DronesnondploysEditField
            app.DronesnondploysEditField = uieditfield(app.Panel, 'numeric');
            app.DronesnondploysEditField.HorizontalAlignment = 'center';
            app.DronesnondploysEditField.Position = [331 149 50 22];

            % Create TempscoulLabel
            app.TempscoulLabel = uilabel(app.Panel);
            app.TempscoulLabel.HorizontalAlignment = 'right';
            app.TempscoulLabel.Position = [20 83 79 22];
            app.TempscoulLabel.Text = 'Temps écoulé';

            % Create EventLoggingLabel
            app.EventLoggingLabel = uilabel(app.Panel);
            app.EventLoggingLabel.HorizontalAlignment = 'center';
            app.EventLoggingLabel.Position = [441 181 490 30];
            app.EventLoggingLabel.Text = 'Event Logging';

            % Create DronesperdusEditFieldLabel
            app.DronesperdusEditFieldLabel = uilabel(app.Panel);
            app.DronesperdusEditFieldLabel.HorizontalAlignment = 'right';
            app.DronesperdusEditFieldLabel.Position = [201 119 84 22];
            app.DronesperdusEditFieldLabel.Text = 'Drones perdus';

            % Create DronesperdusEditField
            app.DronesperdusEditField = uieditfield(app.Panel, 'numeric');
            app.DronesperdusEditField.HorizontalAlignment = 'center';
            app.DronesperdusEditField.Position = [331 119 50 22];

            % Create ResetButton
            app.ResetButton = uibutton(app.Panel, 'push');
            app.ResetButton.ButtonPushedFcn = createCallbackFcn(app, @ResetButtonPushed, true);
            app.ResetButton.Position = [71 119 100 23];
            app.ResetButton.Text = 'Reset';

            % Create PauseButton
            app.PauseButton = uibutton(app.Panel, 'push');
            app.PauseButton.ButtonPushedFcn = createCallbackFcn(app, @PauseButtonPushed, true);
            app.PauseButton.Position = [71 148 100 23];
            app.PauseButton.Text = 'Pause';

            % Create StartButton
            app.StartButton = uibutton(app.Panel, 'push');
            app.StartButton.ButtonPushedFcn = createCallbackFcn(app, @StartButtonPushed, true);
            app.StartButton.Position = [71 179 100 23];
            app.StartButton.Text = 'Start';

            % Create Lamp
            app.Lamp = uilamp(app.Panel);
            app.Lamp.Position = [28 161 28 28];
            app.Lamp.Color = [1 0 0];

            % Show the figure after all components are created
            app.UIFigure.Visible = 'on';
        end
    end

    % App creation and deletion
    methods (Access = public)

        % Construct app
        function app = IHM_FINAL

            % Create UIFigure and components
            createComponents(app)

            % Register the app with App Designer
            registerApp(app, app.UIFigure)

            if nargout == 0
                clear app
            end
        end

        % Code that executes before app deletion
        function delete(app)

            % Delete UIFigure when app is deleted
            delete(app.UIFigure)
        end
    end
end