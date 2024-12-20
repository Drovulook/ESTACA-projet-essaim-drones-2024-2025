function RTPlot2(env, swarm, dt, temps, r, swarm_weights, weights, Target, traceSize)

    framerate = 1/20;

    %% Échelle, Angle d'observation et Légende
    f = figure('Position', [100, 100, 1200, 800]);
    rotate3d on
    ax = gca;

    %XLim = [-150 150];
    %YLim = [-150 150];
    XLim = [min(env.GroundCoordinates.x) max(env.GroundCoordinates.x)]; % dimension défini lors de la création de : Environment
    YLim = [min(env.GroundCoordinates.y) max(env.GroundCoordinates.y)];

    ZLim = [0 150];
    speedVectorSize = 2;

    set(ax, 'XLim', XLim, 'YLim', YLim, 'ZLim', ZLim);

    view(-75, 15);
    grid on;
    hold on;

    axis equal;

    xlabel('X (Nord - Sud)');
    ylabel('Y (Ouest - Est)');
    zlabel('Z (Altitude)');
    title('Rendu de Simulation en Temps Réel');

    %% Affichage des zones et de la surface de sol
    fill3(env.GroundCoordinates.x, env.GroundCoordinates.y, env.GroundCoordinates.z, [0.5 0.5 0.5], 'FaceAlpha', 0.3);

    for i = 1:length(env.ZonesList)
        zone = env.ZonesList{i};

        switch zone.Category
            case 'A'
                zoneColor = 'g';
            case 'P'
                zoneColor = 'r';
            case 'M'
                zoneColor = 'b';
            otherwise
                zoneColor = 'y';
        end

        create_shape(zone.Type, zoneColor, zone.CenterPosition, zone.Dimensions, zone.TiltAngle, ax);
    end

    targ = gobjects(1, size(Target,1));
    for k = 1:size(Target,1)
        targ(k) = scatter3(Target(k,1), Target(k,2), Target(k,3), 50, 'filled', 'MarkerFaceColor', 'r');
    end

    wp = gobjects(1, 1);
    
    for i = 1:length(swarm.FixedWing)
        wp(i) = scatter3(swarm.FixedWing{i}.Waypoints(1,1), swarm.FixedWing{i}.Waypoints(1,2), swarm.FixedWing{i}.Waypoints(1,3), 50, 'filled', 'MarkerFaceColor', 'g');
    end

    n_drone = size(swarm.Drones, 2);
    drone_pos = zeros(n_drone, 3);

    for i = 1:n_drone
        drone_pos(i, :) = swarm.Drones{i}.posState;
    end

    head = gobjects(1, n_drone);
    quiver_handle = gobjects(1, n_drone);
    trace = gobjects(1, n_drone); % Initialize traces

    for i = 1:n_drone
        head(i) = scatter3(drone_pos(i, 1), drone_pos(i, 2), drone_pos(i, 3), 50, 'filled', 'MarkerFaceColor', 'b');
        quiver_handle(i) = quiver3(drone_pos(i, 1), drone_pos(i, 2), drone_pos(i, 3), 0, 0, 0, 'r', 'LineWidth', 1.5, 'MaxHeadSize', 0.5);
        trace(i) = plot3(nan, nan, nan, 'Color', [0 0 1], 'LineStyle', '--'); % Initialize traces with NaNs
    end

    sliderX = uicontrol('Style', 'slider', ...
               'Min', -150, 'Max', 150, 'Value', 0, ...
               'Units', 'normalized', ...
               'Position', [0.2 0.02 0.2 0.03], ...
               'Callback', @(src, event) update(src));
    sliderY = uicontrol('Style', 'slider', ...
               'Min', -150, 'Max', 150, 'Value', 0, ...
               'Units', 'normalized', ...
               'Position', [0.4 0.02 0.2 0.03], ...
               'Callback', @(src, event) update(src));
    sliderZ = uicontrol('Style', 'slider', ...
               'Min', -10, 'Max', 150, 'Value', 0, ...
               'Units', 'normalized', ...
               'Position', [0.6 0.02 0.2 0.03], ...
               'Callback', @(src, event) update(src));

    function toto = update(slider)
        toto = get(slider, 'Value');
    end

    %% Boucle de simulation
    k = 0;
    tic;
    while true
        swarm.update_speeds(dt, r, swarm_weights, weights);

        drone_pos = zeros(n_drone, 3);
        drone_speed = zeros(n_drone, 3);

        for i = 1:n_drone
            drone_pos(i, :) = swarm.Drones{i}.posState;
            drone_speed(i, :) = swarm.Drones{i}.speedState;
        end

        for i = 1:n_drone
            set(head(i), 'XData', drone_pos(i, 1), 'YData', drone_pos(i, 2), 'ZData', drone_pos(i, 3));
            set(quiver_handle(i), 'XData', drone_pos(i, 1), 'YData', drone_pos(i, 2), 'ZData', drone_pos(i, 3), ...
                'UData', drone_speed(i, 1) * speedVectorSize, 'VData', drone_speed(i, 2) * speedVectorSize, 'WData', drone_speed(i, 3) * speedVectorSize);

            % Update trace with the last positions
            last_positions = swarm.Drones{i}.posLog; 
            num_positions = size(last_positions, 1);
            trace_indices = max(1, num_positions - (traceSize - 1)):num_positions; 
            recent_positions = last_positions(trace_indices, :);

            set(trace(i), 'XData', recent_positions(:, 1), 'YData', recent_positions(:, 2), 'ZData', recent_positions(:, 3));
        end

        % Re-implemented feature from v1: updating the displayed waypoint
         
        for i = 1:length(swarm.FixedWing)
            set(wp(i), 'XData', swarm.FixedWing{i}.Waypoints(swarm.FixedWing{i}.CurrentWaypoint,1), ...
                        'YData', swarm.FixedWing{i}.Waypoints(swarm.FixedWing{i}.CurrentWaypoint,2), ...
                        'ZData', swarm.FixedWing{i}.Waypoints(swarm.FixedWing{i}.CurrentWaypoint,3));
        end

        Target(1, 1) = update(sliderX);
        Target(1, 2) = update(sliderY);
        Target(1, 3) = update(sliderZ);

        swarm.target_history_matrix = [swarm.target_history_matrix; Target(1, :)];

        set(targ(1), 'XData', Target(1, 1), 'YData', Target(1, 2), 'ZData', Target(1, 3));
        swarm.update_target(Target);

        set(ax, 'XLim', XLim, 'YLim', YLim, 'ZLim', ZLim);
        
        while true
            if toc > framerate
                drawnow limitrate;
                break
            end
        end
        tic;

        k = k + 1;
        if k == temps
            break;
        end
    end

    pause(0.1);

    figure('Position', [100, 100, 1200, 800]);
    rotate3d on
    ax = gca;
    set(ax, 'XLim', XLim, 'YLim', YLim, 'ZLim', ZLim);
    grid on;
    axis equal

    hold on;

    xlabel('X');
    ylabel('Y');
    zlabel('Altitude (Z)');
    title('Rendu de Simulation en Temps Différé');

    fill3(env.GroundCoordinates.x, env.GroundCoordinates.y, env.GroundCoordinates.z, [0.5 0.5 0.5], 'FaceAlpha', 0.3);

    swarm.drones_pos_history_matrix = cat(3, zeros(n_drone, 3, 15), swarm.drones_pos_history_matrix);

    target = scatter3(swarm.target_history_matrix(1, 1), swarm.target_history_matrix(1, 2), swarm.target_history_matrix(1, 3), 50, 'filled', 'MarkerFaceColor', 'r');

end
