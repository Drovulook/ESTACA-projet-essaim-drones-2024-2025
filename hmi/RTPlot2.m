function RTPlot2(env, swarm, dt, temps, r, swarm_weights, weights, pondeTarg, sat, Target)

    %% Échelle, Angle d'observation et Légende
    % Configurer la figure avec une taille de fenêtre fixe plus grande
    f = figure('Position', [100, 100, 1200, 800]); % Largeur 1200px, Hauteur 800px
    rotate3d on
    ax = gca;
    
    XLim = [-150 150];
    YLim = [-150 150];
    ZLim = [-10 150];

    set(ax, 'XLim', XLim, 'YLim', YLim, 'ZLim', ZLim);

    view(45, 30); % Définir un angle de vue en perspective (azimut=45, élévation=30)
    grid on;
    hold on;
    
    % Mettre les axes à échelle égale pour une représentation réaliste et ajouter des étiquettes
    axis equal;

    xlabel('X'); % Étiquette de l'axe x
    ylabel('Y'); % Étiquette de l'axe y
    zlabel('Altitude (Z)'); % Étiquette de l'axe z
    title('Rendu de Simulation en Temps Réel'); % Titre de la figure


    %% Affichage des zones et de la surface de sol

    % Ajouter une surface de sol grise à z = 0
    fill3(env.GroundCoordinates.x, env.GroundCoordinates.y, env.GroundCoordinates.z, [0.5 0.5 0.5], 'FaceAlpha', 0.3); % Couleur grise avec transparence
    
    % Afficher chaque zone de l'environnement avec un code couleur selon sa catégorie
    for i = 1:length(env.ZonesList)
        zone = env.ZonesList{i};

        switch zone.Category
            case 'A'
                zoneColor = 'g'; % Zones autorisées en vert
            case 'P'
                zoneColor = 'r'; % Zones prohibées en rouge
            case 'M'
                zoneColor = 'b'; % Zones de manœuvre en bleu
            otherwise
                zoneColor = 'y'; % Autres zones en jaune
        end

        % Créer la forme de la zone avec le type, la couleur, la position et les dimensions spécifiées
        create_shape(zone.Type, zoneColor, zone.CenterPosition, zone.Dimensions, zone.TiltAngle, ax);
    end

    targ = gobjects(1, size(Target,1));
    for k = 1:size(Target,1)
        targ(k)=scatter3(Target(k,1), Target(k,2), Target(k,3), 50, 'filled', 'MarkerFaceColor', 'r');
    end

    n_drone=size(swarm.Drones,2);
    drone_pos=zeros(n_drone,3);

    for i=1:n_drone
        drone_pos(i,:)=swarm.Drones{i}.posState;

    end

    % Initialiser les objets de tracé pour les éléments animés    
    head=gobjects(1,n_drone);
    quiver_handle=gobjects(1,n_drone);

    for i=1:n_drone
        head(i)=scatter3(drone_pos(i,1), drone_pos(i,2), drone_pos(i,3), 50, 'filled', 'MarkerFaceColor', 'b');
        quiver_handle(i)=quiver3(drone_pos(i,1), drone_pos(i,2), drone_pos(i,3), 0, 0, 0, 'r', 'LineWidth', 1.5, 'MaxHeadSize', 0.5);
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
        % Récupérer la valeur actuelle du slider
        toto = get(slider, 'Value');
    end

    %% Boucle de simulation
    k =0;
    while true
        %Fonction qui calcule la speed T+1 et update la pos T0
        swarm.update_speed(dt,r,swarm_weights, weights, pondeTarg, sat)
                
        drone_pos=zeros(n_drone,3);
        drone_speed=zeros(n_drone,3);
        for i=1:n_drone
            drone_pos(i,:)=swarm.Drones{i}.posState;
            drone_speed(i,:)=swarm.Drones{i}.speedState;
        end

        for i=1:n_drone
             set(head(i), 'XData', drone_pos(i,1), 'YData', drone_pos(i,2), 'ZData', drone_pos(i,3));
             set(quiver_handle(i), 'XData', drone_pos(i,1), 'YData', drone_pos(i,2), 'ZData', drone_pos(i,3), ...
                 'UData', drone_speed(i,1)*20, 'VData', drone_speed(i,2)*20, 'WData', drone_speed(i,3)*20);
        end
       

        % Rafraîchir le tracé pour montrer les nouvelles positions et vecteurs

        %Test Slider

        Target(1, 1) = update(sliderX);
        Target(1, 2) = update(sliderY);
        Target(1, 3) = update(sliderZ);
        
        swarm.target_history_matrix = [swarm.target_history_matrix; Target(1,:)];

        set(targ(1), 'XData', Target(1,1), 'YData', Target(1,2), 'ZData', Target(1,3));
        swarm.update_target(Target);

        set(ax, 'XLim', XLim, 'YLim', YLim, 'ZLim', ZLim);
        drawnow limitrate
   
        % Pause optionnelle pour contrôler la vitesse de visualisation
        %pause(0.001);  % Ajuster ou supprimer selon les besoins

        k = k + 1;
        if k == temps
            break;
        end
    end
    
    pause(0.1)

    % Traçage temps différé de toutes les positions
    figure('Position', [100, 100, 1200, 800]);
    rotate3d on
    ax = gca;
    set(ax, 'XLim', XLim, 'YLim', YLim, 'ZLim', ZLim);
    grid on;
    axis equal

    hold on;

    xlabel('X'); % Étiquette de l'axe x
    ylabel('Y'); % Étiquette de l'axe y
    zlabel('Altitude (Z)'); % Étiquette de l'axe z
    title('Rendu de Simulation en Temps Différé'); % Titre de la figure

    fill3(env.GroundCoordinates.x, env.GroundCoordinates.y, env.GroundCoordinates.z, [0.5 0.5 0.5], 'FaceAlpha', 0.3); % Couleur grise avec transparence
    
    swarm.drones_pos_history_matrix = cat(3,zeros(n_drone,3,15), swarm.drones_pos_history_matrix);

    head=gobjects(1,n_drone);
    for i=1:n_drone
        head(i)=scatter3(swarm.drones_pos_history_matrix(i,1,16), swarm.drones_pos_history_matrix(i,2,16), swarm.drones_pos_history_matrix(i,3,16), 50, 'filled', 'MarkerFaceColor', 'b');
    end

    target = scatter3(swarm.target_history_matrix(1,1), swarm.target_history_matrix(1,2), swarm.target_history_matrix(1,3), 50, 'filled', 'MarkerFaceColor', 'r');

    for t = 16:temps
        for i=1:n_drone
            set(head(i), 'XData', swarm.drones_pos_history_matrix(i,1,t), 'YData', swarm.drones_pos_history_matrix(i,2,t), 'ZData', swarm.drones_pos_history_matrix(i,3,t));
            set(target, 'XData', swarm.target_history_matrix(t-15,1), 'YData', swarm.target_history_matrix(t-15,2), 'ZData', swarm.target_history_matrix(t-15,3));

        end
        drawnow;
        
    end

end


