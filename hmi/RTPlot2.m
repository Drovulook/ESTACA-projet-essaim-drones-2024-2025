function RTPlot2(env, swarm, dt, temps, r, swarm_weights, weights, pondeTarg, sat, Target)

    %% Échelle, Angle d'observation et Légende
    % Configurer la figure avec une taille de fenêtre fixe plus grande
    f = figure('Position', [100, 100, 1200, 800]); % Largeur 1200px, Hauteur 800px
    
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
        create_shape(zone.Type, zoneColor, zone.CenterPosition, zone.Dimensions, zone.TiltAngle);
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

    slider = uicontrol('Style', 'slider', ...
               'Min', 0, 'Max', 20, 'Value', 1, ...
               'Units', 'normalized', ...
               'Position', [0.2 0.02 0.6 0.03], ...
               'Callback', @(src, event) update(src));

    function toto = update(src, event)
        % Récupérer la valeur actuelle du slider
        toto = src.Value;
    end

    %% Boucle de simulation
    k =0;
    while true
        %Fonction qui calcule la speed T+1 et update la pos T0
        swarm.update_speeds(dt,r,swarm_weights, weights, pondeTarg, sat)
                
        drone_pos=zeros(n_drone,3);
        drone_speed=zeros(n_drone,3);
        for i=1:n_drone
            drone_pos(i,:)=swarm.Drones{i}.posState;
            drone_speed(i,:)=swarm.Drones{i}.speedState;
        end

        for i=1:n_drone
             set(head(i), 'XData', drone_pos(i,1), 'YData', drone_pos(i,2), 'ZData', drone_pos(i,3));
             set(quiver_handle(i), 'XData', drone_pos(i,1), 'YData', drone_pos(i,2), 'ZData', drone_pos(i,3), ...
                 'UData', drone_speed(i,1), 'VData', drone_speed(i,2), 'WData', drone_speed(i,3));
        end
       
        % Rafraîchir le tracé pour montrer les nouvelles positions et vecteurs
        to = update(slider);
        Target(1, 1) = to
        set(targ(1), 'XData', Target(1,1), 'YData', Target(1,2), 'ZData', Target(1,3));
        swarm.update_target(Target);

        set(ax, 'XLim', XLim, 'YLim', YLim, 'ZLim', ZLim);
        drawnow limitrate
   
        % Pause optionnelle pour contrôler la vitesse de visualisation
        pause(0.001);  % Ajuster ou supprimer selon les besoins

        k = k + 1;
        if k == temps
            break;
        end
    end
    
end


