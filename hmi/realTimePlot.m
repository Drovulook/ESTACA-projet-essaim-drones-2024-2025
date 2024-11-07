function realTimePlot(env, swarm) % swarm pas encore utilisé, normal
    %% Initialisation des données
    t = 0:0.05:100;
    z = t;
    z1 = 100 - t;
    y = 100 * sin(2 * z);
    y1 = 100 * sin(2 * z1);
    x = 100 * cos(2 * z);
    x1 = 100 * cos(2 * z1);
    
    %% Échelle, Angle d'observation et Légende
    % Configurer la figure avec une taille de fenêtre fixe plus grande
    figure('Position', [100, 100, 1200, 800]); % Largeur 1200px, Hauteur 800px
    ax = gca;
    set(ax, 'XLim', [-150 150], 'YLim', [-150 150], 'ZLim', [0 100]);
    view(45, 30); % Définir un angle de vue en perspective (azimut=45, élévation=30)
    grid on;
    hold on;
    
    % Mettre les axes à échelle égale pour une représentation réaliste et ajouter des étiquettes
    axis equal
    xlabel('Est (m)'); % Étiquette de l'axe x
    ylabel('Nord (m)'); % Étiquette de l'axe y
    zlabel('Altitude (m)'); % Étiquette de l'axe z
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

    % Initialiser les objets de tracé pour les éléments animés
    head = scatter3(x(1), y(1), z(1), 50, 'filled', 'MarkerFaceColor', 'b');
    head1 = scatter3(x1(1), y1(1), z1(1), 50, 'filled', 'MarkerFaceColor', 'g');
    quiver_handle = quiver3(x(1), y(1), z(1), 0, 0, 0, 'r', 'LineWidth', 1.5, 'MaxHeadSize', 0.5);
    quiver_handle1 = quiver3(x1(1), y1(1), z1(1), 0, 0, 0, 'r', 'LineWidth', 1.5, 'MaxHeadSize', 0.5);
    
    %% Boucle de simulation
    for i = 2:length(t)  % Commencer à 2 pour calculer la vitesse depuis le point précédent
        % Calculer les composantes de vitesse en tant que différences par rapport au point précédent
        vx = 3 * (x(i) - x(i - 1));
        vy = 3 * (y(i) - y(i - 1));
        vz = 3 * (z(i) - z(i - 1));
    
        vx1 = 3 * (x1(i) - x1(i - 1));
        vy1 = 3 * (y1(i) - y1(i - 1));
        vz1 = 3 * (z1(i) - z1(i - 1));
    
        % Mettre à jour les données des objets scatter3 pour les positions
        set(head, 'XData', x(i), 'YData', y(i), 'ZData', z(i));
        set(head1, 'XData', x1(i), 'YData', y1(i), 'ZData', z1(i));
    
        % Mettre à jour les objets quiver3 pour afficher les vecteurs de vitesse
        set(quiver_handle, 'XData', x(i), 'YData', y(i), 'ZData', z(i), ...
            'UData', vx, 'VData', vy, 'WData', vz);
        set(quiver_handle1, 'XData', x1(i), 'YData', y1(i), 'ZData', z1(i), ...
            'UData', vx1, 'VData', vy1, 'WData', vz1);
    
        % Rafraîchir le tracé pour montrer les nouvelles positions et vecteurs
        drawnow limitrate
    
        % Pause optionnelle pour contrôler la vitesse de visualisation
        pause(0.0001);  % Ajuster ou supprimer selon les besoins
    end
end
