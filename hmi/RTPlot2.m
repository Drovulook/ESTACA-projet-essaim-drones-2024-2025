function RTPlot2(env, swarm, dt, temps, r, swarm_weights, weights, pondeTarg, sat, Target)

    %% Échelle, Angle d'observation et Légende
    % Configurer la figure avec une taille de fenêtre fixe plus grande
    figure('Position', [100, 100, 1200, 800]); % Largeur 1200px, Hauteur 800px
    ax = gca;
    set(ax, 'XLim', [-20 20], 'YLim', [-20 20], 'ZLim', [-10 20]);
    view(45, 30); % Définir un angle de vue en perspective (azimut=45, élévation=30)
    grid on;
    hold on;
    
    % Mettre les axes à échelle égale pour une représentation réaliste et ajouter des étiquettes
    axis equal
    xlabel('X'); % Étiquette de l'axe x
    ylabel('Y'); % Étiquette de l'axe y
    zlabel('Altitude (Z)'); % Étiquette de l'axe z
    title('Rendu de Simulation en Temps Réel'); % Titre de la figure


    %% Affichage des zones et de la surface de sol

    % Ajouter une surface de sol grise à z = 0
    fill3(env.GroundCoordinates.x, env.GroundCoordinates.y, env.GroundCoordinates.z, [0.5 0.5 0.5], 'FaceAlpha', 0.3); % Couleur grise avec transparence
    
    %Truc bourrin à changer
    drone1 = swarm.Drones{1}.posState;
    drone2 = swarm.Drones{2}.posState;
    drone3 = swarm.Drones{3}.posState;
    drone4 = swarm.Drones{4}.posState;

    % Initialiser les objets de tracé pour les éléments animés
    target1 = scatter3(Target(1,1), Target(1,2), Target(1,2), 30)
    target2 = scatter3(Target(2,1), Target(2,2), Target(2,2), 30)

    head1 = scatter3(drone1(1), drone1(2), drone1(3), 50, 'filled', 'MarkerFaceColor', 'b');
    head2 = scatter3(drone2(1), drone2(2), drone2(3), 50, 'filled', 'MarkerFaceColor', 'g');
    head3 = scatter3(drone3(1), drone3(2), drone3(3), 'filled', 'MarkerFaceColor', 'b');
    head4 = scatter3(drone4(1), drone4(2), drone4(3), 50, 'filled', 'MarkerFaceColor', 'g');
    quiver_handle1 = quiver3(drone1(1), drone1(2), drone1(3), 0, 0, 0, 'r', 'LineWidth', 1.5, 'MaxHeadSize', 0.5);
    quiver_handle2 = quiver3(drone2(1), drone2(2), drone2(3), 0, 0, 0, 'r', 'LineWidth', 1.5, 'MaxHeadSize', 0.5);
    quiver_handle3 = quiver3(drone3(1), drone3(2), drone3(3), 0, 0, 0, 'r', 'LineWidth', 1.5, 'MaxHeadSize', 0.5);
    quiver_handle4 = quiver3(drone4(1), drone4(2), drone4(3), 0, 0, 0, 'r', 'LineWidth', 1.5, 'MaxHeadSize', 0.5);


    %% Boucle de simulation
    k =0;
    while true
        %Fonction qui calcule la speed T+1 et update la pos T0
        swarm.update_speed(dt,r,swarm_weights, weights, pondeTarg, sat)
        
        %Hard codé à modifier pour que ce soit variable
        drone1 = swarm.Drones{1}.posState;
        drone2 = swarm.Drones{2}.posState;
        drone3 = swarm.Drones{3}.posState;
        drone4 = swarm.Drones{4}.posState;

        speed1 = swarm.Drones{1}.speedState;
        speed2 = swarm.Drones{2}.speedState;
        speed3 = swarm.Drones{3}.speedState;
        speed4 = swarm.Drones{4}.speedState;

        %Hardcodé pareil, c'est bourrin
        % Mettre à jour les données des objets scatter3 pour les positions
        set(head1, 'XData', drone1(1), 'YData', drone1(2), 'ZData', drone1(3));
        set(head2, 'XData', drone2(1), 'YData', drone2(2), 'ZData', drone2(3));
        set(head3, 'XData', drone3(1), 'YData', drone3(2), 'ZData', drone3(3));
        set(head4, 'XData', drone4(1), 'YData', drone4(2), 'ZData', drone4(3));
    
        %Toujours aussi bourrin
        % Mettre à jour les objets quiver3 pour afficher les vecteurs de vitesse
        set(quiver_handle1, 'XData', drone1(1), 'YData', drone1(2), 'ZData', drone1(3), ...
            'UData', speed1(1), 'VData', speed1(2), 'WData', speed1(3));
        set(quiver_handle2, 'XData', drone2(1), 'YData', drone2(2), 'ZData', drone2(3), ...
            'UData', speed2(1), 'VData', speed2(2), 'WData', speed2(3));
        set(quiver_handle3, 'XData', drone3(1), 'YData', drone3(2), 'ZData', drone3(3), ...
            'UData', speed3(1), 'VData', speed3(2), 'WData', speed3(3));
        set(quiver_handle4, 'XData', drone4(1), 'YData', drone4(2), 'ZData', drone4(3), ...
            'UData', speed4(1), 'VData', speed4(2), 'WData', speed4(3));
        
       
        % Rafraîchir le tracé pour montrer les nouvelles positions et vecteurs
        drawnow limitrate
        
        % Pause optionnelle pour contrôler la vitesse de visualisation
        pause(0.0001);  % Ajuster ou supprimer selon les besoins

        k = k + 1;
        if k == temps
            break;
        end
    end
    
end

