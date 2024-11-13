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

    % Initialiser les objets de tracé pour les éléments animés
    n_drone=size(swarm.Drones);
    n_drone=n_drone(2);

    drone=zeros(n_drone,3);
    for i=1:n_drone
        drone(i,:)=swarm.Drones{i}.posState;

    end

    % Initialiser les objets de tracé pour les éléments animés    
    head=gobjects(1,n_drone);
    quiver_handle=gobjects(1,n_drone);
    for i=1:n_drone
        head(i)=scatter3(drone(i,1), drone(i,2), drone(i,3), 50, 'filled', 'MarkerFaceColor', 'b');
        quiver_handle(i)=quiver3(drone(i,1), drone(i,2), drone(i,3), 0, 0, 0, 'r', 'LineWidth', 1.5, 'MaxHeadSize', 0.5);
    end

    %% Boucle de simulation
    k =0;
    while true
        %Fonction qui calcule la speed T+1 et update la pos T0
        swarm.update_speed(dt,r,swarm_weights, weights, pondeTarg, sat)
                
        drone=zeros(n_drone,3);
        speed=zeros(n_drone,3);
        for i=1:n_drone
            drone(i,:)=swarm.Drones{i}.posState;
            speed(i,:)=swarm.Drones{i}.speedState;
        end

        for i=1:n_drone
             set(head(i), 'XData', drone(i,1), 'YData', drone(i,2), 'ZData', drone(i,3));
             set(quiver_handle(i), 'XData', drone(i,1), 'YData', drone(i,2), 'ZData', drone(i,3), ...
                 'UData', speed(i,1), 'VData', speed(i,2), 'WData', speed(i,3));
        end
       
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

