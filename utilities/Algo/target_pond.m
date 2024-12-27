function targetInfluence = target_pond(target, posStateMatrix, swarm)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Renvoie la matrice d'influence des cibles pour chaque drone 
% Grossièrement, ça renvoie un vecteur vitesse vers la cible, qui sera
% pondéré avec les autres valeurs
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    %HYPOTHESE SIMPLIFICATRICE, 1 SEULE CIBLE !!

    WpMatrix = zeros(0,3); % Pour chaque drone donne sa target
    followMatrix = zeros(0,1); % 1 si target, 0 si waypoint

    for idx = 1:length(swarm.Drones) 
        if swarm.Drones{idx}.mode_Follow_waypoint == true
            followMatrix = [followMatrix ; 0];
            wp_id = swarm.Drones{idx}.CurrentWaypoint;
            WpMatrix = [WpMatrix ; swarm.Drones{idx}.Waypoints(wp_id,:)];
        else
            followMatrix = [followMatrix ; 1];
            WpMatrix = [WpMatrix ; swarm.target];
        end
    end

    nmulti = size(swarm.MultiRotor, 2);
    nb_drones = size(swarm.Drones, 2);

    T_x = WpMatrix(:,1) - posStateMatrix(:, 1); % Distance du drone à sa target pprojetée
    T_y = WpMatrix(:,2) - posStateMatrix(:, 2);
    T_z = WpMatrix(:,3) - posStateMatrix(:, 3);
  
    T_eucli = sqrt(T_x.^2 + T_y.^2 + T_z.^2);

    for idx = 1:length(swarm.Drones) 
        if T_eucli(idx) < swarm.threshold_radius & swarm.Drones{idx}.mode_Follow_waypoint == true
            swarm.Drones{idx}.CycleWaypoint;
        end
    end


    %% Orbite drones voilure fixe
    % Pour fixedwing, une fois que target définie, une fois approché à une
    % certaine dist de l'objectif, passer en mode orbite : vecteur tangent
    % à la direction de la cible.
    % A appliquer SSI Follow wp 1 car sinon on veut que le drone franchisse
    % le WP

    row = nmulti + 1 : nb_drones;

    T_X_fixedwing_tangent = -T_y(row(followMatrix(row) == 1)); % on crée le vecteur tangent dans le plan XY
    T_Y_fixedwing_tangent = T_x(row(followMatrix(row) == 1));


    T_X_fixedwing_radial = T_x(row(followMatrix(row) == 1));
    T_Y_fixedwing_radial = T_y(row(followMatrix(row) == 1));

    T_eucli_fixedwing = T_eucli(row(followMatrix(row) == 1));

    T_X_fixedwing_tangent(T_eucli_fixedwing >= swarm.orbit_radius) = T_X_fixedwing_radial(T_eucli_fixedwing >= swarm.orbit_radius); % Si distance a la cible passée, on passe en mode orbite
    T_Y_fixedwing_tangent(T_eucli_fixedwing >= swarm.orbit_radius) = T_Y_fixedwing_radial(T_eucli_fixedwing >= swarm.orbit_radius); % Pas besoin de Z car dans le plan pour être orthogonal

    T_x(row(followMatrix(row) == 1)) = T_X_fixedwing_tangent; % on réinjecte le calcul dans les matrices totales 
    T_y(row(followMatrix(row) == 1)) = T_Y_fixedwing_tangent;


    
    %%
    % Smoothing de l'attraction pour la distance min d'attraction (si trop
    % proche répulsion)

    k = 0.5;
    f = @(x) (tanh(k * (x - swarm.dist_target_min)));


    Y = zeros(size(T_eucli, 1), 1) + 1;

    Y(followMatrix == 1) = f(T_eucli(followMatrix == 1));  % On l'applique que sur les drones en mode target

    T_x_pond = T_x./T_eucli;
    T_y_pond = T_y./T_eucli;
    T_z_pond = T_z./T_eucli;

    T_x_pond = T_x_pond.*Y;
    T_y_pond = T_y_pond.*Y;
    T_z_pond = T_z_pond.*Y;
    

    targetInfluence = [T_x_pond T_y_pond T_z_pond];

end