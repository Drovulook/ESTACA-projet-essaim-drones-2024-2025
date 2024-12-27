function targetInfluence = target_pond(target, posStateMatrix, swarm)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Renvoie la matrice d'influence des cibles pour chaque drone 
% Grossièrement, ça renvoie un vecteur vitesse vers la cible, qui sera
% pondéré avec les autres valeurs
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    %HYPOTHESE SIMPLIFICATRICE, 1 SEULE CIBLE !!

    WpMatrix = zeros(0,3);
    for idx = 1:length(swarm.FixedWing) 
        if swarm.FixedWing{idx}.mode_Follow_waypoint == true
            wp_id = swarm.FixedWing{idx}.CurrentWaypoint;
            WpMatrix = [WpMatrix ; swarm.FixedWing{idx}.Waypoints(wp_id,:)];
        else
            WpMatrix = [WpMatrix ; swarm.target];
        end
    end

    nmulti = size(swarm.MultiRotor, 2);

    T_x = [target(:,1)' - posStateMatrix(1:nmulti, 1); WpMatrix(:,1) - posStateMatrix(nmulti + 1:end, 1)];
    T_y = [target(:,2)' - posStateMatrix(1:nmulti, 2); WpMatrix(:,2) - posStateMatrix(nmulti + 1:end, 2)];
    T_z = [target(:,3)' - posStateMatrix(1:nmulti, 3); WpMatrix(:,3) - posStateMatrix(nmulti + 1:end, 3)];
    
    T_eucli = sqrt(T_x.^2 + T_y.^2 + T_z.^2);

    for idx = 1:length(swarm.FixedWing) 
        if T_eucli(nmulti + idx) < swarm.threshold_radius & swarm.FixedWing{idx}.mode_Follow_waypoint == true
            swarm.FixedWing{idx}.CycleWaypoint;
        end
    end


    %% Orbite drones voilure fixe
    % Pour fixedwing, une fois que target définie, une fois approché à une
    % certaine dist de l'objectif, passer en mode orbite : vecteur tangent
    % à la direction de la cible.

    T_X_fixedwing_tangent = -T_y(nmulti + 1 : end);
    T_Y_fixedwing_tangent = T_x(nmulti + 1 : end);

    T_X_fixedwing_radial = T_x(nmulti + 1 : end);
    T_Y_fixedwing_radial = T_y(nmulti + 1 : end);

    T_eucli_fixedwing = T_eucli(nmulti + 1 : end);

    T_X_fixedwing_tangent(T_eucli_fixedwing >= swarm.orbit_radius) = T_X_fixedwing_radial(T_eucli_fixedwing >= swarm.orbit_radius);
    T_Y_fixedwing_tangent(T_eucli_fixedwing >= swarm.orbit_radius) = T_Y_fixedwing_radial(T_eucli_fixedwing >= swarm.orbit_radius);

    T_x(nmulti + 1 : end) = T_X_fixedwing_tangent;
    T_y(nmulti + 1 : end) = T_Y_fixedwing_tangent;

    
    %%
    % Smoothing de l'attraction pour la distance min d'attraction (si trop
    % proche répulsion)

    k = 0.5;
    f = @(x) (tanh(k * (x - swarm.dist_target_min)));

    Y = f(T_eucli);    

    T_x_pond = T_x./T_eucli;
    T_y_pond = T_y./T_eucli;
    T_z_pond = T_z./T_eucli;

    T_x_pond = T_x_pond.*Y;
    T_y_pond = T_y_pond.*Y;
    T_z_pond = T_z_pond.*Y;
    

    targetInfluence = [T_x_pond T_y_pond T_z_pond];

end