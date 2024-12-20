function targetInfluence = target_pond(target_list, posStateMatrix, threshold_radius, swarm)

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
        wp_id = swarm.FixedWing{idx}.CurrentWaypoint;
        WpMatrix = [WpMatrix ; swarm.FixedWing{idx}.Waypoints(wp_id,:)];
    end

    nmulti = size(swarm.MultiRotor, 2);

    T_x = [target_list(:,1)' - posStateMatrix(1:nmulti, 1); WpMatrix(:,1) - posStateMatrix(nmulti + 1:end, 1)];
    T_y = [target_list(:,2)' - posStateMatrix(1:nmulti, 2); WpMatrix(:,2) - posStateMatrix(nmulti + 1:end, 2)];
    T_z = [target_list(:,3)' - posStateMatrix(1:nmulti, 3); WpMatrix(:,3) - posStateMatrix(nmulti + 1:end, 3)];
    
    T_eucli = sqrt(T_x.^2 + T_y.^2 + T_z.^2);

    for idx = 1:length(swarm.FixedWing) 
        if T_eucli(nmulti + idx) < threshold_radius
            swarm.FixedWing{idx}.CycleWaypoint;
        end
    end

    dist_min_target = 20;
    k = 0.5;
    f = @(x) (tanh(k * (x - dist_min_target)));

    Y = f(T_eucli);    

    T_x_pond = T_x./T_eucli;
    T_y_pond = T_y./T_eucli;
    T_z_pond = T_z./T_eucli;

    T_x_pond = T_x_pond.*Y;
    T_y_pond = T_y_pond.*Y;
    T_z_pond = T_z_pond.*Y;
    

    targetInfluence = [T_x_pond T_y_pond T_z_pond];


end