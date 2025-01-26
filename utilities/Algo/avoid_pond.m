function [avoidInfluence] = avoid_pond(posStateMatrix, speedStateMatrix, dt, zones_object_list, altitude_min, dt_evitement_max)

    avoidZones_posDim = zeros(0,6);
   
    %Le besoin c'est une matrice qui renvoie coords + diamètre
    %sphère (à mettre dans Environnement)
    for i = 1:length(zones_object_list)
        avoidZones_posDim = [avoidZones_posDim ; zones_object_list{i}.CenterPosition zones_object_list{i}.Dimensions]; % (n_zones * 6)
        % stockage des zones d'exclusion + dim
    end

    function [X, Y, Z, Eucli] = diffeo(posMatrix, avoidMatrix)
        X = avoidMatrix(:,1)' - posMatrix(:,1); % (n_drones * n_zones)
        Y = avoidMatrix(:,2)' - posMatrix(:,2); % Delta en Y a chaque zone (colonne) par drone (ligne)
        Z = avoidMatrix(:,3)' - posMatrix(:,3); % Delta en Z
        Eucli = sqrt(X.^2 + Y.^2 + Z.^2); % delta centre zone/drone depuis le drone 
    end


    %diffeo(posStateMatrix_t_plus_1, avoidZones_posDim);
    [zoneCenter_delta_x, zoneCenter_delta_y, zoneCenter_delta_z, zoneCenter_delta_eucli] = diffeo(posStateMatrix, avoidZones_posDim);

    % Assignation du poids négatif pour fuir les zones dangereuses -> Fonctionnement par sphère uniquement
    
    % Création de la fonction continue dépendant de tanh
    k = 0.5;
    r = avoidZones_posDim(:,4)'/2;
    f = @(x, r) (tanh(k * (x - r))*10 - 10);
    Y = f(zoneCenter_delta_eucli, r);


    zoneCenter_delta_x = zoneCenter_delta_x.*Y; % On fait une convolution de la fct continue et le delta de distance
    zoneCenter_delta_y = zoneCenter_delta_y.*Y;
    zoneCenter_delta_z = zoneCenter_delta_z.*Y;
    
    zoneCenter_delta_x = sum(zoneCenter_delta_x./ zoneCenter_delta_eucli, 2, 'omitnan');
    zoneCenter_delta_y = sum(zoneCenter_delta_y./ zoneCenter_delta_eucli, 2, 'omitnan');
    zoneCenter_delta_z = sum(zoneCenter_delta_z./ zoneCenter_delta_eucli, 2, 'omitnan');
    
    zoneCenter_delta_x(isnan(zoneCenter_delta_x)) = 0;
    zoneCenter_delta_y(isnan(zoneCenter_delta_y)) = 0;
    zoneCenter_delta_z(isnan(zoneCenter_delta_z)) = 0;

    zoneCenter_delta_z = zoneCenter_delta_z - f(posStateMatrix(:,3), altitude_min);

    %% Evitement vertical prédictif t+3
    % Si dans une zone contestée avec le même vecteur vitesse à t+1, t+2,
    % t+3, alors évitement vertical pleins gaz vers le haut

    for n=1:dt_evitement_max
        posStateMatrix_t_plus_n = posStateMatrix + speedStateMatrix*dt*n;
        [~, ~, ~, zoneCenter_delta_eucli_n] = diffeo(posStateMatrix_t_plus_n, avoidZones_posDim);
        t_plus_n = any(zoneCenter_delta_eucli_n <= avoidZones_posDim(:,4)'/2, 2)*100;
        zoneCenter_delta_z = zoneCenter_delta_z + t_plus_n;
    end

    avoidInfluence = [zoneCenter_delta_x zoneCenter_delta_y zoneCenter_delta_z];

end

