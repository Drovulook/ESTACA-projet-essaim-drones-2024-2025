function [avoidInfluence] = avoid_pond(posStateMatrix, zones_object_list, altitude_min)

    avoidZones_posDim = zeros(0,6);
   
    %Le besoin c'est une matrice qui renvoie coords + diamètre
    %sphère (à mettre dans Environnement)
    for i = 1:length(zones_object_list)
        avoidZones_posDim = [avoidZones_posDim ; zones_object_list(i).CenterPosition zones_object_list(i).Dimensions]; % (n_zones * 6)
        % stockage des zones d'exclusion + dim
    end
    
    zoneCenter_delta_x = avoidZones_posDim(:,1)' - posStateMatrix(:,1); % (n_drones * n_zones)
    zoneCenter_delta_y = avoidZones_posDim(:,2)' - posStateMatrix(:,2); % Delta en Y a chaque zone (colonne) par drone (ligne)
    zoneCenter_delta_z = avoidZones_posDim(:,3)' - posStateMatrix(:,3); % Delta en Z
    zoneCenter_delta_eucli = sqrt(zoneCenter_delta_x.^2 + zoneCenter_delta_y.^2 + zoneCenter_delta_z.^2);
    % delta centre zone/drone depuis le drone 
    
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

    avoidInfluence = [zoneCenter_delta_x zoneCenter_delta_y zoneCenter_delta_z];

end

