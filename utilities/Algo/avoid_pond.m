function [avoidInfluence] = avoid_pond(posStateMatrix, zones_object_list)

    avoidZones_posDim = [];
   
    %Le besoin c'est une matrice qui renvoie coords + diamètre
    %sphère (à mettre dans Environnement)
    for i = 1:length(zones_object_list)
        avoidZones_posDim = [avoidZones_posDim ; zones_object_list(i).CenterPosition zones_object_list(i).Dimensions]; % (n_zones * 6)
        % stockage des zones d'exclusion + dim
    end
    
    zoneCenter_delta_x = avoidZones_posDim(:,1)' - posStateMatrix(:,1); % (n_drones * n_zones)
    zoneCenter_delta_y = avoidZones_posDim(:,2)' - posStateMatrix(:,2);
    zoneCenter_delta_z = avoidZones_posDim(:,3)' - posStateMatrix(:,3);
    zoneCenter_delta_eucli = sqrt(zoneCenter_delta_x.^2 + zoneCenter_delta_y.^2 + zoneCenter_delta_z.^2);
    % delta centre zone/drone depuis le drone 
    

    % Assignation du poids négatif pour fuir les zones dangereuses -> Fonctionnement par sphère uniquement

    condition = zoneCenter_delta_eucli >= avoidZones_posDim(:,4)'/2; % Ceux en dehors reçoivent 0 pour que seuls ceux dedans soient pondérés en évitement
    zoneCenter_delta_x(condition) = 0;
    zoneCenter_delta_y(condition) = 0;
    zoneCenter_delta_z(condition) = 0;
    
    zoneCenter_delta_x = sum(zoneCenter_delta_x./ zoneCenter_delta_eucli, 2, 'omitnan');
    zoneCenter_delta_y = sum(zoneCenter_delta_y./ zoneCenter_delta_eucli, 2, 'omitnan');
    zoneCenter_delta_z = sum(zoneCenter_delta_z./ zoneCenter_delta_eucli, 2, 'omitnan');
    
    zoneCenter_delta_x(isnan(zoneCenter_delta_x)) = 0;
    zoneCenter_delta_y(isnan(zoneCenter_delta_y)) = 0;
    zoneCenter_delta_z(isnan(zoneCenter_delta_z)) = 0;

    avoidInfluence = [zoneCenter_delta_x zoneCenter_delta_y zoneCenter_delta_z];

end

