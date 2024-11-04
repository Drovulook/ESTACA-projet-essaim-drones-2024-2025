% utilities/generateLawnmowerPattern.m
% Génère des points de passage pour la couverture d'une zone en motif de va-et-vient (lawnmower)

function waypoints = generateLawnmowerPattern(area, altitude, spacing)
    xmin = area(1);
    xmax = area(2);
    ymin = area(3);
    ymax = area(4);
    waypoints = [];
    x = xmin:spacing:xmax;
    direction = 1; % Commencer en montant
    for xi = x
        if direction > 0
            y = ymin:spacing:ymax;
        else
            y = ymax:-spacing:ymin;
        end
        for yi = y
            waypoints = [waypoints; xi, yi, altitude];
        end
        direction = -direction; % Changer de direction
    end
end
