function shape = create_shape(shape_type, color, center, dimensions, angle, ax)
    % Paramètres
    x_center = center(1); % Coordonnée x du centre de la forme
    y_center = center(2); % Coordonnée y du centre de la forme
    z_center = center(3); % Coordonnée z du centre de la forme
    l1 = dimensions(1); % Longueur ou diamètre en x
    l2 = dimensions(2); % Largeur en y
    h = dimensions(3); % Hauteur de la forme
    theta = deg2rad(angle); % Convertir l'angle en radians pour la rotation

    % Choix du type de forme
    switch shape_type
        case 'cylinder'
            % Création d'un cylindre
            [xc, yc, zc] = cylinder(l1/2, 20); % Rayon basé sur l1
            zc = zc * h; % Hauteur h
            % Appliquer la rotation et la translation
            rotX = xc * cos(theta) - yc * sin(theta) + x_center;
            rotY = xc * sin(theta) + yc * cos(theta) + y_center;
            zc = zc + z_center;
            shape = surf(ax, rotX, rotY, zc, 'FaceColor', color, 'EdgeColor', color, 'FaceAlpha', 0.3);
            
        case 'box'
            % Création d'un parallélépipède
            [X, Y, Z] = ndgrid([0 1], [0 1], [0 1]);
            X = l1 * (X - 0.5); % Ajuster la taille en x
            Y = l2 * (Y - 0.5); % Ajuster la taille en y
            Z = h * (Z - 0.5); % Ajuster la taille en z
            % Appliquer la rotation et la translation
            rotX = X * cos(theta) - Y * sin(theta) + x_center;
            rotY = X * sin(theta) + Y * cos(theta) + y_center;
            Z = Z + z_center;
            shape = patch(ax, 'Vertices', [rotX(:), rotY(:), Z(:)], ...
                          'Faces', convhull(rotX(:), rotY(:), Z(:)), ...
                          'FaceColor', color, 'EdgeColor', color, 'FaceAlpha', 0.3);

        case 'half_sphere'
            % Création d'une demi-sphère
            [xs, ys, zs] = sphere(20);
            xs = xs(11:end, :) * l1 / 2; % Rayon basé sur l1
            ys = ys(11:end, :) * l1 / 2;
            zs = zs(11:end, :) * h; % Hauteur ajustée
            % Appliquer la rotation et la translation
            rotX = xs * cos(theta) - ys * sin(theta) + x_center;
            rotY = xs * sin(theta) + ys * cos(theta) + y_center;
            zs = zs + z_center;
            shape = surf(ax, rotX, rotY, zs, 'FaceColor', color, 'EdgeColor', color, 'FaceAlpha', 0.3);

        otherwise
            error('Type de forme inconnu.');
    end
end
