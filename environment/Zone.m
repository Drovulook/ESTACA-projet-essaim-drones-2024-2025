classdef Zone < Environment & handle
    properties
        Type            % Type de la zone : 'cylinder', 'halfsphere', 'box' ou 'surface'
        Category        % Catégorie de la zone : 'authorized', 'prohibited', 'maneuvering', 'target'
        CenterPosition  % Position centrale [x, y, z]
        Dimensions      % Dimensions [l1, l2, h] pour longueur, largeur et hauteur/rayon
        TiltAngle       % Angle d'inclinaison en degrés
    end
    
    methods
        % Constructeur pour initialiser une zone avec des propriétés spécifiques, en la reliant à l'environnement
        function obj = Zone(type, category, centerPosition, dimensions, tiltAngle, env)
            % Initialiser la partie environnement
            obj@Environment(env.UpdateRate, env.StopTime, env.GroundCoordinates.x, env.GroundCoordinates.y, env.GroundCoordinates.z);
            
            % Définir les propriétés spécifiques à la zone
            obj.Type = type; % Type de la zone (ex : 'cylinder')
            obj.Category = category; % Catégorie de la zone (ex : 'authorized')
            obj.CenterPosition = centerPosition; % Position centrale [x, y, z]
            obj.Dimensions = dimensions; % Dimensions [longueur, largeur, hauteur/rayon]
            obj.TiltAngle = tiltAngle; % Angle d'inclinaison
        end
        
        % Méthode pour afficher des informations sur la zone
        function displayInfo(obj)
            % Appeler la méthode de la classe de base pour afficher les informations de l'environnement si nécessaire
            %displayEnvironmentInfo@Environment(obj);
            
            fprintf("Type de zone : %s\n", obj.Type);
            fprintf("Catégorie : %s\n", obj.Category);
            fprintf("Position centrale : [%.2f, %.2f, %.2f]\n", obj.CenterPosition);
            fprintf("Dimensions (l1, l2, h) : [%.2f, %.2f, %.2f]\n", obj.Dimensions);
            fprintf("Angle d'inclinaison : %.2f degrés\n", obj.TiltAngle);
        end
        
        % Méthode pour mettre à jour la position centrale de la zone
        function setCenterPosition(obj, newPosition)
            if numel(newPosition) == 3
                obj.CenterPosition = newPosition; % Mettre à jour la position centrale
            else
                error("La nouvelle position centrale doit être un vecteur de 3 éléments [x, y, z].");
            end
        end
        
        % Méthode pour mettre à jour l'angle d'inclinaison de la zone
        function setTiltAngle(obj, newTiltAngle)
            obj.TiltAngle = newTiltAngle; % Mettre à jour l'angle d'inclinaison
        end
    end
end
