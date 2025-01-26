classdef Zone < handle
    properties
        Env             % Reference to the Environment that owns this zone (optional)
        Name            % Nom de la zone
        Type            % Type de la zone: 'cylinder', 'box', etc. (ou usage spécifique)
        Category        % Catégorie: 'authorized','prohibited','maneuvering', etc.
        CenterPosition  % [x, y, z]
        Dimensions      % [dim1, dim2, dim3] => rayon/hauteur ou longueur/largeur/hauteur, etc.
        TiltAngle       % Inclinaison en degrés
        Status          % 'actif', 'inactif', ...
        ViolationTime
    end
    
    methods
        %------------------------------------------------------------------
        function obj = Zone(name, type, category, centerPosition, dimensions, tiltAngle, status, env)
            obj.Name           = name;
            obj.Type           = type;
            obj.Category       = category;
            obj.CenterPosition = centerPosition;
            obj.Dimensions     = dimensions;
            obj.TiltAngle      = tiltAngle;
            obj.Status         = status;
            obj.ViolationTime  = 0;
            if nargin >= 8
                obj.Env = env;
            end
        end
        %------------------------------------------------------------------
        function displayInfo(obj)
            fprintf("Zone Name        : %s\n", obj.Name);
            fprintf("Type             : %s\n", obj.Type);
            fprintf("Category         : %s\n", obj.Category);
            fprintf("Status           : %s\n", obj.Status);
            fprintf("Center Position  : [%.2f, %.2f, %.2f]\n", obj.CenterPosition);
            fprintf("Dimensions       : [%.2f, %.2f, %.2f]\n", obj.Dimensions);
            fprintf("Tilt Angle (deg) : %.2f\n", obj.TiltAngle);
        end
        %------------------------------------------------------------------
        function setCenterPosition(obj, newPosition)
            if numel(newPosition) == 3
                obj.CenterPosition = newPosition;
            else
                error("Zone::setCenterPosition: newPosition must be [x,y,z].");
            end
        end
        %------------------------------------------------------------------
        function setTiltAngle(obj, newTiltAngle)
            obj.TiltAngle = newTiltAngle;
        end
    end
end
