classdef Obstacle < handle
    properties
        Env             % Reference to the Environment (optional)
        Name            % e.g., "Obstacle #1"
        Type            % Could be "cylinder", "box", etc. if relevant
        Category        % "Category" if you want, e.g. "physical", "virtual", ...
        CenterPosition  % [x, y, z]
        Dimensions      % [dim1, dim2, dim3]
        TiltAngle       % orientation in degrees
        Status          % "actif"/"inactif", etc.
    end
    
    methods
        %------------------------------------------------------------------
        function obj = Obstacle(name, type, category, centerPosition, dimensions, tiltAngle, status, env)
            obj.Name           = name;
            obj.Type           = type;
            obj.Category       = category;
            obj.CenterPosition = centerPosition;
            obj.Dimensions     = dimensions;
            obj.TiltAngle      = tiltAngle;
            obj.Status         = status;
            if nargin == 8
                obj.Env = env;
            end
        end
        %------------------------------------------------------------------
        function displayInfo(obj)
            fprintf("Obstacle Name     : %s\n", obj.Name);
            fprintf("Type              : %s\n", obj.Type);
            fprintf("Category          : %s\n", obj.Category);
            fprintf("Status            : %s\n", obj.Status);
            fprintf("Center Position   : [%.2f, %.2f, %.2f]\n", obj.CenterPosition);
            fprintf("Dimensions        : [%.2f, %.2f, %.2f]\n", obj.Dimensions);
            fprintf("Tilt Angle (deg)  : %.2f\n", obj.TiltAngle);
        end
        %------------------------------------------------------------------
        function setCenterPosition(obj, newPos)
            if numel(newPos) == 3
                obj.CenterPosition = newPos;
            else
                error("Obstacle::setCenterPosition: newPos must be [x,y,z].");
            end
        end
        %------------------------------------------------------------------
        function setTiltAngle(obj, newTilt)
            obj.TiltAngle = newTilt;
        end
    end
end
