classdef Target < handle
    properties
        Env                 % Reference to the Environment that owns this target (optional)
        Name                % Nom de la cible
        Position            % [x, y, z]
        Status              % 'actif', 'inactif', ...
        ObservabilityScore  % numeric value
        AllocatedFleet      % e.g. string or ID for the allocated drones (optional)
    end

    methods
        %------------------------------------------------------------------
        function obj = Target(name, position, status, obsScore, env)
            obj.Name   = name;
            obj.Position = position;
            obj.Status   = status;
            if nargin >= 4
                obj.ObservabilityScore = obsScore;
            else
                obj.ObservabilityScore = 0;  % default
            end

            if nargin == 5
                obj.Env = env;
            end
        end
        %------------------------------------------------------------------
        function displayInfo(obj)
            fprintf("Target Name       : %s\n", obj.Name);
            fprintf("Status            : %s\n", obj.Status);
            fprintf("Position          : [%.2f, %.2f, %.2f]\n", obj.Position);
            fprintf("ObservabilityScore: %.2f\n", obj.ObservabilityScore);
        end
        %------------------------------------------------------------------
        % Optionally, you can add methods to update the target’s position, 
        % status, or do other logic, for example:
        function setPosition(obj, newPos)
            if numel(newPos) == 3
                obj.Position = newPos;
            else
                error("Target::setPosition: newPos must be [x,y,z].");
            end
        end
        % etc.
    end
end
