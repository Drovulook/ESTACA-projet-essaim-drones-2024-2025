classdef test < matlab.apps.AppBase
    properties (Access = public)
        UIFigure          matlab.ui.Figure
        SimulationPanel   matlab.ui.container.Panel
        StartButton       matlab.ui.control.Button
        StatusLabel       matlab.ui.control.Label
        StopButton        matlab.ui.control.Button
        % D'autres propriétés pour d'autres panels, etc.
    end
    
    methods (Access = private)
        function startupFcn(app)
            % Initialiser l'interface utilisateur
            initializeSimulationPanel(app);
        end
    end
end