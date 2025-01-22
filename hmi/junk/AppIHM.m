classdef AppIHM < matlab.apps.AppBase

    % Properties that correspond to app components
    properties (Access = public)
        UIFigure        matlab.ui.Figure
        PanelVue        matlab.ui.container.Panel
        PanelFlotte     matlab.ui.container.Panel
        PanelZones      matlab.ui.container.Panel
        PanelCibles     matlab.ui.container.Panel
        PanelPerf       matlab.ui.container.Panel
        ButtonVue       matlab.ui.control.Button
        ButtonFlotte    matlab.ui.control.Button
        ButtonZones     matlab.ui.control.Button
        ButtonCibles    matlab.ui.control.Button
        ButtonPerf      matlab.ui.control.Button
        UIAxes3D        matlab.ui.control.UIAxes
        UIAxes2D        matlab.ui.control.UIAxes
    end

    methods (Access = private)
        % Function to switch panels
        function switchPanels(app, panelName)
            % Hide all panels by default
            app.PanelVue.Visible = 'off';
            app.PanelFlotte.Visible = 'off';
            app.PanelZones.Visible = 'off';
            app.PanelCibles.Visible = 'off';
            app.PanelPerf.Visible = 'off';
            
            % Show the selected panel
            switch panelName
                case 'Vue'
                    app.PanelVue.Visible = 'on';
                case 'Flotte'
                    app.PanelFlotte.Visible = 'on';
                case 'Zones'
                    app.PanelZones.Visible = 'on';
                case 'Cibles'
                    app.PanelCibles.Visible = 'on';
                case 'Perf'
                    app.PanelPerf.Visible = 'on';
            end
        end
    end

    methods (Access = private)

        % Code that executes after component creation
        function startupFcn(app)
            % Initialize with Vue panel visible
            switchPanels(app, 'Vue');
        end
    end

    methods (Access = public)

        % Construct app
        function app = DroneSwarmControlApp

            % Create UIFigure and components
            createComponents(app);

            % Execute the startup function
            runStartupFcn(app, @startupFcn);

            % Register the app with App Designer
            registerApp(app, app.UIFigure);

            if nargout == 0
                clear app
            end
        end

        % Code that executes before app deletion
        function delete(app)

            % Delete UIFigure when app is deleted
            delete(app.UIFigure)
        end
    end

    methods (Access = private)

        % Create components
        function createComponents(app)

            % Create UIFigure and hide until all components are created
            app.UIFigure = uifigure('Visible', 'off');
            app.UIFigure.Position = [100 100 800 600];
            app.UIFigure.Name = 'Drone Swarm Control';

            % Create UIAxes3D
            app.UIAxes3D = uiaxes(app.UIFigure);
            title(app.UIAxes3D, '3D View')
            xlabel(app.UIAxes3D, 'X')
            ylabel(app.UIAxes3D, 'Y')
            zlabel(app.UIAxes3D, 'Z')
            app.UIAxes3D.Position = [20 320 400 250];

            % Create UIAxes2D
            app.UIAxes2D = uiaxes(app.UIFigure);
            title(app.UIAxes2D, '2D Projection')
            xlabel(app.UIAxes2D, 'X')
            ylabel(app.UIAxes2D, 'Y')
            app.UIAxes2D.Position = [20 50 400 250];

            % Create Buttons for the menu
            app.ButtonVue = uibutton(app.UIFigure, 'push');
            app.ButtonVue.Position = [450 500 100 30];
            app.ButtonVue.Text = 'Vue';
            app.ButtonVue.ButtonPushedFcn = @(~, ~) switchPanels(app, 'Vue');

            app.ButtonFlotte = uibutton(app.UIFigure, 'push');
            app.ButtonFlotte.Position = [450 450 100 30];
            app.ButtonFlotte.Text = 'Flotte';
            app.ButtonFlotte.ButtonPushedFcn = @(~, ~) switchPanels(app, 'Flotte');

            app.ButtonZones = uibutton(app.UIFigure, 'push');
            app.ButtonZones.Position = [450 400 100 30];
            app.ButtonZones.Text = 'Zones';
            app.ButtonZones.ButtonPushedFcn = @(~, ~) switchPanels(app, 'Zones');

            app.ButtonCibles = uibutton(app.UIFigure, 'push');
            app.ButtonCibles.Position = [450 350 100 30];
            app.ButtonCibles.Text = 'Cibles';
            app.ButtonCibles.ButtonPushedFcn = @(~, ~) switchPanels(app, 'Cibles');

            app.ButtonPerf = uibutton(app.UIFigure, 'push');
            app.ButtonPerf.Position = [450 300 100 30];
            app.ButtonPerf.Text = 'Performances';
            app.ButtonPerf.ButtonPushedFcn = @(~, ~) switchPanels(app, 'Perf');

            % Create Panels for each menu section
            app.PanelVue = uipanel(app.UIFigure, 'Visible', 'on');
            app.PanelVue.Position = [570 300 200 250];
            app.PanelVue.Title = 'Options Vue';

            app.PanelFlotte = uipanel(app.UIFigure, 'Visible', 'off');
            app.PanelFlotte.Position = [570 300 200 250];
            app.PanelFlotte.Title = 'Options Flotte';

            app.PanelZones = uipanel(app.UIFigure, 'Visible', 'off');
            app.PanelZones.Position = [570 300 200 250];
            app.PanelZones.Title = 'Options Zones';

            app.PanelCibles = uipanel(app.UIFigure, 'Visible', 'off');
            app.PanelCibles.Position = [570 300 200 250];
            app.PanelCibles.Title = 'Options Cibles';

            app.PanelPerf = uipanel(app.UIFigure, 'Visible', 'off');
            app.PanelPerf.Position = [570 300 200 250];
            app.PanelPerf.Title = 'Options Performances';

            % Show the figure after all components are created
            app.UIFigure.Visible = 'on';
        end
    end
end
