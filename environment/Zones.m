classdef Zones < handle

    properties
        app;
        % Initialise les listes de zones
        authZones = {}; % Zones autorisées
        proZones = {};  % Zones prohibées
        manZones = {};  % Zones de manœuvre
        
        % Créer les caractéristiques de la base
        homeBaseType = 'box'; % Type de la zone pour la base
        homeBaseCoord = [0, 0, 0]; % Coordonnées de la base [x, y, z] (en mètres)
        homeBaseDims = [45, 35, 20]; % Dimensions de la base [longueur, largeur, hauteur] (en mètres)
        homeBaseTilt = 45; % Angle d'inclinaison de la base en degrés
    end

    methods
        function obj = Zones(app)
            obj.app = app

             % Ajouter les zones autorisées
            authZoneType = {obj.homeBaseType};
            authZoneCoord = {obj.homeBaseCoord}; % Coordonnées des zones autorisées [x, y, z] (en mètres)
            authZoneDims = {obj.homeBaseDims}; % Dimensions des zones autorisées [longueur, largeur, hauteur] (en mètres)
            authZoneTilt = {obj.homeBaseTilt}; % Angle d'inclinaison des zones autorisées
            for i = 1:length(authZoneCoord)
                obj.authZones{end+1} = Zone(authZoneType{i}, 'A', authZoneCoord{i}, authZoneDims{i}, authZoneTilt{i}, obj.app.env);
                obj.app.env.addZone(obj.authZones{i}); % Ajouter la zone autorisée à l'environnement
            end
            
            % Ajouter les zones prohibées
            proZoneType = {'half_sphere', 'cylinder'}; % Types de zones prohibées
            proZoneCoord = {[-100, -100, 0], [-40, 60, 0]}; % Coordonnées des zones prohibées [x, y, z] (en mètres)
            proZoneDims = {[100, 100, 50], [30, 30, 50]}; % Dimensions des zones prohibées [longueur, largeur, hauteur] (en mètres)
            proZoneTilt = {0, 0}; % Angle d'inclinaison pour les zones prohibées
            for i = 1:length(proZoneCoord)
                obj.proZones{end+1} = Zone(proZoneType{i}, 'P', proZoneCoord{i}, proZoneDims{i}, proZoneTilt{i}, obj.app.env);
                obj.app.env.addZone(obj.proZones{i}); % Ajouter la zone prohibée à l'environnement
            end
            
            % Ajouter les zones de manœuvre
            manZoneType = {'cylinder'}; % Type de zone de manœuvre
            manZoneCoord = {[0, 0, 0]}; % Coordonnées des zones de manœuvre [x, y, z] (en mètres)
            manZoneDims = {[30, 30, 50]}; % Dimensions des zones de manœuvre [longueur, largeur, hauteur] (en mètres)
            manZoneTilt = {0}; % Angle d'inclinaison des zones de manœuvre
            for i = 1:length(manZoneCoord)
                obj.manZones{end+1} = Zone(manZoneType{i}, 'M', manZoneCoord{i}, manZoneDims{i}, manZoneTilt{i}, obj.app.env);
                obj.app.env.addZone(obj.manZones{i}); % Ajouter la zone de manœuvre à l'environnement
            end
        end
    end
end