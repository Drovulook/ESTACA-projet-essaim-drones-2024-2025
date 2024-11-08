classdef Environment < handle
    properties
        UpdateRate       % Taux de mise à jour de l'environnement
        StopTime         % Temps auquel la simulation s'arrête
        Env              % Objet représentant le scénario de simulation UAV
        GroundDimensions % Dimensions de la zone au sol (longueur, largeur)
        GroundCoordinates % Coordonnées de la surface au sol en x, y, z
        ZonesList = {};  % Liste pour stocker toutes les zones dans l'environnement
    end
    
    methods
        % Constructeur pour initialiser l'environnement avec le taux de mise à jour, le temps d'arrêt et les dimensions du sol
        function obj = Environment(updateRate, stopTime, ground_x, ground_y, ground_z)
            obj.UpdateRate = updateRate; % Initialiser le taux de mise à jour
            obj.StopTime = stopTime; % Initialiser le temps d'arrêt de la simulation
            % Créer un scénario de simulation UAV avec le taux de mise à jour et le temps d'arrêt
            obj.Env = uavScenario("UpdateRate", updateRate, "StopTime", stopTime);
            % Définir les coordonnées de la surface au sol
            obj.GroundCoordinates = struct('x', ground_x, 'y', ground_y, 'z', ground_z);
            % Calculer les dimensions de la zone au sol (longueur et largeur)
            obj.GroundDimensions = [max(ground_x)-min(ground_x), max(ground_y)-min(ground_y)];            
        end
        
        % Méthode pour ajouter une zone à l'environnement
        function addZone(obj, zone)
            obj.ZonesList{end+1} = zone; % Ajouter la zone à la liste des zones
        end

        % Méthode pour afficher les détails de l'environnement
        function displayEnvironmentInfo(obj)
            fprintf("Taux de mise à jour de l'environnement : %d\n", obj.UpdateRate);
            fprintf("Temps d'arrêt de l'environnement : %d\n", obj.StopTime);
            fprintf("Dimensions du sol : [%.2f, %.2f]\n", obj.GroundDimensions);
            fprintf("Nombre total de zones : %d\n", numel(obj.ZonesList)); % Afficher le nombre de zones
        end
    end
end