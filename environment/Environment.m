classdef Environment < handle
    properties
        UpdateRate        % Taux de mise à jour de l'environnement
        StopTime          % Temps auquel la simulation s'arrête
        Env               % Objet représentant le scénario de simulation UAV (uavScenario)
        GroundDimensions  % Dimensions de la zone au sol (longueur, largeur)
        GroundCoordinates % Coordonnées de la surface au sol en x, y, z
        
        ZonesList = {}      % Liste pour stocker toutes les zones
        TargetsList = {}    % Liste pour stocker toutes les cibles
        ObstaclesList = {}  % Liste pour stocker tous les obstacles

        DegradedMode = false   % Mode dégradé => erreurs / bruits sur les calculs 
                               % (par défaut en false)
    end
    
    methods
        % Constructeur
        function obj = Environment(updateRate, stopTime, ground_x, ground_y, ground_z, varargin)
            obj.UpdateRate = updateRate;
            obj.StopTime   = stopTime;

            % Créer le uavScenario (si vous l’utilisez)
            obj.Env = uavScenario("UpdateRate", updateRate, "StopTime", stopTime);

            % Définir les coordonnées au sol
            obj.GroundCoordinates = struct('x', ground_x, ...
                                           'y', ground_y, ...
                                           'z', ground_z);
            % Calculer les dimensions
            obj.GroundDimensions = [max(ground_x) - min(ground_x), ...
                                    max(ground_y) - min(ground_y)];

            if ~isempty(varargin)
                obj.DegradedMode = varargin{1}; 
            end
        end
        
        %-------------------- ADDERS ------------------------%
        function addZone(obj, zone)
            obj.ZonesList{end+1} = zone;
        end
        
        function addTarget(obj, target)
            obj.TargetsList{end+1} = target;
        end

        function addObstacle(obj, obstacle)
            obj.ObstaclesList{end+1} = obstacle;
        end
        
        %-------------------- DISPLAY -----------------------%
        function displayEnvironmentInfo(obj)
            fprintf("Taux de mise à jour : %d\n", obj.UpdateRate);
            fprintf("Temps d'arrêt : %d\n", obj.StopTime);
            fprintf("Dimensions du sol : [%.2f, %.2f]\n", obj.GroundDimensions);
            fprintf("Nombre de zones : %d\n", numel(obj.ZonesList));
            fprintf("Nombre de cibles : %d\n", numel(obj.TargetsList));
            fprintf("Nombre d'obstacles : %d\n", numel(obj.ObstaclesList));
        end

        %-------------------- ZONES HELPER ------------------%
        % (Optionnel) Un exemple de méthode si vous voulez un 
        %  format CSV spécifique pour les Zones.
        %  Fichier CSV attendu avec colonnes : 
        %  Name,Type,Category,CenterX,CenterY,CenterZ,Dim1,Dim2,Dim3,Tilt,Status
        function defineZonesFromFile(obj, filename)
            T = readtable(filename, 'Delimiter', ',');
            
            % Contrôle minimal des colonnes
            neededCols = ["Name","Type","Category","CenterX","CenterY","CenterZ", ...
                          "Dim1","Dim2","Dim3","Tilt","Status"];
            for c = 1:numel(neededCols)
                if ~ismember(neededCols(c), T.Properties.VariableNames)
                    error('Missing column "%s" in "%s".', neededCols(c), filename);
                end
            end

            for i = 1:height(T)
                % Extraire
                name     = T.Name{i};
                type     = T.Type{i};
                category = T.Category{i};
                center   = [T.CenterX(i), T.CenterY(i), T.CenterZ(i)];
                dims     = [T.Dim1(i), T.Dim2(i), T.Dim3(i)];
                tilt     = T.Tilt(i);
                status   = T.Status{i};

                % Construire l’objet Zone
                newZone = Zone(name, type, category, center, dims, tilt, status, obj);

                % Ajouter à la liste
                obj.addZone(newZone);
            end
        end

        %-------------------- TARGETS HELPER -----------------%
        %  Fichier CSV attendu avec colonnes :
        %  Name,CenterX,CenterY,CenterZ,Status,ObservabilityScore
        function defineTargetsFromFile(obj, filename)
            T = readtable(filename, 'Delimiter', ',');
            
            neededCols = ["Name","CenterX","CenterY","CenterZ","Status","ObservabilityScore"];
            for c = 1:numel(neededCols)
                if ~ismember(neededCols(c), T.Properties.VariableNames)
                    error('Missing column "%s" in "%s".', neededCols(c), filename);
                end
            end

            for i = 1:height(T)
                name       = T.Name{i};
                position   = [T.CenterX(i), T.CenterY(i), T.CenterZ(i)];
                status     = T.Status{i};
                obsScore   = T.ObservabilityScore(i);

                % Construire l’objet Target
                newTarget = Target(name, position, status, obsScore, obj);

                % Ajouter à la liste
                obj.addTarget(newTarget);
            end
        end

        %-------------------- OBSTACLES HELPER --------------%
        %  Fichier CSV attendu avec colonnes :
        %  Name,Type,Category,CenterX,CenterY,CenterZ,Dim1,Dim2,Dim3,TiltAngle,Status
        function defineObstaclesFromFile(obj, filename)
            T = readtable(filename, 'Delimiter', ',');

            neededCols = ["Name","Type","Category","CenterX","CenterY","CenterZ", ...
                          "Dim1","Dim2","Dim3","TiltAngle","Status"];
            for c = 1:numel(neededCols)
                if ~ismember(neededCols(c), T.Properties.VariableNames)
                    error('Missing column "%s" in "%s".', neededCols(c), filename);
                end
            end

            for i = 1:height(T)
                name     = T.Name{i};
                type     = T.Type{i};
                category = T.Category{i};
                center   = [T.CenterX(i), T.CenterY(i), T.CenterZ(i)];
                dims     = [T.Dim1(i), T.Dim2(i), T.Dim3(i)];
                tilt     = T.TiltAngle(i);
                status   = T.Status{i};

                % Construire un nouvel Obstacle
                newObstacle = Obstacle(name, type, category, center, dims, tilt, status, obj);

                % Ajouter dans la liste
                obj.addObstacle(newObstacle);
            end
        end

        %-------------------- AUTRES ------------------------%
        % Exemple de récupération de "matrice" ou "table" 
        % de positions depuis les zones. (À adapter si besoin.)
        function zone_pos_weight_matrix = get_zones_pos_weights(obj)
            zone_pos_weight_matrix = [];
            for i = 1:length(obj.ZonesList)
                % Par exemple, vous pourriez rassembler 
                % [ZoneName, X, Y, Z, ...] ou [X, Y, Z, Category] etc.
                % Ici, on illustre juste la concaténation :
                Z = obj.ZonesList{i};
                row = [Z.CenterPosition, Z.Dimensions];
                zone_pos_weight_matrix = [zone_pos_weight_matrix; row];
            end
        end
    end
end
