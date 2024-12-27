% classes/FixedWingDrone.m
% Classe pour les drones à voilure fixe avec un contrôle de base basé sur l'attitude

classdef FixedWingDrone < DroneBase & handle
    properties
        Destination      % Position cible actuelle du drone
        MaxSpeed         % Vitesse maximale du drone
        MinSpeed         % Vitesse minimale du drone (vitesse de décrochage)
        CruiseSpeed      % Vitesse de croisière du drone
        MaxClimbRate     % Taux de montée maximal autorisé
        MaxDescentRate   % Taux de descente maximal autorisé
        % Controller       % Instance de contrôleur d'attitude de base
        Waypoints        % matrice n*3 des waypoints à cycler
        CurrentWaypoint = 1; %Indice du wp actuel
        refSurface          % surface de référence en m^2
        finesse
        
        
    end

    methods
        % Constructeur pour initialiser un drone à voilure fixe avec ses paramètres
        function obj = FixedWingDrone(id, initialPosition, params)
            obj@DroneBase(id, 'fixedwing', initialPosition); % Appel du constructeur de la classe de base

            % Charger les paramètres depuis la structure params
            obj.MaxSpeed = params.MaxSpeed; % Initialiser la vitesse maximale
            obj.MinSpeed = params.MinSpeed; % Initialiser la vitesse minimale
            obj.MaxClimbRate = params.MaxClimbRate; % Initialiser le taux de montée maximal
            obj.MaxDescentRate = params.MaxDescentRate; % Initialiser le taux de descente maximal

            % Initialiser le contrôleur d'attitude de base
            % obj.Controller = BasicAttitudeController(obj);
            obj.posState = initialPosition; % Définir la position actuelle
            obj.Destination = initialPosition; % Définir la destination initiale comme la position de départ

            obj.Radius = 0.75;
            obj.refSurface=10;
            obj.finesse=10;
        end

        % Méthode pour définir une nouvelle destination pour le drone
        function setDestination(obj, dest)
            obj.Destination = dest; % Mettre à jour la position cible
        end

        % Méthode de mise à jour pour recalculer la position du drone
        function update(obj, dt)
            % Obtenir la position actuelle et calculer la vitesse et le taux de montée
            currentPos = obj.Platform.Pose(1:3); % Position actuelle du drone
            [velocity, climbRate] = obj.Controller.computeControlSignal(currentPos, obj.Destination, dt);

            % Contraindre le taux de montée pour respecter les limites du drone à voilure fixe
            if climbRate > obj.MaxClimbRate
                climbRate = obj.MaxClimbRate;
            elseif climbRate < obj.MaxDescentRate
                climbRate = obj.MaxDescentRate;
            end

            % Mise à jour la position en appliquant la vitesse et le taux de montée
            newPos = currentPos + [velocity(1:2); climbRate] * dt;
            obj.Platform.updatePose('Position', newPos); % Appliquer la nouvelle position au drone

            % calcul acceleration
            if (size(obj.speedLog,1)>1)
                acceleration=1/dt*(obj.speedLog(end,:)-obj.speedLog(end,:));
            else
                acceleration=1/dt*obj.speedLog(end,:);
            end


            %% calcul puissance développée
            % dW=du*F, où '*' est le produit scalaire
            averageDrag=norm(acceleration)/obj.finesse*velocity/norm(velocity);
            %averageDrag=0.5*ISA_volumicMass(currentPos(3))*norm(velocity)^2*obj.refSurface;
            deltaWorkDrag=dot([velocity(1:2); climbRate] * dt,averageDrag);
            Weight=obj.mass*9.81*[0 0 -1];
            deltaWorkWeight=dot(newPos-currentPos,Weight);
            powerNow=(deltaWorkWeight+deltaWorkDrag)/dt;   % puissance à l'instant [t-dt, t]

            % la ligne suivante permet d'éviter de faire la moyenne de la matrice complète
            obj.mean_consumption=(obj.mean_consumption*size(obj.powerLog)+powerNow)/(size(obj.powerLog)+1);
            obj.powerLog=[obj.powerLog powerNow];

            if (obj.Capacite_max == 0)
                % moteur thermique
            else
                capacite_consomme=power(obj.powerLog(end)/obj.batteryNominalVoltage, obj.k_peukert)*dt;
                obj.remainingCapacity=obj.remainingCapacity-capacite_consomme*obj.batteryNominalVoltage;
            end

            % calcul de l'autonomie
            if(obj.batteryNominalVoltage>0)
                % calc bat
                I=obj.mean_consumption/obj.batteryNominalVoltage/obj.yield;
                obj.autonomy=obj.remainingCapacity/(I^obj.k_peukert);
            else
                % calc fuel
                obj.autonomy=obj.remainingCapacity*obj.yield/obj.mean_consumption;
            end
            % autonomie en heures

        end

        function SetSpeedWithConstraints(obj, dt, newSpeedVec)
            climbRate = newSpeedVec(3);
            if climbRate > obj.MaxClimbRate
                climbRate = obj.MaxClimbRate;
            elseif climbRate < obj.MaxDescentRate
                climbRate = obj.MaxDescentRate;
            end
            RealnewSpeedVec = [newSpeedVec(1), newSpeedVec(2), climbRate];
            obj.speedState = RealnewSpeedVec;
        end

        function CycleWaypoint(obj)
            obj.CurrentWaypoint = obj.CurrentWaypoint + 1 ;
            if obj.CurrentWaypoint > size(obj.Waypoints,1)
                obj.CurrentWaypoint = 1;
            end
        end

    end
end
