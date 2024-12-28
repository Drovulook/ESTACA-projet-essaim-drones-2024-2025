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
        refSurface          % surface de référence en m^2
        finesse
        Radius
        

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

        end

        % Méthode de calcul de l'autonomie
        function compute_autonomy(obj, dt)

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

    end
end
