% classes/MultirotorDrone.m
% Classe pour les drones multirotors avec contrôle basé sur l'attitude

classdef MultirotorDrone < DroneBase & handle
    properties
        Destination      % Position cible actuelle du drone
        MaxSpeed         % Vitesse maximale autorisée
        MinSpeed         % Vitesse minimale
        CruiseSpeed      % Vitesse de croisière du drone
        MaxVarioUp       % Taux de montée maximal autorisé
        MaxVarioDown     % Taux de descente maximal autorisé
        MaxTurnGLoad     % Charge en G maximale lors d'un virage
        MaxTurnRate      % Taux de virage maximal [rad/s]
        % Controller       % Instance du contrôleur d'attitude

        rotorNumber     % [1]
        rotorSurface    % [m^2] surface d'un rotor
    end

    methods
        % Constructeur pour initialiser un drone multirotor avec ses paramètres
        function obj = MultirotorDrone(id, initialPosition, params)
            obj@DroneBase(id, 'multirotor', initialPosition); % Appel du constructeur de la classe de base

            % Charger les paramètres depuis la structure params
            obj.MaxSpeed = params.MaxSpeed; % Initialiser la vitesse maximale
            obj.CruiseSpeed = params.CruiseSpeed; % Initialiser la vitesse de croisière
            obj.MaxVarioUp = params.MaxVarioUp; % Initialiser le taux de montée maximal
            obj.MaxVarioDown = params.MaxVarioDown; % Initialiser le taux de descente maximal
            obj.MaxTurnGLoad = params.MaxTurnGLoad; % Initialiser la charge en G maximale pour les virages
            obj.MaxTurnRate = params.MaxTurnRate; % Initialiser le taux de virage maximal

            % Initialiser le contrôleur d'attitude pour le drone
            % obj.Controller = BasicAttitudeController(obj);
            obj.posState = initialPosition; % Définir la position actuelle
            obj.Destination = initialPosition; % Définir la destination initiale comme la position de départ

            obj.Radius = 0.75;
            obj.rotorNumber=4;
            obj.rotorSurface=0.05; % m^2 soit 1000 cm^2
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

            % Contraindre le taux de montée pour respecter les limites du multirotor
            if climbRate > obj.MaxVarioUp
                climbRate = obj.MaxVarioUp;
            elseif climbRate < obj.MaxVarioDown
                climbRate = obj.MaxVarioDown;
            end

            % Mettre à jour la position en appliquant la vitesse et le taux de montée
            newPos = currentPos + [velocity(1:2); climbRate] * dt;
            obj.Platform.updatePose('Position', newPos); % Appliquer la nouvelle position au drone


            function SetSpeedWithConstraints(obj, dt, newSpeedVec) %est appelée juste avant d'update la vitesse, pour prendre en compte d'éventuelles contraintes
                climbRate = newSpeedVec(3); % à modifier
                if climbRate > 100
                    climbRate = 100;
                elseif climbRate < -100
                    climbRate = -100;
                end
                RealnewSpeedVec = [newSpeedVec(1), newSpeedVec(2), climbRate];
                obj.speedState = RealnewSpeedVec;
            end

            % calcul force développée
            if (size(obj.speedLog,1)>1)
                acceleration=1/dt*(obj.speedLog(end,:)-obj.speedLog(end,:));
            else
                acceleration=1/dt*obj.speedLog(end,:);
            end
            g=9.81;
            totalThrust=obj.mass*g*[0 0 -1]+acceleration*obj.mass*g;

            % calcul puissance développée (non teste)
            rotorForce=norm(totalThrust)/obj.rotorNumber;
            rho=ISA_volumicMass(obj.currentPos(3)); % à ajuster
            powerNow=(rotorForce^3/(2*rho*obj.rotorSurface))*obj.rotorNumber;   % puissance à l'instant [t-dt, t]

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
