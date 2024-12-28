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
        Radius

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


        % Méthode de calcul de l'autonomie
        function compute_autonomy(obj, dt)

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
