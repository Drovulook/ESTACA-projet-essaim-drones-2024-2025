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
        rotorDiameter    % [m^2] surface d'un rotor
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
            obj.rotorDiameter=0.210;
            obj.mass=1;
            obj.batteryNominalVoltage=3.7*3;
        end


        % Méthode de calcul de l'autonomie
        function compute_autonomy(obj, dt)

            % calcul force développée
            if (size(obj.speedLog,1)>1)
                acceleration=1/dt.*(obj.speedLog(end,:)-obj.speedLog(end-1,:));
            else
                acceleration=1./dt.*obj.speedLog(end,:);
            end
            g=9.81;
            totalThrust=obj.mass*g*[0 0 -1]+acceleration*obj.mass*g;

            currentPos=obj.posLog(end,:);
            % calcul puissance développée (non teste)
            rotorForce=norm(totalThrust)/obj.rotorNumber;
            rho=ISA_volumicMass(currentPos(3)); % à ajuster
            rotorSurface=pi*(obj.rotorDiameter/2)^2;
            powerNow=(rotorForce^3/(2*rho*rotorSurface))^0.5*obj.rotorNumber;   % puissance à l'instant [t-dt, t]

            % la ligne suivante permet d'éviter de faire la moyenne de la matrice complète
            if(isempty(obj.powerLog))
                obj.mean_consumption=powerNow;
                obj.powerLog=powerNow;
            else
                obj.mean_consumption=(obj.mean_consumption*size(obj.powerLog)+powerNow)/(size(obj.powerLog)+1);
                obj.powerLog=[obj.powerLog powerNow];
            end


            if (obj.maxCapacity == 0)
                % moteur thermique
            else
                capacite_consomme=power(obj.powerLog(end)/obj.batteryNominalVoltage, obj.k_peukert)*dt/3600; % Wh
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

            obj.conditionReturnToBase
        end


        function condition=conditionReturnToBase(obj)
            % Dans ce qui suit le taux de monté est déduit du Vcruise, donc
            % Vsol<=Vcruise. Autrement dit Vcruise est la norme du vecteur
            % vitesse choisi pour le calcul et ce peut importe si l'on
            % monte/descend.

            distance=obj.Destination-obj.posLog(end,:);
            pente=asin(distance(3)/norm(distance));

            % pente de retour maximale
            if distance(3)<=0
                penteMax=asin(obj.MaxVarioDown/obj.CruiseSpeed);
            else
                penteMax=asin(obj.MaxVarioUp/obj.CruiseSpeed);
            end
            
            % calcul le temps de retour selon la capacité du drone à
            % descendre rapidement ou non
            if pente<penteMax
                tretour=norm(distance(1:2)/cos(pente))/obj.CruiseSpeed+...
                abs(distance(3)+norm(distance(1:2)*tan(pente)))/obj.MaxVarioDown;
                % calcul du temps horizontal + calcul du temps vertical
                % déduis de la descente durant l'approche
            elseif pente>penteMax
                tretour=norm(distance(1:2))/obj.CruiseSpeed+distance(3)/obj.MaxVarioUp;
                % calcul très approximatif
            else
                tretour=norm(distance)/obj.CruiseSpeed;
            end
            
            Epp=obj.mass*9.81*distance(3);
            energyRTB=obj.mean_consumption*tretour+Epp*obj.yield;

            if energyRTB*1.2>obj.remainingCapacity
                condition=true;
            else
                condition=false;
            end
            % tretour=norm(distance)/obj.CruiseSpeed; % vent non pris en compte  
            % %tretour=norm(distance)/(obj.CruiseSpeed+dot(vent,distance)/norm(distance)); % vent pris en compte vecteur à préciser  
        end
    end
end
