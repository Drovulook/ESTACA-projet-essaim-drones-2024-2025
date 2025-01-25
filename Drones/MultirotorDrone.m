% classes/MultirotorDrone.m
% Classe pour les drones multirotors avec contrôle basé sur l'attitude

classdef MultirotorDrone < DroneBase & handle
    properties
        % ----- Unique to MultirotorDrone (kept) -----
        Destination
        MaxSpeed
        MinSpeed
        CruiseSpeed
        MaxVarioUp
        MaxVarioDown
        MaxTurnGLoad
        %Radius
        rotorNumber
        rotorDiameter
    end

    methods
        % Constructeur pour initialiser un drone multirotor avec ses paramètres
        function obj = MultirotorDrone(id, initialPosition, params)
            % Call the base class constructor
            obj@DroneBase(id, 'multirotor', initialPosition);

            % Assign from 'params'
            obj.MaxSpeed       = params.MaxSpeed;
            obj.MinSpeed       = params.MinSpeed;
            obj.CruiseSpeed    = params.CruiseSpeed;
            obj.MaxVarioUp     = params.MaxVarioUp;
            obj.MaxVarioDown   = params.MaxVarioDwn;
            obj.MaxTurnGLoad   = params.MaxTurnGLoad;
            %obj.Radius         = params.Radius;  % if in CSV
            obj.Destination    = initialPosition;

            % rotor specifics:
            obj.rotorNumber    = params.RotorNumber;
            obj.rotorDiameter  = params.RotorDiameter;  % or rotorSurface, if that’s what you store

            % Now fill base-class properties from CSV
            obj.mass                = params.Mass;
            obj.NominalCapacity     = params.NominalCapacity;
            obj.batteryNominalVoltage = params.NominalVoltage;
            obj.tankVolume          = params.TankVolume;
            obj.AutonomyMins        = params.AutonomyMins;
            obj.ReloadMins          = params.ReloadMins;
            
            % If you want to unify "autonomy" with "AutonomyMins":
            obj.autonomy = obj.AutonomyMins / 60;  % hours
            obj.remainingCapacity=obj.maxCapacity;
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

            if obj.conditionReturnToBase
                obj.phase='return';
            end
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
