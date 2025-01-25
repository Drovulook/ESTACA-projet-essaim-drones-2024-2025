% classes/FixedWingDrone.m
% Classe pour les drones à voilure fixe avec un contrôle de base basé sur l'attitude

classdef FixedWingDrone < DroneBase & handle
    properties
        % ----- Unique to FixedWingDrone (kept) -----
        Destination
        MaxSpeed
        MinSpeed        % Stall speed
        CruiseSpeed
        MaxClimbRate
        MaxDescentRate
        refSurface      % e.g. wing reference area
        finesse
    end

    methods
        % Constructeur pour initialiser un drone à voilure fixe avec ses paramètres
        function obj = FixedWingDrone(id, initialPosition, params)
            % Call the base class constructor
            obj@DroneBase(id, 'fixedwing', initialPosition);

            % Now assign from 'params' (the structure/table row)
            obj.MaxSpeed       = params.MaxSpeed;
            obj.MinSpeed       = params.MinSpeed;
            obj.CruiseSpeed    = params.CruiseSpeed;
            obj.MaxClimbRate   = params.MaxVarioUp;
            obj.MaxDescentRate = params.MaxVarioDwn;
            obj.Destination    = initialPosition; % or some default

            % In DroneBase, we have 'mass', 'NominalVoltage', etc.
            % We can store CSV -> base properties now:
            obj.mass                = params.Mass;
            obj.NominalCapacity     = params.NominalCapacity;
            obj.batteryNominalVoltage = params.NominalVoltage;
            obj.tankVolume          = params.TankVolume;
            obj.AutonomyMins        = params.AutonomyMins;
            obj.ReloadMins          = params.ReloadMins;

            % If you want to unify "autonomy" (in hours) with "AutonomyMins",
            % you could do:
            obj.autonomy = obj.AutonomyMins / 60;  % convert minutes -> hours

            % Example for finesse, refSurface, etc. if in your CSV:
            if isfield(params, 'refSurface')
                obj.refSurface = params.refSurface;
            else
                obj.refSurface = 0.5;  % example default
            end

            if isfield(params, 'finesse')
                obj.finesse = params.finesse;
            else
                obj.finesse = 10; % default
            end
            obj.remainingCapacity= obj.maxCapacity;
        end

        % Méthode de calcul de l'autonomie
        function compute_autonomy(obj, dt)

            % calcul acceleration
            if (size(obj.speedLog,1)>1)
                acceleration=1./dt.*(obj.speedLog(end,:)-obj.speedLog(end-1,:));
            else
                acceleration=1./dt.*obj.speedLog(end,:);
            end

            velocity=obj.speedLog(end,:);
            currentPos=obj.posLog(end-1,:);
            newPos=obj.posLog(end,:);

            %% calcul puissance développée
            % dW=du*F, où '*' est le produit scalaire
            deltaPos=newPos-currentPos;
            pente=acos(norm(deltaPos(1:2)/norm(deltaPos)));
            averageDrag=obj.mass*9.81/obj.finesse;
            %averageDrag=0.5*ISA_volumicMass(currentPos(3))*norm(velocity)^2*obj.refSurface;
            % deltaWorkDrag=dot([velocity(1:2); obj.climbRate] * dt,averageDrag);
            deltaWorkDrag=norm(velocity)*dt*averageDrag;
            Weight=obj.mass*9.81*[0 0 -1];
            deltaWorkWeight=dot(deltaPos,Weight);
            if (deltaWorkWeight+deltaWorkDrag)>0
                powerNow=(deltaWorkWeight+deltaWorkDrag)/dt;   % puissance à l'instant [t-dt, t]
            else
                powerNow=0;
            end

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
                capacite_consomme=power(obj.powerLog(end)/obj.batteryNominalVoltage, obj.k_peukert)*dt/3600; %Wh
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
                penteMax=asin(obj.MaxDescentRate/obj.CruiseSpeed);
            else
                penteMax=asin(obj.MaxClimbRate/obj.CruiseSpeed);
            end

            % calcul le temps de retour selon la capacité du drone à
            % descendre rapidement ou non
            if pente<penteMax
                tretour=norm(distance(1:2)/cos(pente))/obj.CruiseSpeed+...
                    abs(distance(3)+norm(distance(1:2)*tan(pente)))/obj.MaxDescentRate;
                % calcul du temps horizontal + calcul du temps vertical
                % déduis de la descente durant l'approche
            elseif pente>penteMax
                tretour=norm(distance(1:2))/obj.CruiseSpeed+distance(3)/obj.MaxClimbRate;
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
