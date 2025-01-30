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
        function obj = MultirotorDrone(id, initialPosition, params, swarm)
            % Call the base class constructor
            obj@DroneBase(id, 'multirotor', initialPosition, swarm);

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
            obj.NominalVoltage = params.NominalVoltage;
            obj.tankVolume          = params.TankVolume;
            obj.AutonomyMins        = params.AutonomyMins;
            obj.ReloadMins          = params.ReloadMins;

            % If you want to unify "autonomy" with "AutonomyMins":
            obj.autonomy = obj.AutonomyMins / 60;  % hours
            obj.remainingCapacity=obj.NominalCapacity;
        end

        % Méthode de calcul de l'autonomie
        function compute_autonomy(obj, dt)

            if ~(contains(obj.phase,'stand-by')||contains(obj.phase,'reload'))
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
                powerLogSize=sum(obj.powerLog>0);
                obj.mean_consumption=(obj.mean_consumption*powerLogSize+powerNow)/(powerLogSize+1);
                obj.powerLog=[obj.powerLog powerNow];

                if (obj.NominalCapacity == 0)
                    % moteur thermique
                    energie_consomme=obj.powerLog(end)/obj.yieldThermo*dt/3600;
                    obj.remainingCapacity=obj.remainingCapacity-energie_consomme;
                else
                    % moteur electrique
                    capacite_consomme=power(obj.powerLog(end)/obj.yield/obj.NominalVoltage, obj.k_peukert)*dt/3600; %Wh
                    obj.remainingCapacity=obj.remainingCapacity-capacite_consomme*obj.NominalVoltage;
                end

                % calcul de l'autonomie
                if(obj.NominalVoltage>0)
                    % calc bat
                    I=obj.mean_consumption/obj.NominalVoltage/obj.yield;
                    obj.autonomy=obj.remainingCapacity/(I^obj.k_peukert);
                else
                    % calc fuel
                    obj.autonomy=obj.remainingCapacity*obj.yieldThermo/obj.mean_consumption;
                end
                % autonomie en heures

                if obj.conditionReturnToBase && contains(obj.phase,'airborn')
                    obj.setPhase('return');
                end
            else
                obj.powerLog=[obj.powerLog 0];
            end

            test_capacity=obj.remainingCapacity;
            test_phase=obj.phase;
            test_need=obj.needReplacement;


            if contains(obj.phase,'reload')
                obj.charge(dt);
            end
        end


        function condition=conditionReturnToBase(obj)
            % Dans ce qui suit le taux de monté est déduit du Vcruise, donc
            % Vsol<=Vcruise. Autrement dit Vcruise est la norme du vecteur
            % vitesse choisi pour le calcul et ce peut importe si l'on
            % monte/descend.

            distance=obj.posLog(end,:)-obj.Destination;
            pente=-asin(distance(3)/norm(distance)); % négatif car repère avion

            % pente de retour maximale
            if distance(3)<=0 && contains(obj.phase, 'airborn')
                penteMax=asin(obj.MaxVarioDown/obj.CruiseSpeed);
            else
                penteMax=asin(obj.MaxVarioUp/obj.CruiseSpeed);
            end

            % calcul le temps de retour selon la capacité du drone à
            % descendre rapidement ou non
            if pente<penteMax
                tretour=norm(distance(1:2))/cos(pente)/obj.CruiseSpeed+...
                    abs(distance(3)+norm(distance(1:2))*tan(penteMax))/abs(obj.MaxVarioDown);
                % calcul du temps horizontal + calcul du temps vertical
                % déduis de la descente durant l'approche
            elseif pente>penteMax
                % le drone peut descendre en ligne droite
                tretour=norm(distance)/obj.CruiseSpeed;
            else
                tretour=norm(distance)/obj.CruiseSpeed+distance(3)/abs(obj.MaxVarioDown);
            end

            Epp=obj.mass*9.81*distance(3);
            energyRTB=obj.mean_consumption*tretour-Epp*obj.yield;
            energyRTB=energyRTB/3600;

            if energyRTB/obj.NominalCapacity>0.05
                returnCountdownTime=(obj.remainingCapacity-energyRTB)/obj.mean_consumption*3600;
            else
                returnCountdownTime=(obj.remainingCapacity-obj.NominalCapacity*0.05)/obj.mean_consumption*3600;
            end

            if returnCountdownTime<tretour*3
                obj.needReplacement=1;
            end

            if energyRTB*1.2>obj.remainingCapacity || obj.remainingCapacity/obj.NominalCapacity<0.05
                condition=true;
            else
                condition=false;
            end
        end

    end
end
