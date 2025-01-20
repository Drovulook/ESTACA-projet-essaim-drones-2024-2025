% params.m
% Configuration des paramètres pour les drones multirotors et à voilure fixe

% Paramètres du multirotor
multirotorParams = struct();
multirotorParams.MinSpeed = 0;                % Vitesse minimale [m/s]
multirotorParams.MaxSpeed = 10;               % Vitesse maximale vers l'avant [m/s]
multirotorParams.CruiseSpeed = 8;            % Vitesse de croisière [m/s]
multirotorParams.MaxVarioUp = 5;              % Taux de montée maximal [m/s]
multirotorParams.MaxVarioDown = -3;           % Taux de descente maximal [m/s]
multirotorParams.MaxTurnGLoad = 2.0;          % Charge en G maximale lors d'un virage
multirotorParams.MaxTurnRate = pi / 6;        % Taux de virage maximal [rad/s]
multirotorParams.MaxClimbRate = 15;            % Taux de montée maximal [m/s]


% Paramètres de la voilure fixe
fixedWingParams = struct();
fixedWingParams.MinSpeed = 10;                % Vitesse aérienne minimale [m/s] (vitesse de décrochage)
fixedWingParams.MaxSpeed = 50;                % Vitesse aérienne maximale [m/s]
fixedWingParams.CruiseSpeed = 40;             % Vitesse de croisière [m/s]
fixedWingParams.MaxClimbRate = 10;            % Taux de montée maximal [m/s]
fixedWingParams.MaxDescentRate = -5;          % Taux de descente maximal [m/s]
fixedWingParams.MaxBankAngle = deg2rad(45);   % Angle d'inclinaison maximal pour les virages [radians]
fixedWingParams.MaxTurnGLoad = 3.0;           % Charge en G maximale lors d'un virage
fixedWingParams.MaxTurnRate = pi / 4;         % Taux de virage maximal [rad/s]

% Sauvegarder les paramètres dans un fichier .mat
save('data/params.mat', 'multirotorParams', 'fixedWingParams');
