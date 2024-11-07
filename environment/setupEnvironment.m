% environment/setupEnvironment.m
% Initialise l'environnement de simulation 3D

function env = setupEnvironment()
    % Créer un scénario pour les UAV avec un taux de mise à jour de 10 Hz et une durée de 200 secondes
    env = uavScenario("UpdateRate", 10, "StopTime", 200);

    % Ajouter une surface de sol grise au niveau z = 0
    ground_x = [-150, 150, 150, -150]; % Définir les coins de la surface selon x (en mètres)
    ground_y = [-150, -150, 150, 150]; % Définir les coins de la surface selon y (en mètres)
    ground_z = [0, 0, 0, 0]; % Niveau du sol à z = 0 (en mètres)
    
    % Définir les zones de l'espace aérien en appelant la fonction de configuration des zones
    setupZones;
end
