% environment/setupEnvironment.m
% Initialise l'environnement de simulation 3D

function env = setupEnvironment()
    % Créer un scénario pour les UAV
    env = uavScenario("UpdateRate", 10, "StopTime", 200);
   
    % Ajouter le plancher
    % Définir une grille 200x200 pour les coordonnées X et Y
    [x, y] = meshgrid(linspace(0, 200, 200), linspace(0, 200, 200));
    
    % Définir Z comme une hauteur constante, ce qui en fait une surface plane (par exemple, Z=0 pour un terrain plat)
    z = zeros(size(x));
    addMesh(env, "surface", {x, y, z}, [0.75 0.75 0.75]);
    
    % Ajouter des obstacles
    setupObstacles;
end

% function setupCamera(env)
%     figure;
%     axis equal;
%     view(3);
%     xlabel('X (m)');
%     ylabel('Y (m)');
%     zlabel('Altitude (m)');
%     title('Scénario UAV');
%     grid on;
%     hold on;
% end
