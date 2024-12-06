function swarmInfluence = swarm_pond(posStateMatrix, speedStateMatrix, neighborI, n, nnmax, swarm_weights, r, swarm)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Renvoie les matrices d'influence de l'essaim
% On somme les deltas de position relative des voisins proches, pondérés
% avec la matrice d'éloignement (si r1<value, on inverse le vecteur de
% distance, sinon on le laisse tel quel, ça permet d'éloigner les drones
% pour éviter les collisions)
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % On fusionne les matrices + on gère l'indice de fin
    stateA = [posStateMatrix speedStateMatrix ; nan(1,6)];
    
    % Différence de position entre les drones et leurs voisins
    rho_x = reshape(stateA(neighborI,1),n,nnmax) - posStateMatrix(:,1); % Différence axe x
    rho_y = reshape(stateA(neighborI,2),n,nnmax) - posStateMatrix(:,2); % Différence axe y
    rho_z = reshape(stateA(neighborI,3),n,nnmax) - posStateMatrix(:,3); % Différence axe z
    rhon = sqrt(rho_x.^2 + rho_y.^2 + rho_z.^2); % Distance euclidienne en 3D

    %Chaque matrice a le drone par ligne, et ses contacts par
    %dépendance sur la ligne. Ainsi, on peut, en fonction de rhon,
    %qui définit la distance euclidienne, définir si le drone de la
    %case est dans le cercle d'attraction, de répulsion,
    %d'orientation ou d'évitement
    
    %Règles de pondération
    attraction = zeros(size(rhon));
    repulsion = zeros(size(rhon));
    repulsion(rhon < r(1)) = -swarm_weights(1); % Cercle de répulsion
    attraction(rhon >= r(1)) = swarm_weights(2); % Cercle d'attraction

    nmulti = size(swarm.MultiRotor, 2);
    attraction(nmulti + 1:end, :) = 0; %Enlève les fixedwing

    weight_matrix = attraction + repulsion;

    swarminfluence_x = (sum(weight_matrix.*rho_x./rhon, 2, 'omitnan'));
    swarminfluence_y = (sum(weight_matrix.*rho_y./rhon, 2, 'omitnan'));
    swarminfluence_z = (sum(weight_matrix.*rho_z./rhon, 2, 'omitnan'));

    swarmInfluence = [swarminfluence_x swarminfluence_y swarminfluence_z];
    
    %sum(weight_matrix,2) poids par ligne à diviser pour pondérer de la somme
    %On multiplie la projection sur un axe par le poids, qu'on
    %normalise par la norme euclidienne, pour obtenir un nouveau
    %vecteur d'influence

end