function speedinfluence = speed_pond(speedStateMatrix)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Renvoie les matrices d'influence de vitesse (nx1) de chacun des drones.
% ça permet de lisser ses mouvements, et crée une inertie du drone
% Manque a pondérer par le poids adéquat
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    speedNorm = sqrt(sum(speedStateMatrix.^2,2));
    speedinfluence = speedStateMatrix./speedNorm;
    speedinfluence(isnan(speedinfluence)) = 0;

end