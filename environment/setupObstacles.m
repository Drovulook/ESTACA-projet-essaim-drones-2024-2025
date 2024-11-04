% environment/obstacles.m
% Définit les obstacles dans l'environnement

% Ajouter un obstacle de type cylindre
addMesh(env, 'cylinder', {[100 100 10], [0 30]}, [0 1 1]); % Position (50,50), rayon 5, hauteur 20