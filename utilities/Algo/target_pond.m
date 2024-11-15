function targetInfluence = target_pond(target_list, posStateMatrix, target_weights)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Renvoie la matrice d'influence des cibles pour chaque drone 
% Grossièrement, ça renvoie un vecteur vitesse vers la cible, qui sera
% pondéré avec les autres valeurs
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Différence de position aux targets, en ligne, les drones, en
%colonne la diff à chaque target
    
    T_x = target_list(:,1)' - posStateMatrix(:,1);
    T_y = target_list(:,2)' - posStateMatrix(:,2);
    T_z = target_list(:,3)' - posStateMatrix(:,3);
    
    T_eucli = sqrt(T_x.^2 + T_y.^2 + T_z.^2); % On peut y ajouter de la pondération de cible en fct de la distance ; distance d'attraction max à ajouter (r(3)/w(3))
    T_x_pond = T_x./T_eucli;
    T_y_pond = T_y./T_eucli;
    T_z_pond = T_z./T_eucli;
    
    %Pondération targets
    T_x_pond = T_x_pond .* target_weights; % (n_drone x n_target)
    T_y_pond = T_y_pond .* target_weights;
    T_z_pond = T_z_pond .* target_weights;
    
    T_x_pond = sum(T_x_pond,2)/sum(target_weights); % (n_drone x 1)
    T_y_pond = sum(T_y_pond,2)/sum(target_weights);
    T_z_pond = sum(T_z_pond,2)/sum(target_weights);

    targetInfluence = [T_x_pond T_y_pond T_z_pond];

end