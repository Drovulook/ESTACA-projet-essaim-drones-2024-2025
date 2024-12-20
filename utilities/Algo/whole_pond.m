function [desiredVector] = whole_pond(swarmInfluence, speedInfluence, targetInfluence, avoidInfluence, weights)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Permet de calculer le nouveau vecteur vitesse en comutant toutes les
% pondérations
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 %distances pondérées et normalisées = matrices d'attraction
 
            swarm_weight = weights(1);
            speed_weight = weights(2);
            target_weight = weights(3);
            avoid_weight = weights(4);

            speed_x = (swarmInfluence(:,1) * swarm_weight + speedInfluence(:,1) * speed_weight + targetInfluence(:,1) * target_weight + avoidInfluence(:,1) * avoid_weight);
            speed_y = (swarmInfluence(:,2) * swarm_weight + speedInfluence(:,2) * speed_weight + targetInfluence(:,2) * target_weight + avoidInfluence(:,2) * avoid_weight);
            speed_z = (swarmInfluence(:,3) * swarm_weight + speedInfluence(:,3) * speed_weight + targetInfluence(:,3) * target_weight + avoidInfluence(:,3) * avoid_weight);

            %Concrètement, on pondère une fois les cercles de répulsion,
            %orientation des drones, puis on repondère avec la vitesse
            %actuelle + L'attractivité de la target
            
            desiredVector = [speed_x speed_y speed_z];
end

