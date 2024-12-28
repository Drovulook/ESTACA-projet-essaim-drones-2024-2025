function [new_communicationMatrix] = communication_simulation(communicationMatrix, posStateMatrix_history, communicationFrequency, swarm)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% La fonction renvoie une matrice de position itérative, elle n'est pas
% complète, afin de simuler des communications. La matrice de position
% reçue pour le computing des positions n'est pas la matrice des positions
% réelles
%
% Le problème est donc le suivant, pour le calcul de l'influence de
% l'essaim les drones n'utilisent pas leur position réelle par rapport aux
% positions supposées des autres. Tous les drones utilisent les positions
% supposées
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    new_communicationMatrix = communicationMatrix;

    for i = 1:communicationFrequency
        if swarm.Drones{end}.hasCommunicated == 1
            swarm.resetCommunications()
        end
        
        for idx = 1:length(swarm.Drones)
            if swarm.Drones{idx}.hasCommunicated == 0
                break
            end
        end
        
        new_communicationMatrix(idx, :) = posStateMatrix_history(idx, :, end);
        swarm.Drones{idx}.hasCommunicated = 1;
    end
end


