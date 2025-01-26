function [neighborI, nnmax] = VoroiNeighbor(posStateMatrix,n)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% La fonction permet de calculer la matrice d'adjacence des drones avec la
% méthode de delaunay. 
% On obtient sur chaque ligne toutes les interactions pour 1 drone.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


            %La matrice NeighborI permet, lorsque parsée avec stateA, de
            %donner une matrice ou sur la ligne, on a le drone, et sur
            %chaque colonne, le numéro de ligne de ses voisins dans un
            %tétraèdre
            % Actuellement la partie du dessus fonctionne en tétraèdre, si
            % on veut la changer pour utiliser TOUS les voisins
            % (fonctionnement omniscient télépathique), il faut boucler
            % pour chaque drone, tous les autres drones, et computer leur
            % distances comme fait dessous, je trouve que c'est plus
            % élégant au-dessus
            %La pb avec ça, c'est le comportement dans un cas
            %coplanaire/colinéaire, ça marche pas

    if n > 4

        % Calcul des voisins de voronoi avec une triangulation de chaque drone
        dTri = delaunay(posStateMatrix); %Création de la matrice de triangulation
        vn = sparse(dTri, dTri(:,[2 3 4 1]),1); % décalage des indices qui permet de créer la matrice des voisins de voroi (matrice d'adjacence)
        %vn est une matrice sparce, on a viré les zeros pour gain de
        %place
    
        vn = vn | vn'; %On rend vn symétrique pour s'assurer que la relation de voisinage ets bijective
        
        listI = repmat(1:n,1,n); % liste de [1,2,3,..,n_drones,1,2,3,... nfois]
        ns = listI(vn);
        %en indexant listI par vn, on obtient les indices des voisins
    
        nn = sum(vn); % Toujours en sparse, on a le nb de voisin par drone (1 drone/ligne)
        nnmax = max(nn); % Le drone qui a le plus de voisins
    
        neighborI = full(sparse(nn+1,1:n,n+1)); %Init matrice de voisinage finale
        neighborI = cumsum(neighborI(1:end-1,:),1); 
        neighborI(neighborI==0) = ns;
        neighborI = neighborI';
    else
        neighborI = [];
        nnmax = n-1;
        for k = 1:n
            neighborI = [neighborI ; setdiff(1:n, k, 'stable')];
        end
    end
end