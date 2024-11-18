classdef Settings

    properties

        %%%%%%%%%%%%%%%%%%%%%%%%%% Paramètres principaux %%%%%%%%%%%%%%%%%%%%%%%%%%
        dt;     % pas de temps
        temps;  % durée simu
        dims;   % taille de l'environnement 

        %%%%%%%%%%%%%%%%%%%%%%%%%% Initialisation %%%%%%%%%%%%%%%%%%%%%%%%%%
        numMultirotorInit;
        targetListInit;
        spawn_size;
        min_distance;

        %%%%%%%%%%%%%%%%%%%%%%%%%% paramètres de l'algo %%%%%%%%%%%%%%%%%%%%%%%%%%
        r; %Répulsion, évitement, attraction max (rayons)
        swarm_weights; %Pondérations répulsion, alignement, attraction drone, évitement obstacle
        
        weights; %Influence sur le vecteur vitesse de : l'environnement ; la vitesse du drone a t-1 (maniabilité) ; la target
        
        %%%%%%%%%%%%%%%%%%%%%%%%%% autres paramètres %%%%%%%%%%%%%%%%%%%%%%%%%%
        pondeTarg; %Pondération de la value des 2 targets
        %pondDistTarg; %accorde une importance variable aux cibles en fonction de la distance
        satextrem; %Saturation de vitesse projetée
        sat;
    end
    
    methods
        function obj = Settings()
            obj.dt = 0.3;
            obj.temps = 1000;
            obj.dims = [-150 150; -150 150; -10 150];

            obj.numMultirotorInit = 20;
            obj.targetListInit = [100 50 50 ; 50 100 50];
            obj.spawn_size=30;
            obj.min_distance=1.5;

            obj.r = [10 50 100]/2; 
            obj.swarm_weights = [1.4 1 1.2 2];
            obj.weights = [0.5 1.2 1 10]/2; 
            obj.pondeTarg = [10 15]; 
            %obj.pondDistTarg = 0.5; 
            obj.satextrem = 10; 
            obj.sat = [-obj.satextrem obj.satextrem];
            
        end
        

    end
end

