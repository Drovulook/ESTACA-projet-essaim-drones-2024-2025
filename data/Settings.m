classdef Settings

    properties
        r; %Répulsion, évitement, attraction max (rayons)
        swarm_weights; %Pondérations répulsion, alignement, attraction drone, évitement obstacle
        weights; %Influence sur le vecteur vitesse de : l'environnement ; la vitesse du drone a t-1 (maniabilité) ; la target
        pondeTarg; %Pondération de la value des 2 targets
        %pondDistTarg; %accorde une importance variable aux cibles en fonction de la distance
        satextrem; %Saturation de vitesse projetée
        sat;
        temps;
        dt;
    end
    
    methods
        function obj = Settings()
            obj.r = [10 50 100]/2; 
            obj.swarm_weights = [1.4 1 1.2 2];
            obj.weights = [0.5 1.2 0.8]/2; 
            obj.pondeTarg = [10 15]; 
            %obj.pondDistTarg = 0.5; 
            obj.satextrem = 10; 
            obj.sat = [-obj.satextrem obj.satextrem];
            obj.temps = 1000;
            obj.dt = 0.3;
        end
        

    end
end

