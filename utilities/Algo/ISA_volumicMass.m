function [rho] = ISA_volumicMass(altitude)
    % on considère de l'air sec
    
    rho0=1.225; % kg/m^3
    T0=288;     % K
    g=9.81;     % m/s²
    M=0.0289644;% kg/mol
    R=8.31446261815;    % J/K/mol
    %Rs=R/M;  % J*K/kg constante spécifique de l'air
    a=0.0065;   % K/m

    Tz=T0+altitude*a;
    rho=rho0*(1-a*altitude/Tz)^(M*g/(R*a));
end

