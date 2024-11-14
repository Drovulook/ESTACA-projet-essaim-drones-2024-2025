function LandGeneration(env)

% Ajouter la/les zone(s) prohibée(s)

nb=[1 3]; %borne nombre de zone

nb_zone = nb(1)+round(-0.5+((nb(2)-nb(1)+1)*rand()));

for j=1:nb_zone
    x = 1+round(-0.5+(2*rand()));
    if x == 1
proZoneType = 'cylinder'; % Types de zone prohibée
    else
proZoneType = 'half_sphere';
    end

a = [-150 150]; %borne coordonée

x = a(1)+round(-0.5+((a(2)-a(1)+1)*rand()));
y = a(1)+round(-0.5+((a(2)-a(1)+1)*rand()));
z = 0;

proZoneCoord = [x,y,z]; % Coordonnées de la zone prohibée [x, y, z] (en mètres)

a = [0 100]; %borne dimension

x = a(1)+round(-0.5+((a(2)-a(1)+1)*rand()));
y = a(1)+round(-0.5+((a(2)-a(1)+1)*rand()));
z = a(1)+round(-0.5+((a(2)-a(1)+1)*rand()));

proZoneDims = [x,y,z]; % Dimensions de la zone prohibée [longueur, largeur, hauteur] (en mètres)
proZoneTilt = 0; % Angle d'inclinaison pour la zone prohibée

env.addZone(Zone(proZoneType, 'P', proZoneCoord, proZoneDims, proZoneTilt, env));

end
%%

% Ajouter la/les zone(s) autorisée(s)

nb=[1 3]; %borne nombre de zone

nb_zone = nb(1)+round(-0.5+((nb(2)-nb(1)+1)*rand()));

for j=1:nb_zone
    x = 1+round(-0.5+(3*rand()));


    switch x
    case 1
proZoneType = 'cylinder'; % Types de zone autorisée
        case 2
proZoneType = 'half_sphere';
        case 3
proZoneType = 'box';
    end

a = [-150 150]; %borne coordonée

x = a(1)+round(-0.5+((a(2)-a(1)+1)*rand()));
y = a(1)+round(-0.5+((a(2)-a(1)+1)*rand()));
z = 0;

proZoneCoord = [x,y,z]; % Coordonnées de la zone autorisée [x, y, z] (en mètres)

a = [0 100]; %borne dimension

x = a(1)+round(-0.5+((a(2)-a(1)+1)*rand()));
y = a(1)+round(-0.5+((a(2)-a(1)+1)*rand()));
z = a(1)+round(-0.5+((a(2)-a(1)+1)*rand()));

proZoneDims = [x,y,z]; % Dimensions de la zone autorisée [longueur, largeur, hauteur] (en mètres)
proZoneTilt = 0; % Angle d'inclinaison pour la zone autorisée

env.addZone(Zone(proZoneType, 'A', proZoneCoord, proZoneDims, proZoneTilt, env));

end
