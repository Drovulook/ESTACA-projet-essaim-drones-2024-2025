function realTimePlot(swarm)

n = length(swarm.Drones);
coords = [];

for i = 1:n
    coords = [coords ; swarm.Drones{i}.posState];
            
x = coords(:, 1);
y = coords(:, 2);
z = coords(:, 3);

% Ploter les points en 3D avec des lignes connectant les points
plot3(x, y, z, '-o'); % '-o' relie les points par des lignes et ajoute des marqueurs
xlabel('X');
ylabel('Y');
zlabel('Z');
title('Points connectés en 3D');
grid on;



end

