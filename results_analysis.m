close all
clear all

new_main

figure
subplot(2,1,1)
hold on
for i=1:size(swarm.Drones,2)
    if swarm.Drones{i}.Type=="multirotor"
        plot(vecnorm(swarm.Drones{i}.speedLog'))
    end
end
title("Vitesses")

subplot(2,1,2)
hold on
for i=1:size(swarm.Drones,2)
    if swarm.Drones{i}.Type=="multirotor"
        plot(swarm.Drones{i}.powerLog)
    end
end
title("Puissance")
drones_capacity=[];
for i=1:size(swarm.Drones,2)
    if swarm.Drones{i}.Type=="multirotor"
        drones_capacity=[drones_capacity swarm.Drones{i}.remainingCapacity];
    end
end