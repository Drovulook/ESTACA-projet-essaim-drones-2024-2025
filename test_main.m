clear all; close all; clc; % Nettoyer l'espace de travail, fermer les figures et effacer la console

env = Environment(10, 200, [-150, 150, 150, -150], [-150, -150, 150, 150], [0, 0, 0, 0]);

LandGeneration(env);

swarm = SwarmManager(env.Env);
%%
realTimePlot(env, swarm);
