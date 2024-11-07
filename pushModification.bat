@echo off
REM Ce fichier batch sert à initialiser un dépôt git et à effectuer un commit avec une description donnée par l'utilisateur.
REM Il va se déplacer vers le répertoire de travail du projet, ajouter tous les fichiers et pousser les modifications.

cd ..
cd ..

cd D:\gitRepositories\ESTACA-projet-essaim-drones-2024-2025

REM Initialisation du dépôt Git (si nécessaire)
git init
git add .

REM Demande à l'utilisateur de fournir une description pour le commit
set /p input= Description des Modifications : 

REM Effectue le commit avec la description fournie par l'utilisateur
git commit -m "%input%"

REM Pousse les modifications sur la branche principale
git push origin main
