@echo off
cd ..
cd ..

cd D:\gitRepositories\ESTACA-projet-essaim-drones-2024-2025

git init
git add .
set /p input= Description des Modifications : 
git commit -m %input%

git push origin main