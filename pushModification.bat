@echo off
REM Vérifie si Git est installé
git --version >nul 2>&1
IF ERRORLEVEL 1 (
    echo Git n'est pas installé ou non configuré dans le PATH.
    pause
    exit /b
)

REM Instructions Git pour ajouter, commit et push
echo Préparation des changements pour Git...
git add .

echo Commit des changements avec un message par défaut...
set /p commitMessage="Entrez un message de commit : "
git commit -m "%commitMessage%"

echo Envoi des changements sur GitHub...
git push origin main

echo Commit et push terminés.
pause