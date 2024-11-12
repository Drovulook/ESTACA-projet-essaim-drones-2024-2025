from pylab import *
import random
import pygame 
import time
import array
import copy

# Window size
window_x = 800
window_y = 800
#Obstacle
Obstaille = [40, 250]
Obs1 = [window_x/3 - Obstaille[0], window_x/3 + Obstaille[0], 0, window_y/2 + Obstaille[1]]
Obs2 = [2*window_x/3 - Obstaille[0], 2*window_x/3 + Obstaille[0], window_y/2 - Obstaille[1], window_y]

class Dot(pygame.sprite.Sprite):
    def __init__(self):
        super(Dot, self).__init__()
         
        # Define the dimension of the surface
        # Here we are making squares of side 25px
        self.surf = pygame.Surface((2, 2))
         
        # Define the color of the surface using RGB color coding.
        self.surf.fill((0, 0, 0))
        self.rect = self.surf.get_rect()
        self.pos = [[window_x/8, window_y-window_y/8]]
        self.vel = [[0, 0]]
        self.acc = [[0, 0]]
        self.dead = [False]
        self.finesse = 0
        self.reachedGoal = [False]
        self.finesseproportion = 0
        self.champ = False

game_speed = 300
Pop = 600
Generations = 200
alive = Pop
MutationRate = 0.01
velmax = 8
 
# defining colors
black = pygame.Color(0, 0, 0)
white = pygame.Color(255, 255, 255)
red = pygame.Color(255, 0, 0)
blue = pygame.Color(0, 0, 255)
green = pygame.Color(0, 255, 0)

# Initialising pygame
pygame.init()
gameOn = True
 
# Initialise game window
pygame.display.set_caption('Dots : Generation 0')
screen = pygame.display.set_mode((window_x, window_y))
 
# FPS (frames per second) controller
fps = pygame.time.Clock()

start_pos = [window_x/8, window_y-window_y/8]
goal_pos = [window_x - window_x/8, window_y/8]

#Dots creation
for k in range(Pop): 
    globals()[f'Pt{k}'] = Dot()


#------------------------------------------------------------------------------------------------------------------------------------------


#Orientation randomization
def rand(pt):
    r = random.random()
    randomAngle = random.random() * 2*pi
    pt.acc.append([cos(randomAngle) * r, sin(randomAngle) * r])


def calcdir(pt,step):
    if step == len(pt.acc):
        pt.acc.append(pt.acc[-1])
        
    if pt.dead[-1] == False and pt.reachedGoal[-1] == False:
        if abs(pt.vel[-1][1] + pt.acc[step][1]) < velmax or abs(pt.vel[-1][1] + pt.acc[step][1]) < velmax :
            pt.vel.append(np.add(pt.vel[-1], pt.acc[step]).tolist())
            pt.pos.append(np.add(pt.pos[-1], pt.vel[-1]).tolist())
        else:
            pt.vel.append([sign(pt.vel[-1][0] + pt.acc[step][0]) * velmax, sign(pt.vel[-1][1] + pt.acc[step][1]) * velmax])
            pt.pos.append(np.add(pt.pos[-1], pt.vel[-1]).tolist())
    else:
        pt.vel.append(pt.vel[-1])
        pt.pos.append(pt.pos[-1])
        
def randed(pt):
    pt.acc.append(pt.acc[-1])
    pt.vel.append(pt.vel[-1])
    pt.pos.append(pt.pos[-1])

#Update their status
def update(pt, step):
    pos = pt.pos[step]
    dead = pt.dead
    #hors écran
    if (pos[0] < 4 or pos[1] < 4 or pos[0] > window_x-4 or pos[1] > window_y-4 or pt.reachedGoal.count(False) > minstep):
        pt.dead.append(True)
    #obstacles
    
    elif (pos[0] > Obs1[0] and pos[0] < Obs1[1] and pos[1] > Obs1[2] and pos[1] < Obs1[3]):
        pt.dead.append(True)
    elif (pos[0] > Obs2[0] and pos[0] < Obs2[1] and pos[1] > Obs2[2] and pos[1] < Obs2[3]):
        pt.dead.append(True)
    
    
    else:
        pt.dead.append(False)
        
    #reachedGoal
    if math.dist(pos, goal_pos) < 8:
        pt.reachedGoal.append(True)
    else:
        pt.reachedGoal.append(False)

    
#Drawinf function
def draw(pt, step):
    pos = pt.pos[step]
    pygame.draw.rect(screen, black,pygame.Rect(pos[0]-2, pos[1]-2, 4, 4))
    if pt.champ == True:
        pygame.draw.rect(screen, green, pygame.Rect(pos[0]-3, pos[1]-3, 6, 6))

#afficheur
def show_alive(choice, color, font, size, gen):
    alive_font = pygame.font.SysFont(font, size)
    alive_surface = alive_font.render('Alive : ' + str(alive), True, color)
    alive_rect = alive_surface.get_rect()
    screen.blit(alive_surface, alive_rect)
  
#Finesse calculation (goal proximity)
def calcfinesse(pt):
    if pt.reachedGoal[-1] == True:
        pt.finesse = 1/16 + 10000/((pt.reachedGoal.count(False))**2)
    else:
        DistToGoal = math.dist(pt.pos[-1], goal_pos)
        pt.finesse = 1/(DistToGoal**2)

    
#If the generation ended

def MortGeneration():
    if alive == 0:
        return(True)

#Choix des parents
def selecParents(pt, finessetot, rand):
    if pt.finesseproportion > rand :
        return(True)

def selecParents2(pt, finessetot, rand):
    if pt.finesse > rand :
        return(True)
    
def selecParents3(pt, finessetot, rand):
    if pt.finesse > 0.005 * finessetot :
        return(True)
        
    
#Mutation bébés
def mutation(pt):
    r = random.random()
    for k in range(len(pt.acc)):
        if random.random() < MutationRate :
                randomAngle = random.random() * 2*pi
                pt.acc[k] = [cos(randomAngle) * (r), sin(randomAngle) * (r)]
    return(pt)
        

def chgmtGeneration():
    for k in range(len(parentsDots)): 
        index = parentsDots[k]
        globals()[f'BbPt{k}'] = Dot()
        globals()[f'BbPt{k}'].acc = copy.deepcopy(globals()[f'Pt{index}'].acc)
    for k in range(Pop): 
        globals()[f'Pt{k}'] = Dot()
        globals()[f'Pt{k}'].acc = copy.deepcopy(globals()[f'BbPt{k}'].acc)
        mutation(globals()[f'Pt{k}'])



#------------------
def lesbebes(index, k):
    globals()[f'BbPt{k}'] = Dot()
    globals()[f'BbPt{k}'].acc = copy.deepcopy(globals()[f'Pt{index}'].acc)

def resparents():
    for k in range(Pop):
        mutation(globals()[f'BbPt{k}'])
        globals()[f'Pt{k}'] = Dot()
        globals()[f'Pt{k}'].acc = copy.deepcopy(globals()[f'BbPt{k}'].acc)
            
def chgmtGeneration2():
    a=1
    for k in range(len(parentsDots)): 
        index = parentsDots[k]
        lesbebes(index, k)
    resparents()
#------------------

def drawlastgen(speed):
    pygame.init()
    pygame.display.set_caption(f'Dots : Generation {gen}')
    screen = pygame.display.set_mode((window_x, window_y))
    fps = pygame.time.Clock()
    k=0
    while True:
        k +=1
        pygame.display.update()
        fps.tick(speed)
        screen.fill(white)
        pygame.draw.rect(screen, red, pygame.Rect(goal_pos[0]-4, goal_pos[1]-4, 8, 8))
        pygame.draw.rect(screen, blue, pygame.Rect(Obs1[0],Obs1[2], Obs1[1]-Obs1[0], Obs1[3]-Obs1[2]))
        pygame.draw.rect(screen, blue, pygame.Rect(Obs2[0],Obs2[2], Obs2[1]-Obs2[0], Obs2[3]-Obs2[2]))
        pygame.draw.rect(screen, green, pygame.Rect(Pt0.pos[k][0]-3, Pt0.pos[k][1]-3, 6, 6))
        if k-1 == len(Pt0.pos):
            break
    time.sleep(3)
    pygame.quit()


def winnerwinnerchickendiner():
    indexchamp = 0
    for k in range(Pop):
        if globals()[f'Pt{k}'].finesse > globals()[f'Pt{indexchamp}'].finesse :
            indexchamp = k        
    parentsDots.append(indexchamp)

def winnersteps(pt):
    return(pt.reachedGoal.count(False))



#---------------------------------------------------------------------------------------------------------------
#MAIN LOOP


gene = 0
#Generation 1
while True:
#init de la 1e génération
    minstep = 10000000
    step = -1
    sommefinesse = 0 
    IterationAlive = 0
    pygame.display.update()

    screen.fill(white)
    pygame.draw.rect(screen, red, pygame.Rect(goal_pos[0]-4, goal_pos[1]-4, 8, 8))
    pygame.draw.rect(screen, blue, pygame.Rect(Obs1[0],Obs1[2], Obs1[1]-Obs1[0], Obs1[3]-Obs1[2]))
    pygame.draw.rect(screen, blue, pygame.Rect(Obs2[0],Obs2[2], Obs2[1]-Obs2[0], Obs2[3]-Obs2[2]))
        
    #calcul des caractéristiques pour chaque point
    for k in range(Pop):
        
        update(globals()[f'Pt{k}'], step)
        if globals()[f'Pt{k}'].dead[-1] == False and globals()[f'Pt{k}'].reachedGoal[-1] == False:                
            rand(globals()[f'Pt{k}'])
            calcdir(globals()[f'Pt{k}'], step)
            IterationAlive += 1
            calcfinesse(globals()[f'Pt{k}'])
        #Update des Dead
        else:
            randed(globals()[f'Pt{k}'])
        draw(globals()[f'Pt{k}'], step)
        sommefinesse += globals()[f'Pt{k}'].finesse
        globals()[f'Pt{k}'].finesseproportion = sommefinesse

    #nombre de vivants
    alive = IterationAlive
    show_alive(1, black, 'times new roman', 15, 0)
    if MortGeneration() == True :
        break
    
#Générations suivantes 
for gen in range(1, Generations):
    gen += 1
    alive = Pop
    parentsDots = []
    
    #Sélection
    #ajout champion
    winnerwinnerchickendiner()
    if globals()[f'Pt{parentsDots[-1]}'].reachedGoal[-1] == True:
        minstep = winnersteps(globals()[f'Pt{parentsDots[-1]}'])
        
    #ajout parents
    while True:
        rand = random.random() * sommefinesse
        for k in range(Pop):
            #CHOIX TYPE SELEC
            if selecParents3(globals()[f'Pt{k}'], sommefinesse, rand) == True:
                parentsDots.append(k)
        if len(parentsDots) >= Pop:
            break
    delta = len(parentsDots) - Pop
    for k in range(delta):
        parentsDots.pop()
    
    
    print('Génération ', gen)
    #initialisation écran
    chgmtGeneration()
    Pt0 = Dot()
    Pt0.acc = copy.deepcopy(BbPt0.acc)
    Pt0.champ = True
    step = 0
    pygame.init()
    pygame.display.set_caption(f'Dots : Generation {gen}')
    screen = pygame.display.set_mode((window_x, window_y))

    
    while True:
        
        #init de la i-eme génération
        sommefinesse = 0 
        IterationAlive = 0

        pygame.display.update()
        fps.tick(game_speed)
        screen.fill(white)
        pygame.draw.rect(screen, red, pygame.Rect(goal_pos[0]-4, goal_pos[1]-4, 8, 8))
        pygame.draw.rect(screen, blue, pygame.Rect(Obs1[0],Obs1[2], Obs1[1]-Obs1[0], Obs1[3]-Obs1[2]))
        pygame.draw.rect(screen, blue, pygame.Rect(Obs2[0],Obs2[2], Obs2[1]-Obs2[0], Obs2[3]-Obs2[2]))
        for k in range(Pop):
    
            update(globals()[f'Pt{k}'], step)
            if globals()[f'Pt{k}'].dead[-1] == False and globals()[f'Pt{k}'].reachedGoal[-1] == False:  
                calcdir(globals()[f'Pt{k}'], step)
                IterationAlive += 1
                calcfinesse(globals()[f'Pt{k}'])
            else:
                randed(globals()[f'Pt{k}'])
            draw(globals()[f'Pt{k}'],step)
            sommefinesse += globals()[f'Pt{k}'].finesse
            globals()[f'Pt{k}'].finesseproportion = sommefinesse
        
        #nombre de vivants
        alive = IterationAlive
        show_alive(1, black, 'times new roman', 15, gen)
        if MortGeneration() == True :
            break
            
        step += 1
        if step > len(globals()[f'Pt{k}'].pos):
            break
pygame.quit()
    
        


    
