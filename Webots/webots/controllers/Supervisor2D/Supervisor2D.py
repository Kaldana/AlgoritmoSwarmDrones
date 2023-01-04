"""Supervisor controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Supervisor
import numpy as np
import random
import math
import pickle

"Aca va la funcion de matrices"
from MatrizF import *

TIME_STEP = 64
# create the Robot instance.
supervisor = Supervisor()

# get the time step of the current world.

# DATOS DE PLATAFORMA
Plataforma = supervisor.getFromDef("Plataforma")
Tamano = Plataforma.getField("size")
TamanoVec = Tamano.getSFVec3f()

# DATOS DE AGENTES
N = 4
r = 1.3
R = 1
MAX_SPEED = 1.7
agente0 = supervisor.getFromDef("Agente0")
agente1 = supervisor.getFromDef("Agente1")
agente2 = supervisor.getFromDef("Agente2")
agente3 = supervisor.getFromDef("Agente3")

Agentes = [agente0,agente1,agente2,agente3]

# POSICIONES DE AGENTES

pos0 = agente0.getField("translation")
pos1 = agente1.getField("translation")
pos2 = agente2.getField("translation")
pos3 = agente3.getField("translation")

posAgentes = [pos0,pos1,pos2,pos3]
X = np.empty ([3,N])

# POSICIONES ALEATORIAS PARA DRONES

for a in range(0,N):
    sizeR0 = TamanoVec[0] 
    sizeR1 = TamanoVec[1] 
    sizeR2 = TamanoVec[2] 
    X[0,a] = random.uniform(-1, 1)
    X[1,a] = random.uniform(-1, 1)
    X[2,a] = random.uniform(-1, 1)
#print("X",X)

# REVISAR POSICIONES

cW1 = 3

while(cW1 > 1 or cW2 > 1):
    cW1 = 0
    cW2 = 0
    # Asegurar que los agentes no empiecen uno sobre otro
    contR = 1				# contador de intersecciones
    while(contR > 0):
        contR = 0
        for i in range(1, N):
            for j in range(1, N-i):
                resta = math.sqrt((X[0,i]-X[0,i+j])**2+(X[1,i]-X[1,i+j])**2+(X[2,i]-X[2,i+j])**2)	# diferencia entre las posiciones
                if(abs(resta) < r):
                    X[0,i+j] = X[0,i+j] + 0.1									# cambio de posición
                    X[1,i+j] = X[1,i+j] + 0.1
                    X[2,i+j] = X[2,i+j] + 0.1 # hay intersección
                    contR = contR+1
        cW1 = cW1+1

Xi = X

# ASIGNACION DE POSICIONES REVISADAS
for b in range(0, N):
    posAgentes[b].setSFVec3f([X[0,b], X[1,b],0.405+b*0.01])
    posAgentes[0].setSFVec3f([0,0,0.405+b*0.01])
# POSICIONES ACTUALES
posActuales = np.zeros([3,N])
posNuevas = np.zeros([3,N])

# MATRIZ DE VELOCIDADES
V = np.empty([3,N])

# MATRIZ DE FORMACION
d = MatrizF(2)
#print(d)

# Main loop:
cambio = 0						# variable para cambio de control 
while supervisor.step(TIME_STEP) != -1:
    #print("cambio",cambio)
	
	# Se obtienen posiciones actuales
    for c in range(0,N):
        posC = Agentes[c].getField("translation")
        posActuales[0][c] = posC.getSFVec3f()[0]
        posActuales[1][c] = posC.getSFVec3f()[1]
        posActuales[2][c] = posC.getSFVec3f()[2]
    
    for g in range(0,N):
        E0 = 0
        E1 = 0
        E2 = 0
        for h in range(0,N):
            dist = np.asarray([posActuales[0][g]-posActuales[0][h], posActuales[1][g]-posActuales[1][h],posActuales[2][g]-posActuales[2][h]])	# vector xi - xj   
            mdist = math.sqrt(dist[0]**2 + dist[1]**2 + dist[2]**2)														# norma euclidiana vector xi - xj
            dij = 0.2*d[g][h]																				# distancia deseada entre agentes i y j

            # Peso añadido a la ecuación de consenso

            if(mdist == 0):
                w = 0
            else:
                if(cambio == 0): 										# inicio: acercar a los agentes sin chocar
                    #print("collision avoidance")
                    w = (mdist - (2*(r+0.05)))/(mdist - (r+0.05))**2 	# collision avoidance
                else:
                # collision avoidance & formation control
                    #print("formacion")
                    w = (4*(mdist - dij)*(mdist - r) - 2*(mdist - dij)**2)/(mdist*(mdist - r)**2)
                
            # Tensión de aristas entre agentes 
            E0 = E0 + w*dist[0]
            E1 = E1 + w*dist[1]
            E2 = E2 + w*dist[2]
            
        # Actualización de velocidad
        
        V[0][g] = 2*(E0)*TIME_STEP/1000 
        V[1][g] = 2*(E1)*TIME_STEP/1000
        V[2][g] = 2*(E2)*TIME_STEP/1000
        
	# Al llegar muy cerca de la posición deseada realizar cambio de control

    normV2 = 0
    for m in range(0,N):
        nV2 = V[0][m]**2 + V[1][m]**2 + V[2][m]**2
        normV2 = normV2 + nV2
    
    normV = math.sqrt(normV2)
    #print(normV)
    V = V/normV * MAX_SPEED
    
    if(normV < 0.5):
        cambio = cambio + 1
    # Guardar datos necesarios para asignar velocidad a cada agente  
    with open('D:/Kenneth/Documents/UVG/S9/DisenoeInnovacion/Tesis/Webots/webots/controllers/Datos1.pickle','wb') as f:
        pickle.dump(posActuales, f)
    with open('D:/Kenneth/Documents/UVG/S9/DisenoeInnovacion/Tesis/Webots/webots/controllers/Datos2.pickle','wb') as f:
        pickle.dump(posNuevas, f)
    with open('D:/Kenneth/Documents/UVG/S9/DisenoeInnovacion/Tesis/Webots/webots/controllers/Datos3.pickle','wb') as f:
        pickle.dump(V, f)
    pass
