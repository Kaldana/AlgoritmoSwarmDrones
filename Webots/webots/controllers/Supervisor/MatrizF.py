# Librerías importadas
from controller import Robot, Motor, Supervisor, Node
import numpy as np
import math

  
def MatrizF(f):
#FMATRIX Retorna matriz de formación deseada
#   f = número de formación

    acc = 2;
    d2c = math.sqrt((acc**2 + acc**2))
    d3c = math.sqrt((d2c**2 + acc**2))
    ac = 3;
    d2t = math.sqrt(ac**2 - (ac/2)**2)
    dpbc = math.sqrt(d2t**2 + ac**2)
    dbb = math.sqrt((ac/2)**2 + ac**2)
    dbs = math.sqrt(ac**2 + ac**2)

	## MATRICES
	# matrices de adyacencia grafo mínimamente rígido
    cubo = np.array([[0,   acc,  acc,  d2c, acc,  d2c, d2c, d3c],
                     [acc,  0,   d2c, acc,  d2c, d3c, acc,  d2c],
                     [acc,  d2c, 0,   acc,  d2c, acc,  d3c, d2c],
                     [d2c, acc,  acc,  0,   d3c, d2c, d2c, acc],
                     [acc,  d2c, d2c, d3c, 0,   acc,  acc,  d2c],
                     [d2c, d3c, acc,  d2c, acc,  0,   d2c, acc],
                     [d2c, acc,  d3c, d2c, acc,  d2c, 0,   acc],
                     [d3c, d2c, d2c, acc,  d2c, acc,  acc,  0]]);
   
    pristri = np.array([[0,    ac,   ac,   d2t,  ac,   dbs,   dbs,  dpbc],
                        [ac,   0,    ac,   ac/2, dbs,  ac,    dbs,  dbb],
                        [ac,   ac,   0,    ac/2, dbs,  dbs,   ac,   dbb],
                        [d2t,  ac/2, ac/2, 0,    dpbc, dbb,   dbb,  ac],
                        [ac,   dbs,  dbs,  dpbc, 0,    ac,    ac,   d2t],
                        [dbs,  ac,   dbs,  dbb,  ac,   0,     ac,   ac/2],
                        [dbs,  dbs,  ac,   dbb,  ac,   ac,    0,    ac/2],
                        [dpbc, dbb,  dbb,  ac,   d2t,  ac/2,  ac/2, 0]]);
	
	
    MM = [cubo, pristri]
	
    return MM[f]


