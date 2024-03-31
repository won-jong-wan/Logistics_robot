# -*- coding: utf-8 -*-
"""
Created on Mon Apr  1 00:41:24 2024

@author: jonwo
"""
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from queue import PriorityQueue 
import time   

grid = np.zeros([10, 10])
fig, axe = plt.subplots()

ims = []

#grid[1, 2] = -10
# grid[2,2] = -10
# grid[3, 2] = -10
# grid[4, 2] = -10
# grid[5, 2] = -10

grid[7, 8] = -10
grid[8, 7] = -10

startPoint = [1, 1]
endPoint = [8, 8]

pq = PriorityQueue()

g = 0;
h = abs(endPoint[0]-startPoint[0])+abs(endPoint[1]-startPoint[1])

f = g + 2*h

pq.put((f, g, startPoint))

tp = 0

ftmp = 0

estP = startPoint
estP_raw= []
while not pq.empty():
    estP_raw = pq.get()
    print(estP_raw)    
    
    newP = estP_raw[2]
    while abs(newP[0]-estP[0])+abs(newP[1]-estP[1])>1 and estP_raw[0] <= ftmp and pq.qsize()>2:
        estP_raw = pq.get()
        print("wr: "+str(estP_raw))   
        newP = estP_raw[2]
    
    estP = estP_raw[2]
    ftmp = estP_raw[0]
    
    grid[estP[0]][estP[1]] = grid[estP[0]][estP[1]] *10
    
    if estP == endPoint:
        break
    
    if estP[1] >0:
        tmp = [estP[0] , estP[1]-1]
        
        g = estP_raw[1] +1
        h = abs(endPoint[0]- tmp[0]) + abs(endPoint[1] - tmp[1])
        
        f = g + 2*h
        
        if grid[tmp[0]][tmp[1]] ==0:
            print("L:" +str(g)+"+"+str(h)+"="+str(f)+ " tmp:"+str(tmp))
            grid[tmp[0]][tmp[1]] = grid[tmp[0]][tmp[1]] + f
            pq.put((f, g, tmp))
    if estP[0] >0:
        tmp = [estP[0]-1, estP[1]]
        
        g = estP_raw[1] +1
        h = abs(endPoint[0]- tmp[0]) + abs(endPoint[1] - tmp[1])
        
        f = g + 2*h
        
        if grid[tmp[0]][tmp[1]] ==0:
            print("U:" +str(g)+"+"+str(h)+"="+str(f)+ " tmp:"+str(tmp))
            grid[tmp[0]][tmp[1]] = grid[tmp[0]][tmp[1]] + f
            pq.put((f, g, tmp))
    if estP[1] < 9:
        tmp = [estP[0] , estP[1]+1]
        
        g = estP_raw[1] +1
        h = abs(endPoint[0]- tmp[0]) + abs(endPoint[1] - tmp[1])
        
        f = g +2*h
        
        if grid[tmp[0]][tmp[1]] ==0:
            print("R:" +str(g)+"+"+str(h)+"="+str(f)+ " tmp:"+str(tmp))
            grid[tmp[0]][tmp[1]] = grid[tmp[0]][tmp[1]] + f
            pq.put((f,g, tmp))
    if estP[0] < 9:
        tmp = [estP[0]+1, estP[1]]
        
        g = estP_raw[1] +1
        h = abs(endPoint[0]- tmp[0]) + abs(endPoint[1] - tmp[1])
        
        f = g + 2*h
        
        if grid[tmp[0]][tmp[1]] ==0:
            print("D:" +str(g)+"+"+str(h)+"="+str(f)+ " tmp:"+str(tmp))
            grid[tmp[0]][tmp[1]] = grid[tmp[0]][tmp[1]] + f
            pq.put((f, g, tmp))
    im = axe.imshow(grid, animated=True)
    if tp == 0:
        axe.imshow(grid)
    ims.append([im])
    tp = tp+1
    
while not pq.empty():
    print(pq.get())
    im = axe.imshow(grid, animated=True)
    ims.append([im])



ani = animation.ArtistAnimation(fig, ims, interval=200)
ani.save('aster_h_200.gif', writer='imagemagick')

plt.show()