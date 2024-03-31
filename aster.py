import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from queue import PriorityQueue 
import time   

grid = np.zeros([10, 10])
fig, axe = plt.subplots()

startPoint = np.array([2, 2])
endPoint = np.array([8, 8])
estPoint = np.array([0, 0])

pq = PriorityQueue()

pq.put((-abs(sum(startPoint- startPoint))+abs(sum(startPoint-endPoint)), startPoint.tolist()))

while not pq.empty():
    
    
    estPoint_raw = pq.get()
    
    print(estPoint_raw)
    
    estPoint = estPoint_raw[1]
    
    grid[estPoint[0]-1][estPoint[1]-1] = grid[estPoint[0]-1][estPoint[1]-1]+1
    
    if estPoint[0] == endPoint[0] and estPoint[1] == endPoint[1]:
        break
    
    estPoint = np.array(estPoint)
    
    tmp = (estPoint + np.array([1, 0]))
    tmp0 = (estPoint + np.array([0, 1]))
    tmp1 = (estPoint + np.array([-1, 0]))
    tmp2 = (estPoint + np.array([0, -1]))
    
    f = +abs(sum(startPoint- tmp))+abs(sum(tmp-endPoint))
    f0 = +abs(sum(startPoint- tmp0))+abs(sum(tmp0-endPoint))
    f1 = +abs(sum(startPoint- tmp1))+abs(sum(tmp1-endPoint))
    f2 = +abs(sum(startPoint- tmp2))+abs(sum(tmp2-endPoint))
    
    print(tmp)
    
    tmp = tmp.tolist()
    tmp0 = tmp0.tolist()
    tmp1 = tmp1.tolist()
    tmp2 = tmp2.tolist()
    
    if estPoint[0] < 10:
        pq.put((f,tmp))
        print(str(f)+str(tmp))
            
    if estPoint[1] < 10:
        pq.put((f0,tmp0))
        print(str(f0)+str(tmp0))
        
    if estPoint[0] > 0:
        pq.put((f1,tmp1))
        print(str(f1)+str(tmp1))
        
    if estPoint[1] > 0:
        pq.put((f2,tmp2))
        print(str(f2)+str(tmp2))
        
    time.sleep(0.5)

print(pq.get())

axe.imshow(grid)
plt.show()

