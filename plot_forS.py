# -*- coding: utf-8 -*-
"""
Created on Sun Apr  7 23:18:23 2024

@author: jonwo
"""

import numpy as np
import matplotlib.pyplot as plt

maxV = 0.3 #0.3m/s
maxA = 0.4 #1m/s^2
samplingHz = 500 #500Hz
samplingT = 1/samplingHz #1/500s

targetS = 1 #1m


tAcc = maxV/maxA

tStatic = targetS/maxV - tAcc

tTotal = tAcc*2 + tStatic

orderN = int(tTotal/samplingT)
raisingN = int(tAcc/samplingT)
staticN = orderN - 2*raisingN

orderArr = np.zeros([orderN])

vTmp = 0
for i in range(0, orderN):
    orderArr[i] = vTmp
    if i < raisingN:
        vTmp = vTmp + maxA*samplingT
    elif i >= raisingN+staticN:
        vTmp = vTmp - maxA*samplingT

orderX = np.arange(0, tTotal, tTotal/orderN)

plt.plot(orderX, orderArr)

plt.show()