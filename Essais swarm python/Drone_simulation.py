# -*- coding: utf-8 -*-
"""
Created on Tue Nov 12 23:16:17 2024

@author: miche
"""


import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
import timeit
#import pandas as pd

'''
# Global Variables
'''
NUM = 100. # if no NUM in scope, global NUM is used.
AT_W = 1.0 # attraction weight
RE_W = 0.1 # repulsion weights
OR_W = 0.1 # orientation weight
RA_W = 0.1 # random weight
R_MAX = 1. # max random v
P_MAX = 10. # max starting position per component
V_MAX = 5. # max starting component wise velocity
DELTA = 2.0 # time step for movement
p = np.ones(3) #
v = np.ones(3) #

# move all the fish
def move_1():
    global p, v
    for i in range(NUM):
        p_i = np.vstack([p[i]]*NUM) # duplicate current position NUM times

        r = p - p_i # get current vector to every other fish
        mag = np.stack([np.sum(np.multiply(r, r), axis=1)]*3, axis=1) # add in quadrature for mag squared

        v1 = r * (AT_W) # attraction
        v1 = np.sum(v1, axis=0)/NUM

        with np.errstate(invalid='ignore', divide='ignore'):v2 = (-r * (RE_W))/mag # repulsion
        v2 = np.nan_to_num(v2)
        v2 = np.sum(v2, axis=0)

        v3 = (np.sum(v, axis=0)/NUM)*OR_W # orientation
        v4 = 2*np.random.rand(3) - 1

        dv = v1 + v2 + v3 + v4
        v[i] = v1 + v2 + v3 + v4
        p[i] = p[i] + v[i]*DELTA

        del v1, v2, v3, v4

# This was my final and best implementation.
def move_2():
    global p, v
    beta = np.stack([p]*NUM, axis=0) # 3D array containing all positions stacked vertically
    alpha = np.stack([p]*NUM, axis=1) # 3D array containing all positions stacked horizontally

    # r[a][b] contains the vector from fish a to b
    #r = beta - alpha # every vector from every fish to every other fish.
    r = alpha - beta
    mag = np.stack([np.sum(np.multiply(r[:][:], r[:][:]), axis=2)]*3, axis=2)# the magnitude squared of every vector

    # now make the 4 movement vectors
    v1 = np.sum((AT_W)*r, axis=0)/NUM # attraction

    with(np.errstate(invalid='ignore')):v2a = -r*(RE_W)/mag
    v2a = np.nan_to_num(v2a)
    v2 = np.sum(v2a, axis=0) # repulsion

    v3 = np.vstack([np.sum(v, axis=0)/NUM]*NUM)*OR_W # average v

    v4 = 2*np.random.rand(NUM,3)*RA_W -1 # random

    # get new velocity and position
    dv = v1 + v2 + v3 + v4
    v = dv
    p = p + (v*DELTA) # new position

    return #p, v

def plot():
    global p, v
    fig = plt.figure() # set up the fig
    ax = p3.Axes3D(fig) # and the axes

    # get x, y, z
    x = p[:, 0]
    y = p[:, 1]
    z = p[:, 2]

    xa = np.amin(x)
    xb = np.amax(x)
    ya = np.amin(y)
    yb = np.amax(y)
    za = np.amin(z)
    zb = np.amax(z)

    ax.set_xlim(xa - 5, xb + 5)
    ax.set_ylim(ya - 5, yb + 5)
    ax.set_zlim(za - 5, zb + 5)
    ax.scatter(x, y, z)

    plt.show()

def simulate(n=1000, steps=500, method=2):
    # grab and set the global number to n
    global NUM, p, v
    NUM = n # set NUM

    p = np.random.uniform(low=0.0, high=P_MAX, size=(NUM,3)) # create the position vectors
    v = np.random.uniform(low=0.0, high=V_MAX, size=(NUM,3)) # create the velocity vectors

    # do all of the steps
    if(method==2):
        for i in range(steps):
            move_2()
    elif(method==1):
        for i in range(steps):
            move_1()

    return

# main method
def main():
    simulate(n=50, steps=300, method=2)
    return
if __name__=="__main__":main()