# -*- coding: utf-8 -*-
"""
Created on Thu Jun 11 14:20:27 2015

@author: AHMAD
"""
import numpy as np
from pylab import *
import math
import random
from At_Node_3D import *
from math import * 
from pylab import *
import matplotlib.pyplot as plt
from rwalk_3D import random_walk
from operator import itemgetter
from scipy.spatial import distance

import var_measurements    #Importing the file that will save the nr. of measurements variable.

def dist_to_ss(dist):
       
    if (dist < 1):
      dist = 1;
      
    Pr0   = -43;
    W     = 0;
    alpha = 1.9
    SS    =   10 * alpha * np.log10(dist) ; 
    
    n_snapshots = var_measurements.variable   #Taking the nr. of measurements from the global variable saved to the var_file from the main.
    #n_snapshots = 10;     #Number of times we measure the Signal-Strength.
    
    Pr = 0;              #Helpful variable to calculate the average of the Signal-Strength.
   
    for _ in range(n_snapshots):                #Loop over n_snapshots times to average the SS measurement.
           Xr  = 4 * np.random.randn(1)[0];     #Random_Variable Changes after each measurement. Numpy returns a list so that`s why we need the [0]. 
           #Xr  = 0                             #This is for testing. If Xr = 0 We have no noise. Xr=0 is the same as looping inf times.                         
           Pr1 = int (Pr0 - SS + W - Xr )       #This was as int before. Introduces some small error into measurement.
           #Pr1 = Pr0 - SS - W + Xr             #Maybe we should return the exact value, not the int. I don`t know.
           Pr  = Pr + Pr1                       #Adding all the Measurements in order to find the average.
    Pr = Pr / n_snapshots        

    return Pr;

#Pr=[]
#for i in range(1,100):
#    P=dist_to_ss(i)
#    Pr.append(P)
#dist=range(1,100)
#
#plt.figure(3)
#plot(dist,Pr)
#n, bins, patches = plt.hist(Pr, 30, normed=1, facecolor='green',alpha=0.75)
