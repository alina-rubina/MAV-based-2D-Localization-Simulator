# -*- coding: utf-8 -*-
"""
Created on Thu Jun 11 14:24:59 2015

@author: AHMAD
"""
import numpy as np

def ss_to_dist(Pr1):
    Pr0=-43;
    W=0;
    #Xr=3*np.random.randn(1,1);
    #"$distance = pow(10,(((-40) - 4.8 - ($ss))/(10*3.32)));"
    return 1 * 10**( (Pr0 + W - Pr1) / (10*1.9));
