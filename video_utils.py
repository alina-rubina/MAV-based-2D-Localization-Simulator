# -*- coding: utf-8 -*-
"""
Created on Sun Apr 30 16:28:49 2017

@author: user
"""


from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import matplotlib.cm as cmx
import matplotlib.colors as colors
import numpy as np
from At_UAV_3D import *
from At_Node_3D import *
from localization import *
from Dist_to_ss import dist_to_ss
import Algorithm_3D  
from ss_to_dist import *
#import ss_to_dist
from pylab import *

from rwalk_3D import random_walk
from operator import itemgetter
from scipy.spatial import distance
import time
import numpy
from video_utils import *

import math
from scipy import interpolate
from scipy import ndimage
import collections
#for logging data to log files
import csv, sys, logging
#for area representation
from matplotlib.patches import Rectangle
#for video recording purposes
import matplotlib.animation as manimation
import itertools
from random import shuffle
import re
import signal
from scipy import constants
import json

def hex_to_RGB(hex):
  ''' "#FFFFFF" -> [255,255,255] '''
  # Pass 16 to the integer function for change of base
  return [int(hex[i:i+2], 16) for i in range(1,6,2)]

def RGB_to_hex(RGB):
  ''' [255,255,255] -> "#FFFFFF" '''
  # Components need to be integers for hex to make sense
  RGB = [int(x) for x in RGB]
  return "#"+"".join(["0{0:x}".format(v) if v < 16 else
            "{0:x}".format(v) for v in RGB])

def color_dict(gradient):
  ''' Takes in a list of RGB sub-lists and returns dictionary of
    colors in RGB and hex form for use in a graphing function
    defined later on '''
  return {"hex":[RGB_to_hex(RGB) for RGB in gradient],
      "r":[RGB[0] for RGB in gradient],
      "g":[RGB[1] for RGB in gradient],
      "b":[RGB[2] for RGB in gradient]}


def linear_gradient(start_hex, finish_hex="#FFFFFF", n=10):
  ''' returns a gradient list of (n) colors between
    two hex colors. start_hex and finish_hex
    should be the full six-digit color string,
    inlcuding the number sign ("#FFFFFF") '''
  # Starting and ending colors in RGB form
  s = hex_to_RGB(start_hex)
  f = hex_to_RGB(finish_hex)
  # Initilize a list of the output colors with the starting color
  RGB_list = [s]
  # Calcuate a color at each evenly spaced value of t from 1 to n
  for t in range(1, n):
    # Interpolate RGB vector for color at the current value of t
    curr_vector = [
      int(s[j] + (float(t)/(n-1))*(f[j]-s[j]))
      for j in range(3)
    ]
    # Add it to our list of output colors
    RGB_list.append(curr_vector)

  return color_dict(RGB_list)


def additional_steps(x1, y1, x2, y2, size):# used for video recording, it adds more points between 2 steps
      x_new = []     
      y_new = []
      
      if x1==x2 and y1==y2: # same coordinates , just return last of them
          y_new.append(y2)
          x_new.append(x2)
          return x_new, y_new
      
      dist = sqrt( (x2 - x1)**2 + (y2 - y1)**2 )
      
      N_points =  int( dist / size)
      
      if N_points==0: # points are very close, can not insert additional point, return obly last one
          y_new.append(y2)
          x_new.append(x2)
          return x_new, y_new
      
      step = float(dist / N_points)
      sin_p = abs(float(y2 - y1))/dist
      cos_p = abs(float(x2 - x1))/dist
             
      if x1 > x2:    # direction of vektor between x1 and x2             
          x_k = -1
      else:
          x_k = 1
      if y1 > y2:      # direction of vektor between y1 and y2          
          y_k = -1
      else:
          y_k = 1

      for i in range(1, N_points): # will calculate coordinates for additionl points and save them in array
          x_new.append(x1+cos_p*step*i*x_k)
          y_new.append(y1+sin_p*step*i*y_k)
      y_new.append(y2) # adding also last point
      x_new.append(x2) # adding also last point
      #print "x_new",x_new
      #print "y_new",y_new
      return x_new,y_new
