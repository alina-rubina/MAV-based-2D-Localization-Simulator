# -*- coding: utf-8 -*-
"""
Created on Mon May 01 14:50:01 2017

@author: user
"""
from utils import *
from math import *
from ss_to_dist import ss_to_dist
from filters import *
from dsmisc import *
from multilat import *
from At_Node_3D import *
import numpy as np                                        
from numpy import sqrt, dot, cross                       
from numpy.linalg import norm    
import time
import re
import scipy

circles_factor = 1 # for very erroneous data tuning factor between 0.8 and 1.2 can be selected 

def get_two_points_distance(p1, p2):
    return math.sqrt(pow((p1.x - p2.x), 2) + pow((p1.y - p2.y), 2))

def get_two_circles_intersecting_points(c1, c2):
    p1 = c1.center 
    p2 = c2.center
    r1 = c1.radius
    r2 = c2.radius

    d = get_two_points_distance(p1, p2)
    # if to far away, or self contained - can't be done
    if d >= (r1 + r2) or d <= math.fabs(r1 -r2):
        return None

    a = (pow(r1, 2) - pow(r2, 2) + pow(d, 2)) / (2*d)
    h  = math.sqrt(pow(r1, 2) - pow(a, 2))
    x0 = p1.x + a*(p2.x - p1.x)/d 
    y0 = p1.y + a*(p2.y - p1.y)/d
    rx = -(p2.y - p1.y) * (h/d)
    ry = -(p2.x - p1.x) * (h / d)
    #print "res ",str(point(x0+rx, y0-ry))
    #print "res ",str(point(x0-rx, y0+ry))
    return [point(x0+rx, y0-ry), point(x0-rx, y0+ry)]

def get_all_intersecting_points(circles):
    points = []
    num = len(circles)
    for i in range(num):
        j = i + 1
        for k in range(j, num):
            res = get_two_circles_intersecting_points(circles[i], circles[k])
            if res:
                points.extend(res)
    return points

def is_contained_in_circles(point, circles):
    for i in range(len(circles)):
        if (get_two_points_distance(point, circles[i].center) > (circles[i].radius)):
            return False
    return True

def get_polygon_center(points):
    center = point(0, 0)
    num = len(points)
    if num == 0:
        return -1
    for i in range(num):
        center.x += points[i].x
        center.y += points[i].y
    center.x /= num
    center.y /= num
    return center

def print_point(point):
    if point == -1:
        return "no valid coordinates"
    string =  "["+ repr(point.x) +"," + repr(point.y) + "]"
    return string
    
def print_circle(circle):
    string = "[" + print_point(circle.center) + "/" + repr(circle.radius) + "]"
    return string

def init_distanceArray(x , y): 
    fin_arr = []
    tmp_arr = []
    for i in range (0, x):
        tmp_arr = []
        tmp_arr = [-1] * x
        fin_arr.append(tmp_arr)
    return fin_arr
    
def collinear(A,B,C):
    AB=B-A;
    AC=C-A;
    s=np.cross(AB,AC)
    if s[0]==0 and s[1]==0 and s[2]==0:
        return True;
    else:
        return False;
        
class base_station(object):
    def __init__(self, lat, lon, dist):
        self.lat = lat
        self.lon = lon
        self.dist = dist

class point(object):
    def __init__(self, x, y):
        self.x = x
        self.y = y
        
    def __str__(self):
        return "x "+str(self.x)+" y "+str(self.y)


class circle(object):
    def __init__(self, point, radius):
        self.center = point
        self.radius = radius
    
    def __str__(self):
        return "x "+str(self.center)+" y "+str(self.radius)
    
def serialize_instance(obj):
    d = {}
    d.update(vars(obj))
    return d

#def collinear(p0, p1, p2):
#    p1, p2, p3 = Point3D(p0[0], p0[1], p0[2]), Point3D(p1[0], p1[1], p1[2]), Point3D(p2[0], p2[1], p2[2])
#    x1, y1 = p1[0] - p0[0], p1[1] - p0[1]
#    x2, y2 = p2[0] - p0[0], p2[1] - p0[1]
#    #return x1 * y2 - x2 * y1 < 1e-12
#    #return p0[0] * (p1[1] - p2[1]) + p1[0] * (p2[1] - p0[1]) + p2[0] * (p0[1] - p1[1]) == 0
#    return Point3D.are_collinear(p1, p2, p3)