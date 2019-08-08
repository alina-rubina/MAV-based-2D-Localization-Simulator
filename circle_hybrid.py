import math
import itertools
import numpy as np 

def circle_hybrid(coord,radius): #(1.co-ordinates and 2.radius of circle depending on communication range)
    Result=[]    
    th=np.arange(0,2*math.pi,2*math.pi/7); #by this number 26 you make it smooth, you can change it
    r=radius; #these r,r1 are the corresponding radiuses in x and y direction
    r1=radius;
    x=r*np.cos(th)+coord[0];
    y=r1*np.sin(th)+coord[1];
    
    for i in range(0, len(x)):
        A=[x[i],y[i],0]
        Result.append(A)
    x = iter(Result) #returns every element at a time
    
    return x 

#w=circle_hybrid([0,0,0],90)
#print w.next()
#print w.next()
#print w.next()
#print w.next()
#print w.next()
#print w.next()
#print w.next()