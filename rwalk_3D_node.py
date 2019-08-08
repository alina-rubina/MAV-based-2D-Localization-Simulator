from pylab import *

import math

import random


def random_walk_node(x_y_zinit_qc,max_x,max_y,max_z,step): 
    max_x=abs(max_x); 
    max_y=abs(max_y);
    max_z=abs(max_z);
    random.seed(None); # Seed generator, None = system clock
    steps = 1
    x = zeros(steps+1)
    y = zeros(steps+1)
    z = zeros(steps+1)
    
    ### create a function that accepts the current co-ordinates of QC as x_init and y_init

    x[0] = x_y_zinit_qc[0]
    y[0] = x_y_zinit_qc[1]
    z[0] = x_y_zinit_qc[2]
        
    theta = 2.*math.pi*random.random() # 0 =< angle =< 2 pi
    x[1] = x[0] + step*cos(theta) #-1 =< x =< 1
    
    if (abs(x[1])) > max_x :
        if x[1] < 0 :
            x[1] = -max_x;
        else : 
            x[1] = max_x;
   
    y[1] = y[0] + step*sin(theta) #-1 =< y =< 1
    
    if (abs(y[1])) > max_y :
        if y[1] < 0 :
            y[1] = -max_y;
        else :
            y[1] = max_y;
    
    #z[1] = z[0] + step/10*cos(theta) #-1 =< x =< 1
    z[1] = z [0] # no movement on Z
    
    #if (abs(z[1])) > max_z :
    #    if z[1] < 0 :
    #        z[1] = -max_z;
    #    else : 
    #        z[1] = max_z;
       
    #print "This walk's distance R = ", sum(sqrt(x*x + y*y))
    return [x[1],y[1],z[1]];

'''    
x_init_qc = random.sample([20,-19,35,-41,5],1)
y_init_qc = random.sample([12,29,-45,0,-7],1)
max_x = 50
max_y = 50

coord = []
coord = random_walk(x_init_qc,y_init_qc,max_x,max_y)

   
plot(coord[0], coord[1])
print coord
#title('Random Walk (2-D), %s Steps' % steps)
xlabel('y position')
ylabel('x position')
axis ((-50,50, -50, 50))
grid(True, 'major')
axhline(0, color='black', lw=1)
axvline(0, color='black', lw=1)
show()
'''
#Quelle : http://www.science.oregonstate.edu/~rubin/INSTANCES/WebModules/5_RandomWalk/RandomWalkFiles/Pdfs/RandomWalk.pdf
