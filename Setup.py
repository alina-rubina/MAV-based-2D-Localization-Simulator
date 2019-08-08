# -*- coding: utf-8 -*-
"""
Created on Thu Jun 11 13:55:50 2015

@author: AHMAD
"""

from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
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
import matplotlib.pyplot as plt
from rwalk_3D import random_walk
from operator import itemgetter
from scipy.spatial import distance



final_conclusions_length_of_loc_nodes = []
final_conclusions_Average_loc_error = []
final_conclusions_total_route_length = []
conclusions = []

def main():
    #initialization:
    #np.random.seed(None) # Seed generator, None = system clock
    #uav_current_coord = [0, 0, 0]

    uavs = []    
    nodes = []
    n_uavs = 12
    n_nodes = 200
    #uavs = [mUav(0,50,100,0,0,0) for count in range(0,n_uavs)]
    #for uav in range(len(uavs)):
    #uavs[uav].uavId=uav;
    max_x = 250
    max_y = 100
    max_z = 5
    threshold = -100
    step = 50; # step size of one UAV movement
    steps = 8; # number of UAV movements
    #max_distance = ss_to_dist.ss_to_dist(threshold);    
    max_distance = ss_to_dist(threshold);

    
    #localised_nodes = []
    track_uavs = [];
    track_uav = [];
    #track_uavs.append(uavs[uav].uav_current_coord);
    #track_uav.append(uav_current_coord);
    track_nodes = []
    average_nr_neigh = 0;
    #node_list = [100]    
    line_colors = ['b','g','r','c','m','y','k','w'];
    
    The_Result = Algorithm_3D.main(uavs, nodes, n_uavs, n_nodes, max_x, max_y, max_z, threshold, step, steps, max_distance, track_uavs, track_uav, track_nodes, average_nr_neigh);



    uavs = The_Result[0];
    nodes = The_Result[1];
    track_uavs = The_Result[2];
    track_uav = The_Result[3];
    track_nodes = The_Result[4];
    average_nr_neigh = The_Result[5];
    max_distance = The_Result[6];



        #conclusion: plot results   
    #print ("---------Setup-----------")
    #print ("Nodes: 200", '; area: ', max_x*2, 'x', max_y*2, 'x', max_z*2);//ec
    #print ('Threshold:',threshold, '; Tx radius:', ss_to_dist(threshold));//ec
    #print ('Sim steps: ', steps, '; step size: ', step);//ec
    #print ('Average # of neighbors: ', average_nr_neigh/n_nodes);//ec
    #print ("---------Conclusions-----------")//ec
    
    #for i in range(len(uavs))://ec
    #for i in range(1):
        #for j in range(len(track_uav[i]))://ec
            #print ('track of uav ',i,' step ',j, ' coordinates',track_uav[i][j]);//ec
    
    localised_nodes = [];
    loc_error = 0;
    total_route_Length = 0;
    #route_length = 0;
    for i in range(len(nodes)):
        if(nodes[i].mystatus == 1):
            localised_nodes.append(nodes[i].x_y_zestim);
            loc_error += distance.euclidean(nodes[i].x_y_zestim, nodes[i].x_y_zreal);
    #print ('Number of loc nodes = ', len(localised_nodes));//ec
#    for uav in range(1):    
    #fig = plt.figure()//ec
    #ax = fig.add_subplot(111, projection='3d');//ec   
    #for uav in range(len(uavs)): //ec 
        #plot(np.transpose(track_uav[uav])[0], np.transpose(track_uav[uav])[1], np.transpose(track_uav[uav])[2],color=line_colors[uav%len(line_colors)]);//ec
        #ax.plot_wireframe(np.transpose(track_uav)[0], np.transpose(track_uav)[1], np.transpose(track_uav)[2])
    #ax.scatter(np.transpose(track_nodes)[0], np.transpose(track_nodes)[1], np.transpose(track_nodes)[2], c='r')//ec
    #plot(np.transpose(track_uav[0])[0], np.transpose(track_uav[0])[1],'b-');    
    if(len(localised_nodes)>0):
        #plot(np.transpose(localised_nodes)[0], np.transpose(localised_nodes)[1], 'g^', fillstyle='none');
        #ax.scatter(np.transpose(localised_nodes)[0], np.transpose(localised_nodes)[1], np.transpose(localised_nodes)[2], 'g');//ec
        Average_loc_error = loc_error/len(localised_nodes);
        #print ('Average loc error = ', Average_loc_error);//ec
    else :
        Average_loc_error = "undefined"
        #print ('Average loc error = ', Average_loc_error);//ec
    #for s in range (0, steps):
        #for uav in range(len(uavs)):
    for i in range(len(track_uav)):
           if ((i+1)<len(track_uav)):
            uavs[uav].route_length += distance.euclidean(track_uav[i][0], track_uav[i+1][0]);
            uavs[uav].route_length += distance.euclidean(track_uav[i][1], track_uav[i+1][1]);
            uavs[uav].route_length += distance.euclidean(track_uav[i][2], track_uav[i+1][2]);
            i = i+1;
            #print ('Full route length for accumulated UAVs = ',uavs[uav].route_length);//ec
    total_route_Length += uavs[uav].route_length;
    #print 'Full route length for each UAV = ',uavs[uav].route_length;
    
    #print track_uav;
    #print np.transpose(track_uav);
    #title('Random Walk (2-D), %s Steps' % steps)
    #xlabel('y position')
    #ylabel('x position')
    #zlabel('z position')
    #axis ((-200, 200, -200, 200))
    #ax.set_xlabel('X Label')//ec
    #ax.set_ylabel('Y Label')//ec
    #ax.set_zlabel('Z Label')//ec
    #grid(True, 'major')//ec
    #axhline(0, color='black', lw=1)//ec
    #axvline(0, color='black', lw=1)//ec
    #axzline(0, color='black', lw=1)
    #print ('The Average Route length for each UAV = ',total_route_Length/len(uavs));//ec
    #print ('The Total Route Length for all UAVs = ',total_route_Length)//
    #show()//ec
    
    return [len(localised_nodes),Average_loc_error, total_route_Length]
 
      
if __name__ == '__main__':
  for i in range(0,0):
        #print ("Start of new simulation", i)//ec
        conclusions = main()
        final_conclusions_length_of_loc_nodes.append(conclusions[0])
        final_conclusions_Average_loc_error.append(conclusions[1])
        final_conclusions_total_route_length.append(conclusions[2])  
    

                        
