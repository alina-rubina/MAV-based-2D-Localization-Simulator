# -*- coding: utf-8 -*-
"""
Created on Thu Jun 11 14:06:04 2015

@author: AHMAD
"""
line_colors = ['b', 'g', 'r', 'c', 'm', 'y', 'k', 'w'];
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import matplotlib.pyplot as plt
import matplotlib.cm as cmx
import matplotlib.colors as colors
import numpy as np
from g_shaped import *
from At_UAV_3D import *
from At_Node_3D import *
from localization import *
from Dist_to_ss import dist_to_ss
from ss_to_dist import *
from pylab import *
import matplotlib.pyplot as plt
from Setup import *
from square_location import square_location
from next_location_and_updated_database import next_location_and_updated_database
from rwalk_3D_hybrid import random_walk
from rwalk_3D_hybrid import circle_walk
from rwalk_3D_node import random_walk_node
from operator import itemgetter
from scipy.spatial import distance

import time
from triangle import triangle                 #Trajectory 1
from Doublescan import DoubleScan             #Trajectory 2
from circle_hybrid import circle_hybrid

import math

final_conclusions_length_of_loc_nodes = []
final_conclusions_Average_loc_error = []
final_conclusions_total_route_length = []
conclusions = []
nodes_real_for_each_step = []
nodes_estim_for_each_step = []

   

# Setup.main(uavs, nodes, n_uavs, n_nodes, max_x, max_y, max_z, step, steps, max_distance, track_uavs, track_uav, track_nodes, average_nr_neigh, line_colors);
def main(uavs, nodes, n_uavs, n_nodes, max_x, max_y, max_z, threshold, step, steps, max_distance, track_uavs, track_uav,
         track_nodes, average_nr_neigh, localised_unlocalised_ngr, step_for_node, build = 0, localisation_method = 0, localisation_filtering = 0, copter_trajectory = 0):

         
#start here 
    global max_range
    max_range = ss_to_dist(threshold)
    square_mid_points=square_location(max_x, max_y, max_z , max_range)
    print max_range
    global shai #Makes sure that the given figure(circle or G-shaped in our case) is formed around a node with maximun unlocalised nodes
    shai=0 
    global temp #temporary list to pass co-ordinate valves to UAV
    temp=[]
#stop here
    
    uavs = [mUav(0, step, steps, 0, 0, 0) for count in range(0, n_uavs)]
    for uav in range(len(uavs)):
        track_uav.append([])
        uavs[uav].uavId = uav;

        uavs[uav].uav_current_coord = [-max_x, -max_y, max_z];
        track_uavs.append(uavs[uav].uav_current_coord);
    nodes = [mNode(0, 0, 0, 0) for count1 in range(0, n_nodes)] 

    #--------------------------------------------modif according to original map
    for node in range(len(nodes)):
        nodes[node].myId = node;
           #Uniform distirbution
        #nodes[node].x_y_zreal=[np.random.uniform(-1*max_x,max_x), np.random.uniform(-1*max_y,max_y),0];
        #nodes[node].x_y_real=[np.random.uniform(-500,500), np.random.uniform(-500,500)];
        #mixed distribution
#        if (node <= int(0.25 * n_nodes)):
#            nodes[node].x_y_zreal = [np.random.uniform(-0.4 * max_x, -0.8 * max_x),
#                                     np.random.uniform(-0.8*max_y, -0.4 * max_y),
#                    np.random.uniform(0,0)];  # idmt
#        #track_nodes.append(nodes[node].x_y_zreal);
#        #for i in range(len(nodes)/4):
#          #  for j in range(i + 1, len(nodes)/4):
#                ##dist1 = distance.euclidean(nodes[i].x_y_zreal, nodes[j].x_y_zreal);
#                #dist_inter_1=np.append(dist_inter_1,dist1)
#        if (node > int(0.25 * n_nodes) and node <= int(0.5 * n_nodes)):
#            nodes[node].x_y_zreal = [np.random.uniform(-0.4 * max_x, -0.8 * max_x), np.random.uniform(0.4*max_y, 0.8 * max_y),
#                                     np.random.uniform(0, 0)];  # usz
#        if (node > int(0.5 * n_nodes) and node <= int(0.75 * n_nodes)):
#            nodes[node].x_y_zreal = [np.random.uniform(0 * max_x, 1* max_x),
#                                     np.random.uniform(-1*max_y, 0 * max_y),
#                                     #np.random.uniform(-0.0 * max_z, 0.8 * max_z)];  # haus H
#                                     np.random.uniform(0, 0)];  # haus H
##        track_nodes.append(nodes[node].x_y_zreal);
##        for i in range(len(nodes)/4):
##            for j in range(i + 1, len(nodes)/4):
##                dist2 = distance.euclidean(nodes[i].x_y_zreal, nodes[j].x_y_zreal);
##                dist_inter_2=np.append(dist_inter_2,dist2)
#        if (node > int(0.75 * n_nodes) and node <= int( n_nodes)):
#            nodes[node].x_y_zreal = [np.random.uniform(0 * max_x, 1 * max_x),
#                                     np.random.uniform(0*max_y, 1 * max_y),
#                                     #np.random.uniform(0.28264 * max_z, 0.41864 * max_z)];  # haus I
#                                     np.random.uniform(0, 0)];  # haus I
###        track_nodes.append(nodes[node].x_y_zreal);
#        for i in range(len(nodes)/4):
#            for j in range(i + 1, len(nodes)/4):
#                dist3 = distance.euclidean(nodes[i].x_y_zreal, nodes[j].x_y_zreal);
#                dist_inter_3=np.append(dist_inter_3,dist3)
                 #nodes[node].x_y_zreal=[np.random.random_integers(-1*150,150), np.random.uniform(-1*150,150),0];
        #print 'nodes real coordinates', nodes[node].x_y_real
        #clusters
#        
        if (node <= int(0.25 * n_nodes)):
            nodes[node].x_y_zreal = [np.random.uniform(-0.2 * max_x, -0.7 * max_x),
                                     np.random.uniform(-0.7*max_y, -0.2 * max_y),
                                     np.random.uniform(0, 0.6666 * max_z)];  # idmt
        if (node > int(0.25 * n_nodes) and node <= int(0.5 * n_nodes)):
            nodes[node].x_y_zreal = [np.random.uniform(-0.2 * max_x, -0.7 * max_x), np.random.uniform(0.2*max_y, 0.7 * max_y),
                                     np.random.uniform(0.4 * max_z, 0.7 * max_z)];  # usz
        if (node > int(0.5 * n_nodes) and node <= int(0.75 * n_nodes)):
            nodes[node].x_y_zreal = [np.random.uniform(0.2 * max_x, 0.7* max_x),
                                     np.random.uniform(-0.7*max_y, -0.2 * max_y),
                                     #np.random.uniform(-0.0 * max_z, 0.8 * max_z)];  # haus H
                                     np.random.uniform(-0.3 * max_z, -0.8 * max_z)];  # haus H
        if (node > int(0.75 * n_nodes) and node <= int( n_nodes)):
            nodes[node].x_y_zreal = [np.random.uniform(0.2 * max_x, 0.7 * max_x),
                                     np.random.uniform(0.2*max_y, 0.7 * max_y),
                                     #np.random.uniform(0.28264 * max_z, 0.41864 * max_z)];  # haus I
                                     np.random.uniform(-0.28264 * max_z, -0.41864 * max_z)];  # haus I
#        if (node > int(0.45 * n_nodes) and node <= int(0.68 * n_nodes)):
#            nodes[node].x_y_zreal = [np.random.uniform(-0.37 * max_x, -0.79 * max_x),
#                                     np.random.uniform(0.12 * max_y, 0.62 * max_y),
#                                     np.random.uniform(0.8 * max_z, 1 * max_z)];  # EAZ
#        if (node > int(0.68 * n_nodes) and node <= int(n_nodes)):
#            nodes[node].x_y_zreal = [np.random.uniform(0.17 * max_x, 0.42 * max_x),
#                                     np.random.uniform(-0.15 * max_y, -0.483 * max_y),
        track_nodes.append(nodes[node].x_y_zreal);

    #create_list_of_poten_neighb()
    for i in range(len(nodes)):
        nodes[i].NULoN = 0;  # clean the local number of unlocalized neighbors
    if (step_for_node == 0):
        #    creating list of potential neighboars
        for i in range(len(nodes)):
            for j in range(i + 1, len(nodes)):
                dist = distance.euclidean(nodes[i].x_y_zreal, nodes[j].x_y_zreal);
                if dist < max_distance * 1.5:  # we take 50% more than max possibe distance for current treshould
                    ss = dist_to_ss(dist);
                    if (ss > threshold):  #if within range , receive packets. Threshold = -100dBm
                        if (nodes[i].mystatus == 0):
                            nodes[j].NULoN += 1;
                        if (nodes[j].mystatus == 0):
                            nodes[i].NULoN += 1;
                        nodes[i].visible_neigbords.append(j);
                        nodes[j].visible_neigbords.append(i);

    #simulation for N steps for each UAV
    finished_flag = 0;  # will become TRUE when all nodes are detected
    j = 0;
    for s in range(0, steps):  #main simulation loop
        #print" step ",s," started-----------------------------------------------"   #trace           
        for uav in range(len(uavs)):
            #print" step ",s," uav ",uav #trace
            uavs[uav].unlocalized_nodes = []
            if (s == 1):
                for node in range(len(nodes)):
                    average_nr_neigh += nodes[node].NULoN;
            #step 1: receive at UAV                             
            for node in range(len(nodes)):
                dist = distance.euclidean(uavs[uav].uav_current_coord, nodes[node].x_y_zreal);
                ss = dist_to_ss(dist);
                #print 'Dist=', dist, '; ss=',ss, 'uav=', uav, '; node=',node; #trace
                if (ss > threshold):  #if within range , receive packets. Threshold = -100dBm
                    RX_packet = nodes[node].packet_transmit();  #receive the packet transmitted by the node
                    #print 'UAV:Received packet', RX_packet; #trace
                    nodes[node].update_database(RX_packet, ss, uavs[
                        uav].uav_current_coord);  # store the packet and ss, uav coordinates in the object nodes directly
                    #uavs[uav].update_database(RX_packet,ss,uavs[uav].uav_current_coord); #store packet also inside UAV,            
                    nodes[node].NULoN_UAV_view = RX_packet[3];  # how many unlocalized nodes around node
                    uavs[uav].save_node_NULON(node, RX_packet[
                        3]);  #add node number and how many unlocalized nodes around it to UAV DB

                    #step 2: Perform localisation and Update of localised_nodes, since new data arrieved
                    #localisation_method, localisation_filtering are coming from simulation via main parameters
                    res = localization(node, nodes, localisation_method, localisation_filtering);
                    if (res != 0):
                        #print "res", res;
                        #Localization was successful: update the local view of the UAV and inform the node

                        #print" localized node ", node , " neighb here", nodes[node].my_neigbords, " known ", nodes[node].mystatus
                        nodes[node].mystatus = 1;
                        # uavs[uav].packet_transmit(RX_packet); #not sure what is this
                        nodes[node].x_y_zestim = res;         
             
            #step 3: UAV moves to a new position. Decision is based on his local view on the network. The UAV moves towards the node with the bigger number of unloc neighbours or does the random walk if the number of neighbors is 0
            localised_nodes = [];
            loc_nodes = 0;
            
            for i in range(len(nodes)):
                if (nodes[i].mystatus == 1):
                    loc_nodes += 1;
            if (loc_nodes == n_nodes): # all nodes are localized
                finished_flag = 1;
                break;

            for i in range(len(uavs[uav].unlocalized_nodes)):
                if ((nodes[uavs[uav].unlocalized_nodes[i][0]].mystatus == 1) & (uavs[uav].unlocalized_nodes[i][
                                                                                    1] > 0)):  # UAV received packet from localized node and node has unlocalized memebers
                    localised_nodes.append([uavs[uav].unlocalized_nodes[i][0], uavs[uav].unlocalized_nodes[i][1]]);
            localised_nodes.sort(key=itemgetter(1));  #sorting according to localised nodes with highest NULoN(Number of Unlocalised Neighbours)
            localised_nodes.reverse();
            #print "localized nodes here", localised_nodes# trace
           
            #localised to unlocalised neighbors
            localised_unlocalised_ngr.append(['Number of loc nodes with unloc neighbors =', len(localised_nodes)]);
            if (len(localised_nodes) > 0):
                #print "that is what first node know "," node number ", nodes[localised_nodes[0][0]].myId , " estim",  nodes[localised_nodes[0][0]].x_y_zestim, " real ",  nodes[localised_nodes[0][0]].x_y_zreal
                i = 0
                j += 1
                #i  #f (np.array_equal(uavs[uav].uav_current_coord, nodes[localised_nodes[i][0]].x_y_zestim)): # to many small movement
               
               #In the Hybrid trajectory approach
               #Here we have to make sure that the entire pre-defined figure is generated before it resumes with the normal flow of program
               #for this we use a global variable shai 
                                                           
                if (distance.euclidean(uavs[uav].uav_current_coord, nodes[localised_nodes[i][0]].x_y_zestim) < step/18 ): 
                    #### main distance between uav and node,for it to form static structure 
                
                    if i + 1 < len(localised_nodes): ##main1 - we have some localized nodes, we go for preapring shape and run around
                         #update database square_mid_points
                         uavs[uav].uav_current_coord,square_mid_points = next_location_and_updated_database([uavs[uav].uav_current_coord],square_mid_points)                     
                         #prepare all next steps
                         temp= circle_hybrid(uavs[uav].uav_current_coord, step);      
                         #uav is moving to next position                
                         uavs[uav].uav_current_coord=temp.next(); 
                         #counting how many steps uav made using developed trajectory
                         shai=shai+1;                     

                    else:##main2 - no nodes were localized during this step (this hould never work)
                        
                        if(shai>0):## copter still following shape prepared early
                            uavs[uav].uav_current_coord = temp.next();                        
                            shai=shai+1;
                        
                        if(shai==0):# Normal flow of program and there is no shape to follow
                            uavs[uav].uav_current_coord,square_mid_points = next_location_and_updated_database([uavs[uav].uav_current_coord],square_mid_points)
                            uavs[uav].uav_current_coord = random_walk(uavs[uav].uav_current_coord, max_x, max_y, max_z, step);
             
                else:##main3 UAV is very close to nodes estimated coordinates

                    if(shai>0):##  copter still following shape prepared early                       
                        uavs[uav].uav_current_coord=temp.next()
                        shai=shai+1
                    
                    if(shai==0):## Normal flow of program and there is no shape to follow
                        #uavs[uav].uav_current_coord,square_mid_points = next_location_and_updated_database([uavs[uav].uav_current_coord],square_mid_points)
                        uavs[uav].uav_current_coord = nodes[localised_nodes[i][0]].x_y_zestim;                                     
            
            else:#main4 -  no nodes were localized during this step 
                #print "we don't have any visible nodes with known unlocalized neighboard, do random walk "
              # we don't have any visible nodes with known unlocalized neighboard, do random walk
                if(shai>0):## copter still following shape prepared early       
                    uavs[uav].uav_current_coord=temp.next()
                    shai=shai+1
                
                if(shai==0):## Normal flow of program
                    #print len(square_mid_points)
                    uavs[uav].uav_current_coord,square_mid_points = next_location_and_updated_database([uavs[uav].uav_current_coord],square_mid_points)
                    uavs[uav].uav_current_coord = random_walk(uavs[uav].uav_current_coord, max_x, max_y, max_z, step);
                          
            if(shai==7): #as when 26 extra points are  be created, around the localised node with many unlocalised neighbours,
            #we have to reset it to zero,so that code is executed with the normal flow 
                shai=shai-7
                #version=version+1
                         
            track_uav[uav].append(uavs[uav].uav_current_coord);
            #print 'UAV coordinates log  ',track_uav[uav]; #trace
          
        #step 4: nodes update their local view receiving neighbors beacons
        for i in range(len(nodes)):
            nodes[i].NULoN = 0;  # clean the local number of unlocalized neighbors
            nodes[i].my_neigbords = []  #used for debugging
        for i in range(len(nodes)):
            if (step_for_node > 0):
                #print "full scan executed" #trace
                for j in range(i + 1, len(nodes)):
                    dist = distance.euclidean(nodes[i].x_y_zreal, nodes[j].x_y_zreal);
                    ss = dist_to_ss(dist);
                    if (ss > threshold):  #if within range , receive packets. Threshold = -100dBm
                        if (nodes[i].mystatus == 0):
                            nodes[j].NULoN += 1;
                            nodes[j].my_neigbords.append(i)
                        if (nodes[j].mystatus == 0):
                            nodes[i].NULoN += 1;
                            nodes[i].my_neigbords.append(j)
            else:
                #print "only through visible neighbords scan";                
                for j in range(len(nodes[i].visible_neigbords)):
                    dist = distance.euclidean(nodes[i].x_y_zreal, nodes[nodes[i].visible_neigbords[j]].x_y_zreal);
                    #print " node i is ", i, " node j is ",nodes[i].visible_neigbords[j]
                    ss = dist_to_ss(dist);
                    if (ss > threshold):  #if within range , receive packets. Threshold = -100dBm
                        if (nodes[i].mystatus == 0):
                            nodes[nodes[i].visible_neigbords[j]].NULoN += 1;
                            if (nodes[nodes[i].visible_neigbords[j]].my_neigbords.count(i) == 0):
                                nodes[nodes[i].visible_neigbords[j]].my_neigbords.append(i)
                        if (nodes[nodes[i].visible_neigbords[j]].mystatus == 0):
                            nodes[i].NULoN += 1;
                            if (nodes[i].my_neigbords.count(j) == 0):
                                nodes[i].my_neigbords.append(j)


        #step 5: nodes update their location      
        nodes_real_for_each_step.append([])
        nodes_estim_for_each_step.append([])
        for i_node in range(len(nodes)):  #logging data
            nodes_real_for_each_step[s].append(nodes[i_node].x_y_zreal)
            if (step_for_node > 0):
                #nodes[i_node].x_y_zreal = nodes[i_node].x_y_zreal
                nodes[i_node].x_y_zreal = random_walk_node(nodes[i_node].x_y_zreal, max_x, max_y, max_z, step_for_node);
            if (nodes[i_node].mystatus == 1):
                nodes_estim_for_each_step[s].append(nodes[i_node].x_y_zestim)

        #print" step ",s," finished-----------------------------------------------"
        if (finished_flag or len(square_mid_points)==1): #all nodes already localized     not 5   

            break;
    return [uavs, nodes, track_uavs, track_uav, track_nodes, average_nr_neigh, max_distance, localised_unlocalised_ngr,
            nodes_real_for_each_step, nodes_estim_for_each_step]
                        
    
     
