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
from At_UAV_3D import *
from At_Node_3D import *
from localization import *
from Dist_to_ss import dist_to_ss
from ss_to_dist import *
from pylab import *
import matplotlib.pyplot as plt
from Setup import *
from rwalk_3D import random_walk
from rwalk_3D import circle_walk
from rwalk_3D_node import random_walk_node
from operator import itemgetter
from scipy.spatial import distance
import time
from triangle import triangle                 #Trajectory 1
from Doublescan import DoubleScan             #Trajectory 2
from circle import circle


final_conclusions_length_of_loc_nodes = []
final_conclusions_Average_loc_error = []
final_conclusions_total_route_length = []
conclusions = []
nodes_real_for_each_step = []
nodes_estim_for_each_step = []

def next_copter_coordinates(come_uav_current_coord, come_max_x, come_max_y, come_max_z, come_step, come_copter_trajectory):
    if come_copter_trajectory == 0: # random walk
        return random_walk(come_uav_current_coord, come_max_x, come_max_y, come_max_z, come_step);
    if come_copter_trajectory == 1: # circle
        return circle (come_uav_current_coord, come_max_x, come_max_y, come_max_z, come_step);                
    if come_copter_trajectory == 2: # triangle 
        return triangle (come_uav_current_coord, come_max_x, come_max_y, come_max_z, come_step);    
    if come_copter_trajectory == 3: # circle_walk 
        return circle_walk (come_uav_current_coord, come_max_x, come_max_y, come_max_z, come_step);   
    if come_copter_trajectory == 4: # double_scan 
        return DoubleScan (come_uav_current_coord, come_max_x, come_max_y, come_max_z, come_step);   
    if come_copter_trajectory == 5: # another_random
        return DoubleScan (come_uav_current_coord, come_max_x, come_max_y, come_max_z, come_step);         
    if come_copter_trajectory == 6: # hybrid-circle
        return DoubleScan (come_uav_current_coord, come_max_x, come_max_y, come_max_z, come_step);         

# Setup.main(uavs, nodes, n_uavs, n_nodes, max_x, max_y, max_z, step, steps, max_distance, track_uavs, track_uav, track_nodes, average_nr_neigh, line_colors);
def main(uavs, nodes, n_uavs, n_nodes, max_x, max_y, max_z, threshold, step, steps, max_distance, track_uavs, track_uav,
         track_nodes, average_nr_neigh, localised_unlocalised_ngr, step_for_node, build = 0, localisation_method = 0, localisation_filtering = 0, copter_trajectory = 0):
    #create and fill in with initial data: uavId, step, steps and x, y, z coordinates of UAV 
    #max_distance = ss_to_dist.ss_to_dist(threshold);  
         
#start here 
    global max_range; #for hybrid-circle
    max_range = ss_to_dist(threshold); #for hybrid-circle
    #print max_range

    global temp;
    global R
    global shai; #updates its valve by one for every new cycle 
    shai=0;
    R=ss_to_dist(threshold); 
    #----------------------------------------starting coordinates selection for trajectories--------------
    temp=[ step , 0, max_z]    
    if copter_trajectory == 2: # for triangle
        temp=[-max_x, -max_y, max_z]; #Provides the first point,which is important to obtain the next points in Dualscan trajectory
    if copter_trajectory == 4: # for double_scan
        temp=[-max_x, -max_y, max_z]; #Provides the first point,which is important to obtain the next points in Dualscan trajectory

    if copter_trajectory == 1: #for circle
        temp=[ step , 0, max_z]; #Provides the first point,which is important to obtain the next points in Dualscan trajectory
#stop here
    uavs = [mUav(0, step, steps, 0, 0, 0) for count in range(0, n_uavs)]
    for uav in range(len(uavs)):
        track_uav.append([])
        uavs[uav].uavId = uav;
        #uavs[uav].uav_current_coord = [0, 0, 0];
        if copter_trajectory == 5 or copter_trajectory ==0:
            uavs[uav].uav_current_coord = [-max_x, -max_y, max_z];
        else:
            uavs[uav].uav_current_coord = [-max_x,-max_y, 0];

        track_uavs.append(uavs[uav].uav_current_coord);


    nodes = [mNode(0, 0, 0, 0) for count1 in range(0, n_nodes)]
    #print len(nodes)
    #--------------------------------------------modif according to original map
    for node in range(len(nodes)):
        nodes[node].myId = node;
        #nodes[node].x_y_real=[np.random.uniform(-500,500), np.random.uniform(-500,500)];
       # nodes[node].x_y_zreal=[np.random.uniform(-1*max_x,max_x), np.random.uniform(-1*max_y,max_y),0];
    #mixed
#        if (node <= int(0.25 * n_nodes)):
#            nodes[node].x_y_zreal = [np.random.uniform(-0.4 * max_x, -0.8 * max_x),
#                                     np.random.uniform(-0.8*max_y, -0.4 * max_y),
#                    np.random.uniform(0,0)];  # idmt
#        
#
#        if (node > int(0.25 * n_nodes) and node <= int(0.5 * n_nodes)):
#            nodes[node].x_y_zreal = [np.random.uniform(-0.4 * max_x, -0.8 * max_x), np.random.uniform(0.4*max_y, 0.8 * max_y),
#                                     np.random.uniform(0, 0)];  # usz
#        if (node > int(0.5 * n_nodes) and node <= int(0.75 * n_nodes)):
#            nodes[node].x_y_zreal = [np.random.uniform(0 * max_x, 1* max_x),
#                                     np.random.uniform(-1*max_y, 0 * max_y),
#                                     #np.random.uniform(-0.0 * max_z, 0.8 * max_z)];  # haus H
#                                     np.random.uniform(0, 0)];  # haus H
##       
#        if (node > int(0.75 * n_nodes) and node <= int( n_nodes)):
#            nodes[node].x_y_zreal = [np.random.uniform(0 * max_x, 1 * max_x),
#                                     np.random.uniform(0*max_y, 1 * max_y),
#                                     #np.random.uniform(0.28264 * max_z, 0.41864 * max_z)];  # haus I
#                                     np.random.uniform(0, 0)];  # haus I
#        #clusters
       
          
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
#       
        track_nodes.append(nodes[node].x_y_zreal);
#        print 'bla' , nodes[node].x_y_zreal

    dist_inter = []
#    print test.size
    #create_list_of_poten_neighb()
    for i in range(len(nodes)):
        nodes[i].NULoN = 0;  # clean the local number of unlocalized neighbors
    if (step_for_node == 0):
        #    creating list of potential neighboars
        for i in range(len(nodes)):
            for j in range(i + 1, len(nodes)):
                dist = distance.euclidean(nodes[i].x_y_zreal, nodes[j].x_y_zreal);
#                test[i]=dist.append(i)
               
                dist_inter=np.append(dist_inter,dist)
#                print test
                if dist < max_distance * 1.5:  # we take 50% more than max possibe distance for current treshould
                    ss = dist_to_ss(dist);
                    if (ss > threshold):  #if within range , receive packets. Threshold = -100dBm
                        if (nodes[i].mystatus == 0):
                            nodes[j].NULoN += 1;
                        if (nodes[j].mystatus == 0):
                            nodes[i].NULoN += 1;
                        nodes[i].visible_neigbords.append(j);
                        nodes[j].visible_neigbords.append(i);

    
#    print 'test',test.size
#    dist_inter_mean=np.mean(dist_inter)
#    dist_inter_std=np.std(dist_inter)
##    test=np.sum(dist_inter)/len(dist_inter)
#    print max_distance
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
                if (ss > threshold):  #if within range , receive packets. Threshold = -100dBm
                    RX_packet = nodes[node].packet_transmit();  #receive the packet transmitted by the node
                    nodes[node].update_database(RX_packet, ss, uavs[
                        uav].uav_current_coord);  # store the packet and ss, uav coordinates in the object nodes directly
                    #uavs[uav].update_database(RX_packet,ss,uavs[uav].uav_current_coord); #store packet also inside UAV,            
                    nodes[node].NULoN_UAV_view = RX_packet[3];  # how many unlocalized nodes around node
                    uavs[uav].save_node_NULON(node, RX_packet[3]);  #add node number and how many unlocalized nodes around it to UAV DB

                    #step 2: Perform localisation and Update of localised_nodes, since new data arrieved
                    #localisation_method, localisation_filtering are coming from simulation via main parameters
                    res = localization(node, nodes, localisation_method, localisation_filtering);
                    if (res != 0):#Localization was successful: update the local view of the UAV and inform the node
                        #print" localized node ", node , " neighb here", nodes[node].my_neigbords, " known ", nodes[node].mystatus
                        nodes[node].mystatus = 1;
                        nodes[node].x_y_zestim = res;
              
            #step 3: UAV moves to a new position. Decision is based on his local view on the network. The UAV moves towards the node with the bigger number of unloc neighbours or does the random walk if the number of neighbors is 0
            localised_nodes = [];
            loc_nodes = 0;

            for i in range(len(nodes)):
                if (nodes[i].mystatus == 1):
                    loc_nodes += 1;
            if (len(nodes)==len(localised_nodes)):
               # if(steps==20):
                finished_flag = 1;
                break;

            for i in range(len(uavs[uav].unlocalized_nodes)):
                if ((nodes[uavs[uav].unlocalized_nodes[i][0]].mystatus == 1) & (uavs[uav].unlocalized_nodes[i][1] > 0)):  
                # UAV received packet from localized node and node has unlocalized memebers
                    localised_nodes.append([uavs[uav].unlocalized_nodes[i][0], uavs[uav].unlocalized_nodes[i][1]]);
            localised_nodes.sort(key=itemgetter(1));  #sorting according to localised nodes with highest NULoN(Number of Unlocalised Neighbours)
            localised_nodes.reverse();

            #localised to unlocalised neighbors
            localised_unlocalised_ngr.append(['Number of loc nodes with unloc neighbors =', len(localised_nodes)]);

            if (len(localised_nodes) > 0):
                #print "that is what first node know "," node number ", nodes[localised_nodes[0][0]].myId , " estim",  nodes[localised_nodes[0][0]].x_y_zestim, " real ",  nodes[localised_nodes[0][0]].x_y_zreal
                i = 0;
                j += 1;

                if (distance.euclidean(uavs[uav].uav_current_coord, nodes[localised_nodes[i][0]].x_y_zestim) < step / 4):  
                # this limits small movement
                    if i + 1 < len(localised_nodes):
                        #print "that is what second node know "," node number ", nodes[localised_nodes[1][0]].myId , " neighboard", nodes[localised_nodes[1][0]].my_neigbords , " estim",  nodes[localised_nodes[1][0]].x_y_zestim, " real ",  nodes[localised_nodes[1][0]].x_y_zreal
                        #uavs[uav].uav_current_coord = nodes[localised_nodes[i+1][0]].x_y_zestim;#we go to next localized node if we already in place of node with max unlocal. memebers
                        if copter_trajectory == 5:
                            uavs[uav].uav_current_coord = circle_walk(uavs[uav].uav_current_coord, max_x, max_y, max_z,step, j);
                        else:
                            uavs[uav].uav_current_coord = temp;
                            temp = next_copter_coordinates(uavs[uav].uav_current_coord, max_x, max_y, max_z, step, copter_trajectory); 
                        
                    else:
                        if copter_trajectory == 5:
                            uavs[uav].uav_current_coord = circle_walk(uavs[uav].uav_current_coord, max_x, max_y, max_z,
                                                                  step, j);

                        else:
                            uavs[uav].uav_current_coord = temp;
                            temp = next_copter_coordinates(uavs[uav].uav_current_coord, max_x, max_y, max_z, step, copter_trajectory); 
          
                        #print "made random walk bcs don't have more unknown", uavs[uav].uav_current_coord #trace
                else:
                    if copter_trajectory == 0 or copter_trajectory == 5: # moving to estimated position of node when random_walk is used, next_copter_coordinates can not do it
                        uavs[uav].uav_current_coord = nodes[localised_nodes[i][0]].x_y_zestim;
                        temp = next_copter_coordinates(uavs[uav].uav_current_coord, max_x, max_y, max_z, step, copter_trajectory);
                    else:
                        uavs[uav].uav_current_coord = temp;
                        temp = next_copter_coordinates(uavs[uav].uav_current_coord, max_x, max_y, max_z, step, copter_trajectory);                   
        


                    #print "moved to node with max unknown",uavs[uav].uav_current_coord #trace
            else:  # we don't have any visible nodes with known unlocalized neighboard, do random walk
                if copter_trajectory == 5:
                    
                    uavs[uav].uav_current_coord = random_walk(uavs[uav].uav_current_coord, max_x, max_y, max_z, step);
                    j = 0;
                else:
                    uavs[uav].uav_current_coord = temp; #DOuble scan trajectory
                    temp = next_copter_coordinates(uavs[uav].uav_current_coord, max_x, max_y, max_z, step, copter_trajectory);                

                #print "made random walk to " ,uavs[uav].uav_current_coord #trace
            #track_uav.insert(uav+1, uavs[uav].uav_current_coord); #original code
            if copter_trajectory!=5:
                if(uavs[uav].uav_current_coord == [-max_x,-max_y, 0]): 
                    shai=shai+1;
            track_uav[uav].append(uavs[uav].uav_current_coord);
            #print 'UAV coordinates log  ',track_uav[uav]; #trace


        #print ' Step 4 is here uav= ',uav, ' step = ', s ;            
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
        if (finished_flag or shai==2):  #all nodes already localized     not 5   
        #if (finished_flag):  #all nodes already localized
            break;
        #print time.clock() - start_time_cycle, " for 1 step seconds"
    return [uavs, nodes, track_uavs, track_uav, track_nodes, average_nr_neigh, max_distance, localised_unlocalised_ngr,
            nodes_real_for_each_step, nodes_estim_for_each_step]
                        
    
     
