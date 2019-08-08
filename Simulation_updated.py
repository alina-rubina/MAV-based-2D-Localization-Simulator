# -*- coding: utf-8 -*-
"""
Created on Thu Jun 11 13:55:50 2015

@author: AHMAD
"""
from __future__ import division
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import matplotlib.cm as cmx
import matplotlib.colors as colors
import numpy as np
from At_UAV_3D import *
from At_Node_3D import *
from localization import *
from Dist_to_ss import dist_to_ss

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
#import itertools
#from random import shuffle
#import re
#import signal
from scipy import constants
import json
import Algorithm_3D  
#from Algorithm_3D import max_range  
import Algorithm_3D_hybrid
import Algorithm_3D_hybrid_circle

import var_measurements                 #Importing the file that will save the nr. of measurements variable.

final_conclusions_length_of_loc_nodes = []
final_conclusions_Average_loc_error = []
final_conclusions_total_route_length = []
conclusions = []
running_localy = 0
#$data = array($numofsims, $numofnodes,$numofuavs,$threshold,$stepsize,$steps,$maxx,$maxy,$maxz,$step_for_node, $loc_method,$loc_filter);
#data = [1,100,3,-100,20,100,200,200,40,15]


try:
        data = json.loads(sys.argv[1]);
	
except:
        print("Could not read data");
  
        x_max = 200; #used for video
        y_max = 200; #used for video
        z_max = 0;
# main localization, will use localisation_method and localisation_filtering to select which localization exectly to be used
        #localisation_method = 0 ;#--> centroid localisation (default) was here
        #localisation_method = 1 ;#--> trilateration localisation  2d 
        ##localisation_method = 2 ;#--> multilateration localisation from copter 2d
        #localisation_method = 3 ;#--> weighted centroid localisation 3d
        #localisation_method = 4 ;#--> trilateration 3D from students delete
        #localisation_method = 5 ; #--> trilateration 3d 3 points //  takes 3 highest RSSI, not colinear 3d
        #localisation_method = 6; #-->  localization_2D_DCE /// takes 3 first dots in filtered_db 2d
        #localisation_method = 7; #-->  localization_minmaxshrink  3d
        localisation_method = 8; #-->  Least Squares 
        
        #------------------------------------------------------
        #localisation_filtering = 0; #--> no filtering (default)
        #
        localisation_filtering = 1; #--> SS filtering
        #localisation_filtering = 2; #--> LastThree filtering 
        #localisation_filtering = 3; #--> JointClustering filtering 3 dots ss
        #localisation_filtering = 4; #--> Perimeter filtering , return 4 dots
        #------------------------------------------------------
        copter_trajectory = 4; #--> random walk ok
        #copter_trajectory = 1; #--> circle ok
        #copter_trajectory = 2; #--> triangle ok
        #copter_trajectory = 3; #--> missing
        #copter_trajectory = 4; #--> double scan ok
        #copter_trajectory = 5 ;#--> another_random
        #copter_trajectory = 4 ;#--> hybrid-G
        #copter_trajectory = 7  ;#--> hybrid-circle-
        #localisation_method = 3;
        #localisation_filtering = 2;
        #copter_trajectory = 1;
        k=[]
                #N simulations  #N nodes   #N uavs    treshould    #stepsize   #steps    x_max     y_max    z_max build         L_method                L_filter               copter_trajectory
        data = [   50 ,        100,        1,       -75,          k,       1374,     x_max,    y_max,    z_max,     0,      localisation_method , localisation_filtering,   copter_trajectory];
        running_localy = 1;
        #sys.exit(1);




def main(k,step_1):
    #initialization:
    #np.random.seed(6) # Seed generator, None = system clock
    #uav_current_coord = [0, 0, 0]
    
    nr_measurements = 10                            #Changing the number of measurements per anchor.
    var_measurements.init(nr_measurements)          #Saving the variable into another file to make it accessible for the dist_to_ss function.
   
    uavs = []    
    nodes = []
    sim_number = int(data[0]);
    n_uavs = int(data[2]);
    n_nodes = int(data[1]);
    #uavs = [mUav(0,50,100,0,0,0) for count in range(0,n_uavs)]
    #for uav in range(len(uavs)):
    #uavs[uav].uavId=uav;
    max_x = int(data[6]);
    max_y = int(data[7]);
    max_z = int(data[8]);
    threshold = int(data[3]);
    build=0
    #step = int(data[4]); # step size of one UAV movement
    steps = int(data[5]); # number of UAV movements
    step_for_node = 0
    #max_distance = ss_to_dist.ss_to_dist(threshold);    
    max_distance = ss_to_dist(threshold);
    #print max_distance
    localisation_method = int(data[10]); #added for**** different localisation methods
    #max_distance=int(max_distance)
    #print 'bla' , max_distance
#    copter_trajectory = int(k);
## step size of one uav movement   
##        #print step
#    if(copter_trajectory==0):
#        copter_trajectory == 0
#        print k
#    elif(copter_trajectory==2):
#        copter_trajectory == 2
#        #print step
#    elif(copter_trajectory==4):
#        copter_trajectory == 4
#    elif(copter_trajectory==6):
#        copter_trajectory == 6 
#        #print step
#    else:        
#        copter_trajectory == 7
    step_size = int(k);
    if (step_size==0):
        step = (1/3)*max_distance # step size of one UAV movement   
        #print step
    elif(step_size==1):
        step = (2/3)*max_distance
        #print step
    elif(step_size==2):
        step = 1*max_distance
        #print step
    elif(step_size==3):
        step = (4/3)*max_distance
        #print step
    else:        
        step = (5/3)*max_distance
        #print step
    s_Average_loc_error= 0;
    s_total_route_Length= 0;
    s_aver_route_len= 0;
    s_localised_nodes= 0;
    list_Average_loc_error=[];
    list_PDF_Average_loc_error=[];
    list_total_route_Length=[];
    list_PDF_total_route_Length=[];
    list_aver_route_len=[];
    list_PDF_aver_route_len=[];
    list_localised_nodes=[];
    list_PDF_localised_nodes=[];
    
   
    localisation_filtering = int(data[11]); #added for localisation filtering
    copter_trajectory = int(data[12]);
    loc_method_str, loc_filter_str = localization(0, 0,localisation_method,localisation_filtering,1); # last 1 will do reporting = convert value to strings 

    files_name = "logs/Run"+"_"+str(sim_number)+"_Nodes"+"_"+str(n_nodes)+"_"+"UAVs"+"_"+str(n_uavs)+"_"+"ssUAV_"+str(step)+"_"+"ssNode"+"_"+str(step_for_node)+"_"+"StepsEachUAV"+"_"+str(steps)+"_Date"+time.strftime("%d-%m-%y-%H-%M")+"_LocMetd_"+str(localisation_method)+"_Filt_"+str(localisation_filtering)+".txt"
    fo = open(files_name, "wb")
    fo.write( "#"+"Localisation Method: "+loc_method_str+"\r\n");
    fo.write( "#"+"Filter: "+loc_filter_str+"\r\n");
    fo.write( "#"+"trajectoty number: "+str(copter_trajectory)+"\r\n");
    fo.write( "#"+"The Simulation Number"+"    "+"Average_loc_error"+"         "+"The Total Route Length"+"         "+"The Average Route Length"+"         "+"The Number of localized nodes"+"    "+"\r\n")
  

    for sim_counter in range (sim_number):
        
        #localised_nodes = []
        track_uavs = [];
        track_uav = [];
        #track_uavs.append(uavs[uav].uav_current_coord);
        #track_uav.append(uav_current_coord);
        track_nodes = []
        localised_unlocalised_ngr = [];
        nodes_real_for_each_step = [];
        nodes_estim_for_each_step = [];
        uav_coords = [];
        average_nr_neigh = 0;
        #node_list = [100]    
        line_colors = ['b','g','r','c','m','y','k','w'];
        if copter_trajectory == 6:
            The_Result = Algorithm_3D_hybrid.main(uavs, nodes, n_uavs, n_nodes, 
                                           max_x, max_y, max_z, threshold, 
                                           step, steps, max_distance,
                                           track_uavs, track_uav, track_nodes, 
                                           average_nr_neigh,localised_unlocalised_ngr,step_for_node,
                                           build, localisation_method, localisation_filtering, copter_trajectory);
                                           #"static", 0,0,0,0, #trajectory_type, building, build_X, build_Y, build_Z, 
                                           #build, localisation_method, localisation_filtering);
        elif copter_trajectory == 7:                                
            The_Result = Algorithm_3D_hybrid_circle.main(uavs, nodes, n_uavs, n_nodes, 
                                           max_x, max_y, max_z, threshold, 
                                           step, steps, max_distance,
                                           track_uavs, track_uav, track_nodes, 
                                           average_nr_neigh,localised_unlocalised_ngr,step_for_node,
                                           build, localisation_method, localisation_filtering, copter_trajectory);
                                           #"static", 0,0,0,0, #trajectory_type, building, build_X, build_Y, build_Z, 
                                           #build, localisation_method, localisation_filtering);    
        else:   
            The_Result = Algorithm_3D.main(uavs, nodes, n_uavs, n_nodes, 
                                           max_x, max_y, max_z, threshold, 
                                           step, steps, max_distance,
                                           track_uavs, track_uav, track_nodes, 
                                           average_nr_neigh,localised_unlocalised_ngr,step_for_node,
                                           build, localisation_method, localisation_filtering, copter_trajectory);
                                           #"static", 0,0,0,0, #trajectory_type, building, build_X, build_Y, build_Z, 
                                           #build, localisation_method, localisation_filtering);    
    
      
        uavs = The_Result[0];
        nodes = The_Result[1];
        track_uavs = The_Result[2];
        track_uav = The_Result[3];
        track_nodes = The_Result[4];
        average_nr_neigh = The_Result[5];
        max_distance = The_Result[6];
        localised_unlocalised_ngr = The_Result[7];
        nodes_real_for_each_step = The_Result[8];
        nodes_estim_for_each_step = The_Result[9];
              
        #print            "nodes_real_for_each_step", nodes_real_for_each_step[0]     
        #print            "track_uav", track_uav[0]     
        #print            "track_uavs", track_uavs[0]     
        #print            "nodes", nodes_estim_for_each_step
            #conclusion: plot results   
        #print ("---------Setup-----------")
        #print ("Nodes: 200", '; area: ', max_x*2, 'x', max_y*2, 'x', max_z*2);//ec
        #print ('Threshold:',threshold, '; Tx radius:', ss_to_dist(threshold));//ec
        #print ('Sim steps: ', steps, '; step size: ', step);//ec
        #print ('Average # of neighbors: ', average_nr_neigh/n_nodes);//ec
        #print ("---------Conclusions-----------")//ec
    
                        
        for i in range(len(uavs)):
        #for i in range(1):
            for j in range(len(track_uav[i])):
                #print ('track of uav ',i,' step ',j, ' coordinates',track_uav[i][j]);
                uav_coords.append([i,j,track_uav[i][j]]);

            
        localised_nodes = [];
        loc_error = 0;
        total_route_Length = 0;
        #route_length = 0;
        for i in range(len(nodes)):
            if(nodes[i].mystatus == 1):
                localised_nodes.append(nodes[i].x_y_zestim);
                loc_error += distance.euclidean(nodes[i].x_y_zestim, nodes[i].x_y_zreal);
#        if running_localy:        
#            print ('Number of loc nodes = ', len(localised_nodes));#//ec
#        for uav in range(1):    
#        if (running_localy == 1):
##            fig = plt.figure()
##            ax = fig.add_subplot(111);
#            fig_m = plt.figure()
#            bx = fig_m.add_subplot(111);
#            for uav in range(len(uavs)):
#                plot(np.transpose(track_uav[uav])[0], np.transpose(track_uav[uav])[1],color='#0066CC',linewidth=2,label='MAVs Trajectory');
##                plt.ylim(-200, 200)
#                plt.xlim(-200, 200)
#             #   plt.legend(loc='upper left',fontsize=18)
#                plt.xlabel('x, m',fontsize=18)
#                plt.ylabel('y, m',fontsize=18)
#                ax1 = gca()    
#                #ax1.axis([0, 100,0,105])
#                for tick in ax1.yaxis.get_major_ticks(): 
#                    tick.label1.set_fontsize('16')    ##font size of text  on x-axis
#        #tick.label1.set_fontweight('bold')
#
#                for tick in ax1.xaxis.get_major_ticks(): 
#                    tick.label1.set_fontsize('16')  ##font size of text on y-axis
#        #tick.label1.set_fontweight('bold')   
##plt.title(' Route length vs localisation error using '+ localisation +' centroid'  ,fontsize=26)
#                ax1 = plt.axes()
#                plt.show()
#    #        Axes3D.
#    
#            for i in range(len(nodes)):
#                bx.scatter(np.transpose(nodes[i].x_y_zreal)[0], np.transpose(nodes[i].x_y_zreal)[1], s=30, c='#FF8800',label='Unknown nodes')
###                #plt.legend(loc='upper left')
#            for i in range(len(nodes)):
#                bx.scatter(np.transpose(nodes[i].x_y_zestim)[0], np.transpose(nodes[i].x_y_zestim)[1], s=20, c=u'b')
##                    
#        ax.plot_wireframe(np.transpose(track_uav)[0], np.transpose(track_uav)[1], np.transpose(track_uav)[2])
#        ax.scatter(np.transpose(track_nodes)[0], np.transpose(track_nodes)[1], np.transpose(track_nodes)[2], c='r')//ec
#        plot(np.transpose(track_uav[0])[0], np.transpose(track_uav[0])[1],'b-');    
        if(len(localised_nodes)>0):
            #plot(np.transpose(localised_nodes)[0], np.transpose(localised_nodes)[1], 'g^', fillstyle='none');
            #ax.scatter(np.transpose(localised_nodes)[0], np.transpose(localised_nodes)[1], np.transpose(localised_nodes)[2], 'g');//ec
            Average_loc_error = loc_error/len(localised_nodes);
            #print ('Average loc error = ', Average_loc_error);//ec
        else :
            Average_loc_error = "undefined"
            #Average_loc_error = 0;
    
        
        for i in range(len(track_uav)):
            uavs[i].route_length = 0
            for j in range(len(track_uav[i])):
               if ((j+1)<len(track_uav[i])):
                 uavs[i].route_length += distance.euclidean(track_uav[i][j], track_uav[i][j+1]);
            total_route_Length += uavs[i].route_length;
        #print "total length", total_route_Length
        #for i in range(len(track_uav)):
        #    print "total length uav=",i," ", uavs[i].route_length
        number_of_steps_done = len(track_uav[0])
        print number_of_steps_done
        aver_route_len = total_route_Length/len(uavs)

        list_Average_loc_error.append(Average_loc_error);
        list_total_route_Length.append(total_route_Length);
        list_aver_route_len.append(aver_route_len);
        list_localised_nodes.append(len(localised_nodes));

        if not isinstance(Average_loc_error, basestring):
            s_Average_loc_error+= Average_loc_error;
        else: 
            s_Average_loc_error = 0;
        s_total_route_Length+= total_route_Length;
        s_aver_route_len+= aver_route_len;
        s_localised_nodes+= len(localised_nodes);
        if running_localy: 
            print "Name of the file: ", fo.name
            print "Closed or not : ", fo.closed
            print "Opening mode : ", fo.mode
            print "Softspace flag : ", fo.softspace
        fo.write("#"+str(sim_counter)+"                         ")
        fo.write(str(Average_loc_error)+"             ");
        fo.write(str(total_route_Length)+"                   ");
        fo.write(str(aver_route_len)+"                       ");
        fo.write(str(len(localised_nodes))+"\r\n");

        The_Mean_Average_loc_error = (s_Average_loc_error)/sim_number;
        The_Standard_Deviation_Average_loc_error = np.std(list_Average_loc_error);

        if The_Standard_Deviation_Average_loc_error > 0:                               
            PDF_Average_loc_error = (1/((sqrt(2*pi))*The_Standard_Deviation_Average_loc_error))*exp(-1/(2*(The_Standard_Deviation_Average_loc_error**2))*((list_Average_loc_error[i]-The_Mean_Average_loc_error)**2));
        else:
            PDF_Average_loc_error = 0;
        list_PDF_Average_loc_error.append(PDF_Average_loc_error);
                                                 
        The_Mean_total_route_Length = (s_total_route_Length)/sim_number;
        The_Standard_Deviation_total_route_Length = np.std(list_total_route_Length);    
        if The_Standard_Deviation_total_route_Length > 0:                                                   
            PDF_total_route_Length = (1/((sqrt(2*pi))*The_Standard_Deviation_total_route_Length))*exp(-1/(2*(The_Standard_Deviation_total_route_Length**2))*((list_total_route_Length[i]-The_Mean_total_route_Length)**2));
        else:
            PDF_total_route_Length = 0;  
        list_PDF_total_route_Length.append(PDF_total_route_Length);
                                                  
        The_Mean_aver_route_len = (s_aver_route_len)/sim_number;
        The_Standard_Deviation_aver_route_len = np.std(list_aver_route_len);
        if The_Standard_Deviation_aver_route_len > 0: 
            PDF_aver_route_len = (1/((sqrt(2*pi))*The_Standard_Deviation_aver_route_len))*exp(-1/(2*(The_Standard_Deviation_aver_route_len**2))*((list_aver_route_len[i]-The_Mean_aver_route_len)**2));            
        else:
            PDF_aver_route_len = 0;    
        list_PDF_aver_route_len.append(PDF_aver_route_len);    
                                      
        The_Mean_localised_nodes = (s_localised_nodes)/sim_number;
        The_Standard_Deviation_localised_nodes = np.std(list_localised_nodes);
        if The_Standard_Deviation_localised_nodes > 0: 
            PDF_localised_nodes = (1/((sqrt(2*pi))*The_Standard_Deviation_localised_nodes))*exp(-1/(2*(The_Standard_Deviation_localised_nodes**2))*((list_localised_nodes[i]-The_Mean_localised_nodes)**2));         
        else:
            PDF_localised_nodes = 0;  
        list_PDF_localised_nodes.append(PDF_localised_nodes);
    
    fo.write("#----------------------------------------------------------------------------------------------------------------------------------------------------------"+"\r\n")
    fo.write("#The Final Results"+"        "+"Mean_Average_loc_error"+"         "+"Mean_The Total Route Length"+"         "+"Mean_The Average Route Length"+"         "+"The_Mean_localised_nodes"+"\r\n")
    fo.write("#                            "+str(The_Mean_Average_loc_error)+"                "+str(The_Mean_total_route_Length)+"                          "+str(The_Mean_aver_route_len)+"                          "+str(The_Mean_localised_nodes)+"\r\n")
    fo.write("\r\n");
    fo.write("#                         "+"Std_Average_loc_error"+"          "+"Std_The Total Route Length"+"          "+"Std_The Average Route Length"+"          "+"The_Std_localised_nodes"+"\r\n")
    fo.write("#                            "+str(The_Standard_Deviation_Average_loc_error)+"               "+str(The_Standard_Deviation_total_route_Length)+"                         "+str(The_Standard_Deviation_aver_route_len)+"                         "+str(The_Standard_Deviation_localised_nodes)+"\r\n")
    fo.write("#----------------------------------------------------------------------------------------------------------------------------------------------------------"+"\r\n")
    fo.write("#The Final Results"+"        "+"PDF_Average_loc_error"+"         "+"Average_loc_error"+"         "+"PDF_The Total Route Length"+"         "+"The Total Route Length"+"         "+"PDF_The Average Route Length"+"         "+"The Average Route Length"+"         "+"PDF_localised_nodes"+"         "+"localised_nodes"+"\r\n")
    for i in range(sim_number):
        fo.write("                           "+str(list_PDF_Average_loc_error[i]).rjust(4)+"               "+str(list_Average_loc_error[i])+"              "+str(list_PDF_total_route_Length[i])+"                    "+str(list_total_route_Length[i])+"                 "+str(list_PDF_aver_route_len[i])+"                    "+str(list_aver_route_len[i])+"                 "+str(list_PDF_localised_nodes[i])+"               "+str(list_localised_nodes[i])+"\r\n")
#        fo.write("                           "+str(list_PDF_Average_loc_error[i])+"               "+"\r\n")
#        fo.write("                           "+str(list_Average_loc_error[i])+"              "+str(list_PDF_total_route_Length[i])+"   \r\n")
#        fo.write("                           "+str(list_PDF_Average_loc_error[i])+"               "+str(list_Average_loc_error[i])+"              "+str(list_PDF_total_route_Length[i])+" \r\n")
#        fo.write("                           "+str(list_PDF_Average_loc_error[i])+"               "+str(list_Average_loc_error[i])+"              "+str(list_PDF_total_route_Length[i])+"                    "+str(list_total_route_Length[i])+"                 "+str(list_PDF_aver_route_len[i])+"                    "+str(list_aver_route_len[i])+"                 "+str(list_PDF_localised_nodes[i])+"               "+str(list_localised_nodes[i])+"\r\n")
#        fo.write("                           "+str(list_PDF_Average_loc_error[i])+"               "+str(list_Average_loc_error[i])+"              "+str(list_PDF_total_route_Length[i])+"                    "+str(list_total_route_Length[i])+"                 "+str(list_PDF_aver_route_len[i])+"                    "+str(list_aver_route_len[i])+"                 "+str(list_PDF_localised_nodes[i])+"               "+str(list_localised_nodes[i])+"\r\n")
#        fo.write("                          "+str(list_PDF_aver_route_len[i])+"                    "+str(list_aver_route_len[i])+"            \r\n")
#        fo.write("                           "+str(list_PDF_localised_nodes[i])+"               "+str(list_localised_nodes[i])+"\r\n")
    fo.close()

    return[len(uavs), total_route_Length/len(uavs), track_uavs, track_uav, track_nodes,localised_nodes,average_nr_neigh,max_distance,len(localised_nodes),Average_loc_error,
           total_route_Length, localised_unlocalised_ngr,uav_coords,number_of_steps_done, nodes_real_for_each_step, nodes_estim_for_each_step ]

#print "I AM STARTING..."   
#print  "started at :",time.strftime("%d-%m-%y-%H-%M")
main(0,1374)
print "----------------------------------------------------------------------------------------------"
print "PLEASE WAIT I AM NOT FINISHED YET;STEPSIZE=30,DONE at",time.strftime("%d-%m-%y-%H-%M") 
main(1,1374)
#print "----------------------------------------------------------------------------------------------"
#print "PLEASE WAIT I AM NOT FINISHED YET;STEPSIZE=40,DONE at",time.strftime("%d-%m-%y-%H-%M") 
main(2,1374)
#print "----------------------------------------------------------------------------------------------"
#print "PLEASE WAIT I AM NOT FINISHED YET;STEPSIZE=50,DONE at ",time.strftime("%d-%m-%y-%H-%M") 
main(3,1374)
print "PLEASE WAIT I AM NOT FINISHED YET;STEPSIZE=60,DONE at "  ,time.strftime("%d-%m-%y-%H-%M")   
##
#print "----------------------------------------------------------------------------------------------"    
main(4,1374)
##print "PLEASE WAIT I AM NOT FINISHED YET;STEPSIZE=70,DONE at " ,time.strftime("%d-%m-%y-%H-%M")   
#
##main(70,300)
##print "PLEASE WAIT I AM NOT FINISHED YET;STEPSIZE=80,DONE at " ,time.strftime("%d-%m-%y-%H-%M")  
###main(90,250)
#
##print "PLEASE WAIT I AM NOT FINISHED YET;STEPSIZE=90,DONE at " ,time.strftime("%d-%m-%y-%H-%M")  
#print "----------------------------------------------------------------------------------------------"             
#print "-----------------------DONE :-) ,THANK YOU FOR YOUR TIME--------------------------------------"   
#print "----------------------------------------------------------------------------------------------"  
           
#if __name__ == '__main__':
#  for i in range(0,1):
#        conclusions = main()
#        final_conclusions_length_of_loc_nodes.append(conclusions[0])
#        final_conclusions_Average_loc_error.append(conclusions[1])
#        final_conclusions_total_route_length.append(conclusions[2])
#        logging.basicConfig(filename='example.txt',level=logging.DEBUG)
#        logging.info(len(conclusions[11]))

 
#bulkData = {"accumuavs":conclusions[0],"tuavs":conclusions[2],"avgroutelen":conclusions[1]
#            ,"tuav":conclusions[3],"tnodes":conclusions[4],"locnodes":conclusions[5],"averagenn":conclusions[6],"maxdis":conclusions[7]
#            ,"noflocnodes":conclusions[8],"averaglocerrot":conclusions[9]
#            ,"totalroutelen":conclusions[10],"locunlocngr":conclusions[11],"coords":conclusions[12],"steps_done":conclusions[13],"nodes_real_for_each_step":conclusions[14] , "nodes_estim_for_each_step":conclusions[15]}
#print json.dumps(bulkData);
#                  
