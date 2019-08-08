# -*- coding: utf-8 -*-
"""
Created on Thu Jun 11 14:02:33 2015

@author: AHMAD
"""
from utils import *;
from math import *;
from ss_to_dist import ss_to_dist;
from filters import *;
from dsmisc import *;
from multilat import *;
from At_Node_3D import *;
import numpy as np       ;                                 
from numpy import sqrt, dot, cross    ;                   
from numpy.linalg import norm    ;
import time;
import re;
import scipy;
from localiz_utils import *;

from scipy.optimize import minimize, least_squares


circles_factor = 1 # for very erroneous data tuning factor between 0.8 and 1.2 can be selected 

###############################################################################################################################
class AnchorValues:                              
    
    def __init__(self,DataBase):       #Take all the datavalues from the DataBase_Anchors function. Make sure DataBase_Anchors returns SS not distances.
        self.DataBase = DataBase

    def objective(self,x):              #Objective Method creates the function we want to minimize.
        x_node = x[0]                   #Just for making it easier to read.
        y_node = x[1]
        z_node = x[2]
    
        spheres_vector = []  
        sum_weight_i = 0
        
        for i in range(len(self.DataBase)):        #Iterate till the last row of DataBase.
            
            sphere_i = (x_node-self.DataBase[i][0])**2 + (y_node-self.DataBase[i][1])**2 + (z_node-self.DataBase[i][2])**2
            sphere_i =  np.sqrt(sphere_i)
            sphere_i = (sphere_i-self.DataBase[i][4])**2 
            weight_i = (75 - np.abs(self.DataBase[i][3]))**2 
            #weight_i = self.DataBase[i][3]
            sum_weight_i = sum_weight_i + weight_i
            
            sphere_i = sphere_i * weight_i
            spheres_vector = np.append( spheres_vector, sphere_i )
            
        result = np.sum(spheres_vector) / sum_weight_i     
        return result
    
    def minimize_F(self,x0):             # minimize_F  Method with compute the point that has the smallest squared error.
        solution = minimize(self.objective,x0,method = 'CG')   #We can use other techniques like Powell,SLSQP,BFGS,Nelder-Mead,CG
        #solution = least_squares(self.objective,x0)           #We can use Least Squares not minimize. They perform very similar.
        return solution                                        #Return the solution of the minimization algorithm.

def Least_Squares(node_id, nodes, filtered_db):                 #Least Squares or Mimimization_Algorithm.    
        
        DataBase = DataBase_Anchors (node_id, nodes, filtered_db)  #Create the matrix of Anchors. Call the DataBase_Anchors function.

        if ( isinstance(DataBase, (int)) ):                        #Check if DataBase is an Int. If it is return 0. 
             return 0;       
             
        AnchorValsObject = AnchorValues(DataBase)                  #Create an AnchorValues object with the parameters of DataBase.
        
        temp = np.sum(DataBase,axis=0)                   #Either calculate the centroid as a starting point.
        x0 = temp[:3]/len(DataBase)                                  #Keep only the X-Y-Z 
        #x0 = [1,1,0]                                    #Or initiate X0 as a random point.
        
        solution = AnchorValsObject.minimize_F(x0)            #Call the method of minimizing the Error.
        result = [solution.x[0],solution.x[1],solution.x[2]]  #Return the Resulting: estimated position.
                
        return result 

###################################################################################################################       
def DataBase_Anchors (node_id, nodes, filtered_db):   #This function will create a Matrix of the Anchor-data, X-s Y-s Z-s SS-s Distances-s

    if(len(filtered_db) < 3 ):          #If we have less than 4 some Localization Algorithms will not work.Return Int = 0;
        return 0;
    
    #DataBase = np.array ([[1,2,3,4,5],[1,2,3,4,5],[1,2,3,4,5],[1,2,3,4,5]])   #Initialize DataBase.  Old Version. Works only for 4 Anchors.
    DataBase  = np.zeros ((len(filtered_db),5), dtype = int)                   #Initialize DataBase.  New Version. Works will any N Anchors.

    row_index = 0;                               #Variable used to increment the row index of the DataBase Matrix.
             
    for i in filtered_db:                      # filtered_db looks like [4,2,0,6] , It contain IDs of selected DB records

            DataBase[row_index][0] = nodes[node_id].database[i][2][0];   #Column 1 keeps the X-s
            DataBase[row_index][1] = nodes[node_id].database[i][2][1];   #Column 2 keeps the Y-s
            DataBase[row_index][2] = nodes[node_id].database[i][2][2];   #Column 3 keeps the Z-s
            DataBase[row_index][3] = nodes[node_id].database[i][1]       #Column 4 keeps the SS-s
            DataBase[row_index][4] = ss_to_dist(DataBase[row_index][3])      #Column 5 keeps the calculated Distance-s
            row_index = row_index + 1;                                       #Increment the Row index.

    return DataBase                                #Return the Matrix of Anchor-Values.
################################################################################################################### 




def centroid_localisation(node_id, nodes, filtered_db): # centroid localization
    #data set in database: [[[1, 'm', 1, 0], ss, [x_uav, y_uav]]]
    x = 0
    y = 0
    z = 0
    #for i in range(len(nodes[node_id].database)):
    for i in filtered_db:       #we are taking in account only nodes from filtered_db 
        x += nodes[node_id].database[i][2][0];
        y += nodes[node_id].database[i][2][1];
        z += nodes[node_id].database[i][2][2];
    res_x = x / len(filtered_db);
    res_y = y / len(filtered_db);
    res_z = z / len(filtered_db);
    return [res_x ,res_y, res_z];


def weight_centroid_localisationxx(node_id, nodes, filtered_db): # weighted centroid localization
    #data set in database: [[[1, 'm', 1, 0], ss, [x_uav, y_uav]]]
    x = 0
    y = 0
    z = 0
    sum_pr=0;
    for i in filtered_db:
        prss=abs(nodes[node_id].database[i][1]); #prss is RSSI value
        
        prss = (100 - prss)**2
        
        x += prss*nodes[node_id].database[i][2][0];
        y += prss*nodes[node_id].database[i][2][1];
        z += prss*nodes[node_id].database[i][2][2];
        sum_pr=sum_pr+prss;#summation of RSSIs
        
    res_x = x / sum_pr;
    res_y = y / sum_pr;
    res_z = z / sum_pr;
    
    return [res_x, res_y, res_z];

    
# latest localization with 2D trilateration   (Artemenko) 
# input (x,y,Distance) for three coordinates
# output -1: no valid result due to devision by 0, usually due to very erroneous data
# output [x,y] as a point object result of the localization  
def trilateration_localization(node_id, nodes, filtered_db):
    #data set in database: [[[1, 'm', 1, 0], ss, [x_uav, y_uav]]]
    #print "len = ", len(filtered_db)
    circle_list = [];
    if(len(filtered_db) > 2): #trilateration need exactly 3 points
        for i in filtered_db:  # filtered_db looks like [1,3,45] , it contain IDs of selected DB records
            radius = ss_to_dist((nodes[node_id].database[i][1]));
            #print radius
            x = nodes[node_id].database[i][2][0]; 
            y = nodes[node_id].database[i][2][1]; 
            p = point(x, y);
            c = circle(p, radius);
            circle_list.append(c);        
        #calculation start here                
        inner_points = []
        #print len(circle_list)
        for p in get_all_intersecting_points(circle_list):
            #print p.x, p.y
            if is_contained_in_circles(p, circle_list):
                #print p.x, p.y
                inner_points.append(p)   
        if len(inner_points)>0:
            center = get_polygon_center(inner_points)
        else:
            return 0;
        #print center.x, center.y
        return [center.x, center.y, 0]
    else:
        print "Trilateration was not applyed bcs it received more or less then 3 points, it needs only 3 points"
        return 0;    
    
  



        
# Find the intersection of three spheres                 
# P1,P2,P3 are the centers, r1,r2,r3 are the radii       
# Implementaton based on Wikipedia Trilateration article.    
# http://stackoverflow.com/questions/1406375/finding-intersection-points-between-3-spheres                          
def calculate_trilateration_3D_3points(P1,P2,P3,r1,r2,r3):                      
    temp1 = P2 - P1;                                        
    e_x = temp1 / norm(temp1);                              
    temp2 = P3 - P1;                                        
    i = dot(e_x, temp2);                                   
    temp3 = temp2 - i * e_x;                                
    e_y = temp3 / norm(temp3);                              
    e_z = cross(e_x, e_y);                                 
    d = norm(P2 - P1);                                      
    j = dot(e_y, temp2);                                   
    x = (r1*r1 - r2*r2 + d*d) / (2*d);                    
    y = (r1*r1 - r3*r3 - 2*i*x + i*i + j*j) / (2*j);       
    temp4 = r1*r1 - x*x - y*y;                            
    if temp4<0:                                          
        #print ("The three spheres do not intersect!");
        return 0;               
    z = sqrt(temp4);                                      
    p_12_a = P1 + x*e_x + y*e_y + z*e_z;                  
    p_12_b = P1 + x*e_x + y*e_y - z*e_z;   

    try :
        x_result =  (p_12_a[0]+p_12_b[0])/2; 
        y_result =  (p_12_a[1]+p_12_b[1])/2; 
        z_result =  (p_12_a[2]+p_12_b[2])/2; 
        return [x_result, y_result, z_result]
    except: 
        return 0

#trilateration method using 2 sphere intersection
def trilateration_3D_3points(node_id, nodes, filtered_db):
    start_time_conversion = time.clock();
    if(len(nodes[node_id].database)>2): #data set in database: [[[1, 'm', 1, 0], ss, [x_uav, y_uav]]]
        #--------------Data Collect and Sorting--------------#
        #columns are [x , y , z , d ]
        #matrix_dimensions = len(nodes[node_id].database);
        matrix_dimensions = len(filtered_db);
        data = np.zeros((4, matrix_dimensions), dtype = float); #it was (matrix_dim x matrix dim) not necessary
        #for i in range(len(nodes[node_id].database)):
        for i in range(len(filtered_db)):
            data[0, i] = nodes[node_id].database[i][2][0]; # x-coord
            data[1, i] = nodes[node_id].database[i][2][1]; # y-coord
            data[2, i] = nodes[node_id].database[i][2][2]; # z-coord
            data[3, i] = ss_to_dist(nodes[node_id].database[i][1]); #RSSI turned into radius
            #End of data matrix filling
    
        
        #------------Sorting with respect to RSSI-------------#
        #data_backup = data;

        datat = np.transpose(data) ;                                      #transposed data
        datas = datat[datat[:, 3].argsort()][::-1][:matrix_dimensions];   #sorted data to be transposed
        data = np.transpose(datas);                                         #sorted data in columns

    
        #----------------------------------------------------#
        x1 = data[0,0]; # X component of the highest RSSI valued anchor
        y1 = data[1,0]; # Y component of the highest RSSI valued anchor
        z1 = data[2,0]; # Z component of the highest RSSI valued anchor
        d1 = data[3,0]; # the highest RSSI valued anchor
        p1 = np.array([x1,y1,z1]);          
    
        x2 = data[0,1]; #2nd highest RSSI
        y2 = data[1,1];
        z2 = data[2,1];
        d2 = data[3,1];
        p2 = np.array([x2,y2,z2]);
        
        j = 2;
        x3 = data[0,j]; #3rd higest RSSI
        y3 = data[1,j];
        z3 = data[2,j];
        d3 = data[3,j];
        p3 = np.array([x3,y3,z3]);
        #print time.clock() - start_time_conversion, "before colinear"; 
        while collinear(p1, p2, p3)==True: #Check whether last 3 anchors are colinear
            j = j + 1;
            if j + 2 > len(filtered_db):
            #if j + 2 > len(nodes[node_id].database): #The case if they are notcolinear #allesgut
                #print "can not find 3 non collinear points, returning 0" 
                return 0
            else:
                x3 = data[0, j];                 #Increment the counter select lower RSSI anchor
                y3 = data[1, j];
                z3 = data[2, j];
                d3 = data[3, j];
                p3 = np.array([x3, y3, z3])
       # print time.clock() - start_time_conversion, " for data conversion";  
        return calculate_trilateration_3D_3points(p1,p2,p3,d1,d2,d3)


# from students, not working
def localization_3D_DCE(node_id, nodes, filtered_db):
    if(len(nodes[node_id].database)>3): #data set in database: [[[1, 'm', 1, 0], ss, [x_uav, y_uav]]]
        xa=nodes[node_id].database[0][2][0];
        ya=nodes[node_id].database[0][2][1];
        za=nodes[node_id].database[0][2][2];
        da=ss_to_dist(nodes[node_id].database[0][1])
        pa=np.array([xa,ya,za])

        xb = nodes[node_id].database[1][2][0];
        yb = nodes[node_id].database[1][2][1];
        zb = nodes[node_id].database[1][2][2];
        db = ss_to_dist(nodes[node_id].database[1][1])
        pb=np.array([xb,yb,zb])

        j=2
        xc = nodes[node_id].database[j][2][0];
        yc = nodes[node_id].database[j][2][1];
        zc = nodes[node_id].database[j][2][2];
        dc = ss_to_dist(nodes[node_id].database[j][1])
        pc=np.array([xc,yc,zc])

        while collinear(pa,pb,pc)==True:
            j=j+1
            if j+2>len(nodes[node_id].database):
                return 0;
            else:
                xc = nodes[node_id].database[j][2][0];
                yc = nodes[node_id].database[j][2][1];
                zc = nodes[node_id].database[j][2][2];
                dc = ss_to_dist(nodes[node_id].database[j][1])
                pc = np.array([xc, yc, zc])

        k=j+1
        xd = nodes[node_id].database[k][2][0];
        yd = nodes[node_id].database[k][2][1];
        zd = nodes[node_id].database[k][2][2];
        dd = ss_to_dist(nodes[node_id].database[k][1])
        pd=np.array([xd,yd,zd])
        while collinear(pa,pb,pd)==True or collinear(pa,pc,pd)==True or collinear(pb,pc,pd)==True:
            k=k+1
            if k+1>len(nodes[node_id].database):
                return 0;
            else:
                xd = nodes[node_id].database[k][2][0];
                yd = nodes[node_id].database[k][2][1];
                zd = nodes[node_id].database[k][2][2];
                dd = ss_to_dist(nodes[node_id].database[k][1])
                pd = np.array([xd, yd, zd])
        vectors=np.array([[da,db,dc,dd],[xa,xb,xc,xd],[ya,yb,yc,yd],[za,zb,zc,zd]]);
        s=np.array([da,db,dc,dd]);
        s.sort()
        t=0
        while vectors[0,t]!=s[0]:
            t=t+1
        d1,x1,y1,z1=vectors[:,t]
        vectors=scipy.delete(vectors,t,1)

        t=0
        while vectors[0,t]!=s[1] :
            t=t+1
        d2,x2,y2,z2=vectors[:,t]
        vectors=scipy.delete(vectors,t,1)
        t = 0
        while vectors[0, t] != s[2]:
            t = t + 1
        d3, x3, y3, z3 = vectors[:, t]
        vectors[:,t]=0
        t = 0
        while vectors[0, t] != s[3]:
            t = t + 1
        d4, x4, y4, z4 = vectors[:, t]

        p1 = np.array([x1, y1, z1])
        p2 = np.array([x2, y2, z2])
        p3 = np.array([x3, y3, z3])
        p4 = np.array([x4, y4, z4])

        delta2 = 0.01
        delta3 = d3 / d2 * delta2
        delta4 = d4 / d2 * delta2


        d_12 = np.linalg.norm(p2 - p1)
        d_13 = np.linalg.norm(p3 - p1)
        d_23 = np.linalg.norm(p3 - p2)

        # adjusment of d2
        while ((d2 + d1) < (d_12)):
            d2 = d2 + delta2
        while ((d2 - d1) > (d_12)):
            d2 = d2 - delta2

        # adjusment of d3
        R12 = 1 / (2 * d_12) * ((d_12 + d1 + d2) * (d_12 - d1 + d2) * (d_12 + d1 - d2) * (d1 + d2 - d_12)) ** 0.5
        d_10 = math.sqrt(d1 ** 2 - R12 ** 2)
        d_20 = math.sqrt(d2 ** 2 - R12 ** 2)
        if (d_20 > max(d_10, d_12)):
            p0 = p1 + d_10 / d_12 * (p1 - p2)
        else:
            p0 = p1 - d_10 / d_12 * (p1 - p2)
        d_30 = np.linalg.norm(p3 - p0)
        d_0E = 1 / (2 * d_12) * ((d_12 + d_13 + d_23) * (
            d_12 + d_13 - d_23) * (d_12 - d_13 + d_23) * (
        -d_12 + d_13 + d_23)) ** 0.50
        if d_30**2-d_0E**2<=0:
            return 0;
        d_3E = math.sqrt(d_30 ** 2 - d_0E ** 2)
        d_3A = (d_3E ** 2 + (d_0E - R12) ** 2) ** 0.5
        d_3B = (d_3E ** 2 + (d_0E + R12) ** 2) ** 0.5

        while (d3 < d_3A):
            d3 = d3 + delta3
        while (d3 > d_3B):
            d3 = d3 - delta3

        # adjusment of d4
        cos_theta = (d_3E ** 2 + d_0E ** 2 + R12 ** 2 - d3 ** 2) / (2 * R12 * d_0E)
        if np.abs(cos_theta)>1:
            return 0;
        sin_theta = math.sqrt(1 - cos_theta ** 2)
        n_pre = np.cross((p1 - p3), (p2 - p3)) / np.linalg.norm(np.cross((p1 - p3), (p2 - p3)))
        n_par = np.cross((p1 - p2), n_pre) / d_12
        q1 = (R12 * cos_theta) * n_par + (R12 * sin_theta) * n_pre + p0
        q2 = (R12 * cos_theta) * n_par - (R12 * sin_theta) * n_pre + p0
        p4_q1 = np.linalg.norm(p4 - q1)
        p4_q2 = np.linalg.norm(p4 - q2)

        while (d4 < min(p4_q1, p4_q2)):
            d4 = d4 + delta4
        while (d4 > max(p4_q1, p4_q2)):
            d4 = d4 - delta4

        x_n11 = (d1 ** 2 - d2 ** 2) - (x1 ** 2 - x2 ** 2) - (y1 ** 2 - y2 ** 2) - (z1 ** 2 - z2 ** 2);
        x_n21 = (d1 ** 2 - d3 ** 2) - (x1 ** 2 - x3 ** 2) - (y1 ** 2 - y3 ** 2) - (z1 ** 2 - z3 ** 2);
        x_n31 = (d1 ** 2 - d4 ** 2) - (x1 ** 2 - x4 ** 2) - (y1 ** 2 - y4 ** 2) - (z1 ** 2 - z4 ** 2);
        x_n12 = 2 * (y2 - y1);
        x_n22 = 2 * (y3 - y1);
        x_n32 = 2 * (y4 - y1);
        x_n13 = 2 * (z2 - z1);
        x_n23 = 2 * (z3 - z1);
        x_n33 = 2 * (z4 - z1);

        d11 = 2 * (x2 - x1);
        d21 = 2 * (x3 - x1);
        d31 = 2 * (x4 - x1);
        d12 = 2 * (y2 - y1);
        d22 = 2 * (y3 - y1);
        d32 = 2 * (y4 - y1);
        d13 = 2 * (z2 - z1);
        d23 = 2 * (z3 - z1);
        d33 = 2 * (z4 - z1);

        d = np.array([[d11, d12, d13], [d21, d22, d23], [d31, d32, d33]]);
        s=np.linalg.det(d)
        if np.abs(s) < 10**-6:
            s=0;
        if s==0:
            return 0;
        d_inv = np.linalg.inv(d);

        x_n = np.array([[x_n11, x_n12, x_n13], [x_n21, x_n22, x_n23], [x_n31, x_n32, x_n33]]);
        #x = np.dot(x_n, d_inv);
        x = np.linalg.det(x_n)/s;

        y_n11 = 2 * (x2 - x1);
        y_n21 = 2 * (x3 - x1);
        y_n31 = 2 * (x4 - x1);
        y_n12 = x_n11;
        y_n22 = x_n21;
        y_n32 = x_n31;
        y_n13 = 2 * (z2 - z1);
        y_n23 = 2 * (z3 - z1);
        y_n33 = 2 * (z4 - z1);

        y_n = np.array([[y_n11, y_n12, y_n13], [y_n21, y_n22, y_n23], [y_n31, y_n32, y_n33]])
        #y = np.dot(y_n, d_inv)
        y = np.linalg.det(y_n)/s;

        z_n11 = 2 * (x2 - x1);
        z_n21 = 2 * (x3 - x1);
        z_n31 = 2 * (x4 - x1);
        z_n12 = 2 * (y2 - y1);
        z_n22 = 2 * (y3 - y1);
        z_n32 = 2 * (y4 - y1);
        z_n13 = x_n11;
        z_n23 = x_n21;
        z_n33 = x_n31;

        z_n = np.array([[z_n11, z_n12, z_n13], [z_n21, z_n22, z_n23], [z_n31, z_n32, z_n33]]);
        #z = np.dot(z_n, d_inv);
        z = np.linalg.det(z_n)/s
        #loc=np.array([x,y,z])
        #if distance.euclidean(loc, nodes[node_id].x_y_zreal)>1000:
            #N
        return [x,y,z];
    else:
        return 0;
        
        
# from dtudents        
def localization_2D_DCE(node_id, nodes, filtered_db):
#    if(len(nodes[node_id].database)>2): #data set in database: [[[1, 'm', 1, 0], ss, [x_uav, y_uav]]]
    if(len(filtered_db)>2): #data set in database: [[[1, 'm', 1, 0], ss, [x_uav, y_uav]]]
        #ind = 0; # original was 0, but bcs filter we change it
        ind = filtered_db[0];
        xa=nodes[node_id].database[ind][2][0];
        ya=nodes[node_id].database[ind][2][1];
        da=ss_to_dist(nodes[node_id].database[ind][1])
        
        pa=np.array([xa,ya])
        ind = filtered_db[1];
        xb = nodes[node_id].database[ind][2][0];
        yb = nodes[node_id].database[ind][2][1];
        db = ss_to_dist(nodes[node_id].database[ind][1])
        pb= np.array([xb,yb])
        
        ind = filtered_db[2];
        xc = nodes[node_id].database[ind][2][0];
        yc = nodes[node_id].database[ind][2][1];
        dc = ss_to_dist(nodes[node_id].database[ind][1])
        pc=np.array([xc,yc])

        vectors=np.array([[da,db,dc],[xa,xb,xc],[ya,yb,yc]]);
        s=np.array([da,db,dc]);
        s.sort()
        t=0
        while vectors[0,t]!=s[0]:
            t=t+1
        d1,x1,y1=vectors[:,t]
        vectors=scipy.delete(vectors,t,1)

        t=0
        while vectors[0,t]!=s[1] :
            t=t+1
        d2,x2,y2=vectors[:,t]
        vectors=scipy.delete(vectors,t,1)
        t = 0
        while vectors[0, t] != s[2]:
            t = t + 1
        d3, x3, y3= vectors[:, t]
        vectors[:,t]=0
        t = 0

        p1 = np.array([x1, y1])
        p2 = np.array([x2, y2])
        p3 = np.array([x3, y3])

        delta2 = 0.001
        delta3 = d3 / d2 * delta2


        d_12 = np.linalg.norm(p2 - p1)
        d_13 = np.linalg.norm(p3 - p1)
        d_23 = np.linalg.norm(p3 - p2)


        # adjusment of d2
        while ((d2 + d1) < (d_12)):
            d2 = d2 + delta2
        while ((d2 - d1) > (d_12)):
            d2 = d2 - delta2

        # adjusment of d3
        if x1>x2:
            costheta=(d2**2+d_12**2-d1**2)/float(2*d2*d_12)
            sintheta=np.sqrt(1-costheta**2)
            u=(p1-p2)/float(d_12)
            xu1=1
            yu1=-u[0]/float(u[1])
            u1=np.array([xu1,yu1])
            u1=u1/float(np.linalg.norm(u1))
            I1=p2+u*d2*costheta+u1*d2*sintheta
            I2=p2+u*d2*costheta-u1*d2*sintheta
        else:
            costheta = (d1 ** 2 + d_12 ** 2 - d2 ** 2) / float(2 * d1 * d_12)
            sintheta = np.sqrt(1 - costheta ** 2)
            u = (p2 - p1) / float(d_12)
            xu1 = 1
            yu1 = -u[0] / float(u[1])
            u1 = np.array([xu1, yu1])
            u1 = u1 / float(np.linalg.norm(u1))
            I1 = p1 + u * d1 * costheta + u1 * d1 * sintheta
            I2 = p1 + u * d1 * costheta - u1 * d1 * sintheta
        p3_I1 = np.linalg.norm(p3 - I1)
        p3_I2 = np.linalg.norm(p3 - I2)
        while (d3 < min(p3_I1, p3_I2)):
            d3 = d3 + delta3
        while (d3 > max(p3_I1, p3_I2)):
            d3 = d3 - delta3

        x_n11 = (d1**2 - d2**2) - (x1**2 - x2**2) - (y1**2 - y2**2); #alpha
        x_n21 = (d1**2 - d3**2) - (x1**2 - x3**2) - (y1**2 - y3**2); #Beta
        x_n12 = 2 * (y2 - y1);
        x_n22 = 2 * (y3 - y1);
        d11 = 2 * (x2 - x1);
        d21 = 2 * (x3 - x1);
        d12 = 2 * (y2 - y1);
        d22 = 2 * (y3 - y1);
        x_n = np.array([[x_n11, x_n12],[x_n21, x_n22]])
        d =np.array([[d11, d12],[d21, d22]])
        s=np.linalg.det(d);
        if np.abs(s)<10**-2:
            return 0;
        d_inv = np.linalg.inv(d);
        #x = np.dot(x_n, d_inv);
        x = np.linalg.det(x_n)/s
        y_n11 = d11
        y_n21 = d21
        y_n12 = x_n11
        y_n22 = x_n21
        y_n = np.array([[y_n11, y_n12],[y_n21, y_n22]])
        #y = np.dot(y_n, d_inv);
        y = np.linalg.det(y_n)/s;
        z=0

        return [x,y,z];
    else:
        return 0;        
        
#algo from students using matrixes  , it is not working      
# also it was called "localization_Tilateration_sorted"
def trilateration_3D_students(node_id, nodes, filtered_db):
    if(len(nodes[node_id].database)>3): #data set in database: [[[1, 'm', 1, 0], ss, [x_uav, y_uav]]]
        #--------------Data Collect and Sorting--------------#
        #columns are [x , y , z , d ]
        matrix_dimensions=len(nodes[node_id].database)
        data=np.zeros((4,matrix_dimensions),dtype=float) #it was (matrix_dim x matrix dim) not necessary
        for i in range(len(nodes[node_id].database)):
            data[0, i] = nodes[node_id].database[i][2][0] # x-coord
            data[1, i] = nodes[node_id].database[i][2][1] # y-coord
            data[2, i] = nodes[node_id].database[i][2][2] # z-coord
            data[3, i] = ss_to_dist(nodes[node_id].database[i][1]) #RSSI turned into radius
            #End of data matrix filling


        #------------Sorting with respect to RSSI-------------#
        data_backup=data
        datat=np.transpose(data)                                       #transposed data
        datas=datat[datat[:, 3].argsort()][::-1][:matrix_dimensions]   #sorted data to be transposed
        data=np.transpose(datas)                                       #sorted data in columns


        #----------------------------------------------------#
        x1=data[0,0] # X component of the highest RSSI valued anchor
        y1=data[1,0] # Y component of the highest RSSI valued anchor
        z1=data[2,0] # Z component of the highest RSSI valued anchor
        d1=data[3,0] # the highest RSSI valued anchor
        p1=np.array([x1,y1,z1])

        x2 = data[0,1] #2nd highest RSSI
        y2 = data[1,1]
        z2 = data[2,1]
        d2 = data[3,1]
        p2=np.array([x2,y2,z2])

        j=2
        x3 = data[0,j] #3rd higest RSSI
        y3 = data[1,j]
        z3 = data[2,j]
        d3 = data[3,j]
        p3=np.array([x3,y3,z3])

        while collinear(p1,p2,p3)==True: #Check whether last 3 anchors are colinear
            j=j+1
            if j+2>len(nodes[node_id].database): #The case if they are notcolinear #allesgut
                return 0
            else:
                x3 = data[0, j]                 #Increment the counter select lower RSSI anchor
                y3 = data[1, j]
                z3 = data[2, j]
                d3 = data[3, j]
                p3 = np.array([x3, y3, z3])

        k=j+1
        x4 = data[0,k]
        y4 = data[1,k]
        z4 = data[2,k]
        d4 = data[3,k]
        p4=np.array([x4,y4,z4])
        while collinear(p1,p2,p4)==True or collinear(p1,p3,p4)==True or collinear(p2,p3,p4)==True: #checking if 4 anchor points are colinear
            k+=1
            if k+1>len(nodes[node_id].database):
                return 0
            else:
                x4 = data[0, k]
                y4 = data[1, k]
                z4 = data[2, k]
                d4 = data[3, k]
                p4 = np.array([x4, y4, z4])



        #Trilateration calculations......
        x_n11 = (d1 ** 2 - d2 ** 2) - (x1 ** 2 - x2 ** 2) - (y1 ** 2 - y2 ** 2) - (z1 ** 2 - z2 ** 2)
        x_n21 = (d1 ** 2 - d3 ** 2) - (x1 ** 2 - x3 ** 2) - (y1 ** 2 - y3 ** 2) - (z1 ** 2 - z3 ** 2)
        x_n31 = (d1 ** 2 - d4 ** 2) - (x1 ** 2 - x4 ** 2) - (y1 ** 2 - y4 ** 2) - (z1 ** 2 - z4 ** 2)
        x_n12 = 2 * (y2 - y1)
        x_n22 = 2 * (y3 - y1)
        x_n32 = 2 * (y4 - y1)
        x_n13 = 2 * (z2 - z1)
        x_n23 = 2 * (z3 - z1)
        x_n33 = 2 * (z4 - z1)

        d11 = 2 * (x2 - x1)
        d21 = 2 * (x3 - x1)
        d31 = 2 * (x4 - x1)
        d12 = 2 * (y2 - y1)
        d22 = 2 * (y3 - y1)
        d32 = 2 * (y4 - y1)
        d13 = 2 * (z2 - z1)
        d23 = 2 * (z3 - z1)
        d33 = 2 * (z4 - z1)

        d = np.array([[d11, d12, d13], [d21, d22, d23], [d31, d32, d33]])
        s=np.linalg.det(d);
        if np.abs(s)<10**-4:
            s=0;
        if s==0:
            return 0;

        d_inv = np.linalg.inv(d)

        x_n = np.array([[x_n11, x_n12, x_n13], [x_n21, x_n22, x_n23], [x_n31, x_n32, x_n33]])
        x = np.dot(x_n, d_inv)
        x = np.linalg.det(x)

        y_n11 = 2 * (x2 - x1)
        y_n21 = 2 * (x3 - x1)
        y_n31 = 2 * (x4 - x1)
        y_n12 = x_n11
        y_n22 = x_n21
        y_n32 = x_n31
        y_n13 = 2 * (z2 - z1)
        y_n23 = 2 * (z3 - z1)
        y_n33 = 2 * (z4 - z1)

        y_n = np.array([[y_n11, y_n12, y_n13], [y_n21, y_n22, y_n23], [y_n31, y_n32, y_n33]])
        y = np.dot(y_n, d_inv)
        y = np.linalg.det(y)

        z_n11 = 2 * (x2 - x1)
        z_n21 = 2 * (x3 - x1)
        z_n31 = 2 * (x4 - x1)
        z_n12 = 2 * (y2 - y1)
        z_n22 = 2 * (y3 - y1)
        z_n32 = 2 * (y4 - y1)
        z_n13 = x_n11
        z_n23 = x_n21
        z_n33 = x_n31

        z_n = np.array([[z_n11, z_n12, z_n13], [z_n21, z_n22, z_n23], [z_n31, z_n32, z_n33]])
        z= np.dot(z_n, d_inv)
        z = np.linalg.det(z)

        return [x,y,z];
    else:
        return 0;

#from students
def localization_minmaxshrink(node_id, nodes, filtered_db): # centroid localization
    if(len(filtered_db)>2): #data set in database: [[[1, 'm', 1, 0], ss, [x_uav, y_uav]]]
        b_square_coords = np.zeros(2)   # current iteration's square matrix
        zzz = (2, 3)  # (2,3) for 3-dimension
        c_square_coords = np.zeros(zzz)  # ----> Results
        # [max(x-d) / max (y-d)]
        # [min(x+d) / min (y+d)]
        for i in filtered_db:
        #for i in range(len(nodes[node_id].database)):
            #       Initialize Square; Center Points are UAV or Anchors
            dist = ss_to_dist(nodes[node_id].database[i][1]);
            eps=(1-(1/np.sqrt(2)))*dist
            center_x = nodes[node_id].database[i][2][0] + 100;  # 100 is for the take origin bottom left
            center_y = nodes[node_id].database[i][2][1] + 100;
            center_z = nodes[node_id].database[i][2][2] + 100;
            b_square_coords = np.array([center_x, center_y, center_z])  # Origin has moved to left-bottom and Coords assigned
            # ----------Initialize---------#
            zeros = np.zeros(zzz)
            if c_square_coords.all() == zeros.all():  # for first iteration
                c_square_coords[0, 0] = b_square_coords[0] - dist  # x-dist
                c_square_coords[0, 1] = b_square_coords[1] - dist  # y-dist
                c_square_coords[0, 2] = b_square_coords[2] - dist  # z-dist
                c_square_coords[1, 0] = b_square_coords[0] + dist  # x+dist
                c_square_coords[1, 1] = b_square_coords[1] + dist  # y+dist
                c_square_coords[1, 2] = b_square_coords[2] + dist  # z+dist

            else:  # Each iteration will be here except the 1st one
                # -----Here Comparisons of MIN & MAX-------  #

                if c_square_coords[0, 0] > b_square_coords[0] - dist:  # max(x-d) into [xxx / ... / ...]
                    c_square_coords[0, 0]  # has won "c_square_coords[0, 0] != 0 and"  [... / ... / ...]                                                           # [ .../ ...]
                else:
                    c_square_coords[0, 0] = b_square_coords[0] - dist + eps # b_x returns

                if c_square_coords[0, 1] > b_square_coords[1] - dist:  # max(y-d) into [... / xxx / ...]
                    c_square_coords[0, 1]  # has won                                   [ .../ ... / ...]
                else:
                    c_square_coords[0, 1] = b_square_coords[1] - dist + eps # b_y returns

                if c_square_coords[0, 2] > b_square_coords[2] - dist:  # max(z-d) into [... / ... / xxx]
                    c_square_coords[0, 2]                              #               [ .../ ... / ...]
                else:
                    c_square_coords[0, 2] = b_square_coords[2] - dist + eps

                if c_square_coords[0, 1] < b_square_coords[0] + dist:  # min(x+d) into [... / ... / ...]
                    c_square_coords[1, 0]                              #               [ xxx/ ... / ...]
                else:
                    c_square_coords[1, 0] = b_square_coords[0] + dist - eps

                if c_square_coords[1, 1] < b_square_coords[1] + dist:  # min(y+d) into [... / ... / ...]
                    c_square_coords[1, 1]                              #               [ .../ xxx/ ....]
                else:
                    c_square_coords[1, 1] = b_square_coords[1] + dist - eps

                if c_square_coords[1,2] < b_square_coords[2] + dist:  # min(z+d) into [... / ... / ...]
                    c_square_coords[1, 2]                             #               [ .../ ... / xxx]
                else:
                    c_square_coords[1, 2] = b_square_coords[2] + dist - eps





                    # ---End of finding intersection square
        ##
        res_x = ((c_square_coords[0, 0] + c_square_coords[1, 0]) / 2) - 100. #origin located back with "-100"
        res_y = ((c_square_coords[0, 1] + c_square_coords[1, 1]) / 2) - 100.
        res_z = ((c_square_coords[0, 2] + c_square_coords[1, 2]) / 2) - 100.

        return [res_x,res_y,res_z];
    else:
        return 0;

    
    
#        case 'multilateration': #for wireless sensor network
#        #according to http://www-cs-students.stanford.edu/~dbfaria/files/faria-TR-KP06-0118.pdf
#        #we become the following path loss model: Pr(d)=Pr0 - 10aLog(d) - W - Xa
#        #where Pr(d)-received SS; Pr0- SS at 1m; a=3.32; d- distance; W-wall attanuation-4.8dBm; Xa=3.1dBm
#        #propModel = PropagationModel(-44.8, 3.32, 1)
#        ss = (-44.8) - (10*3.32)*log(dist,10);
#        return ss;
#        break;
#        case 'free_space': #for wireless sensor network
#        #according to http://www-cs-students.stanford.edu/~dbfaria/files/faria-TR-KP06-0118.pdf
#        #we become the following path loss model: Pr(d)=Pr0 - 10aLog(d) - W - Xa
#        #where Pr(d)-received SS; Pr0- SS at 1m; a=3.32; d- distance; W-wall attanuation-4.8dBm; Xa=3.1dBm
#        #propModel = PropagationModel(-44.8, 3.32, 1)
#        ss = (-40) - (10*3.32)*log(dist,10) + gauss_ms(0.0,3.1);
#        return ss;
#        break;	
def multilateration_calculate(anchors, mode = 1, use3D = False):   
    #propModel = PropagationModel(-30, 3, 1); # was default path loss model from alex (simulations 2011-2013)     
    propModel = PropagationModel(-44.8, 3.32, 1); #this fit it return almost reference values 
    #0.700133281739     #-39.66  expected 0.70
    #0.510023704576     #-35.092 expected  0.51
    #0.630082750607     #-38.14 expected 0.63
    ML1 = Multilateration(2, propModel, minRssi=-130, windowSize = np.inf);
    for j in range (0, len(anchors)):                        
        dataSet=[];
        dataSet = [ 1, [float(anchors[j][0]), float(anchors[j][1])], float(anchors[j][3]) ] #3D ready, ss moved from 2 to 3, z now in 2 
        ML1 += dataSet;

    if (len(ML1) >= 3) and (ML1.changed):
        #print "len ML1", len(ML1);
        #print ML1.estimatePosition().T 
        result_mult = ML1.estimatePosition().T; 
        http_response = str(result_mult);
        http_response = re.sub('\[\[\s+\]\]', ' ', http_response);
        http_response = re.sub('\s+', ' ', http_response);
    
        part1 = http_response.split("[[");   
        part2 = part1[1].split("]]");           
        part2[0] = re.sub('\[\[\s+\]\]', ' ', part2[0]);
    
        if(part2[0][0] == ' '):
            result = part2[0][1:].split(" ");    
        else:
            result = part2[0].split(" ");
    
        result_x = float(result[0]);
        result_y = float(result[1]);

        return [result_x, result_y, 0] # z will be always 0
    else:
        return 0;

# multilateration localization, will prepare data and call method for calclation    
def multilateration_localization(node_id, nodes, filtered_db):
    #data set in database: [[[1, 'm', 1, 0], ss, [x_uav, y_uav]]]
    if(len(filtered_db)>2): #multilateration need > 2 points
        AnchorArray = [];
        for i in filtered_db:  # filtered_db looks like [1,3,45] , it contain IDs of selected DB records
            ss = nodes[node_id].database[i][1];
            x = nodes[node_id].database[i][2][0]; 
            y = nodes[node_id].database[i][2][1]; 
            z = 0;
            AnchorArray.append([x, y, z, ss]);          
                              
        #calculation start here                
        result = multilateration_calculate(AnchorArray, 1)       
        return result
    else:
        #print "Trilateration was not applyed bcs it received more or less then 3 points, it needs only 3 points"
        return 0;   
    
# main localization, will use localisation_method and localisation_filtering to select which localization exectly to be used
        #localisation_method = 0 --> centroid localisation (default)
        #localisation_method = 1 --> trilateration localisation 
        #localisation_method = 2 --> multilateration localisation
        #localisation_method = 3 --> weighted centroid localisation
        #localisation_method = 4 --> trilateration 3D from students
        #localisation_method = 5 --> trilateration 3d 3 points //  takes 3 highest RSSI, not colinear
        #localisation_method = 6-->  localization_2D_DCE /// takes 3 first dots in filtered_db
        #localisation_method = 7-->  localization_minmaxshrink  
        #------------------------------------------------------
        #localisation_filtering = 0 --> no filtering (default)
        #localisation_filtering = 1 --> SS filtering
        #localisation_filtering = 2 --> LastThree filtering 
        #localisation_filtering = 3 --> JointClustering filtering 
        #localisation_filtering = 4 --> Perimeter filtering , return 4 dots


def localization(node_id, nodes, localisation_method = 0, localisation_filtering = 0, report = 0): 
    #start_time = time.clock();
    if report == 0: # we like to localize node
        enough_points = len(nodes[node_id].database)>2;
    else: # we like only report method and filter
        enough_points = True;
    if(enough_points or report > 0): #data set in database: [[[1, 'm', 1, 0], ss, [x_uav, y_uav]]]
        x = 0;
        y = 0;
        z = 0 ;   
        #print "localisation_filtering", localisation_filtering, "localisation_method", localisation_method; 
        #-----------------------------------------------applying filtering on node db
        #-----------------------------------------------data will be keept in filtered_db like [1,2,3] where 
        #-----------------------------------------------where 1,2,3 is number of reading N in database
        
        if localisation_filtering == 0:
            if report > 0:
                filter_type = "none";
            else:
                filtered_db = range(len(nodes[node_id].database)); #all nodes from db are included
        if localisation_filtering == 1:
            if report > 0:
                filter_type = "SS_Filter";
            else:
                filtered_db = SS_Filter(nodes[node_id].database); 
        if localisation_filtering == 2:
            if report > 0:
                filter_type = "LastThree_Filter";
            else:
                filtered_db = LastThree_Filter(nodes[node_id].database);   
        if localisation_filtering == 3:
            if report > 0:
                filter_type = "JointClustering_Filter";
            else:
                filtered_db = JointClustering_Filter(nodes[node_id].database); 
        if localisation_filtering == 4:
            if report > 0:
                filter_type = "Perimeter_Filter";
            else:
                filtered_db = Perimeter_Filter(nodes[node_id].database); 
        if localisation_filtering == 5:
            if report > 0:
                filter_type = "JointClustering_Filter_Centroid";
            else:
                filtered_db = JointClustering_Filter_Centroid(nodes[node_id].database);
           # print filtered_db;
       # print time.clock() - start_time, "seconds for filter"
        #------------------------------------------------applying localisation on filtered_db 
        if localisation_method == 0:
            if report > 0:
                loc_type = "centroid_localisation";
            else:
                loc_res = centroid_localisation(node_id, nodes, filtered_db);  #ok
                
        if localisation_method == 1:
            if report > 0:
                loc_type = "trilateration_localization";
            else:
                loc_res = trilateration_localization(node_id, nodes, filtered_db);   
                
        if localisation_method == 2:
            if report > 0:
                loc_type = "multilateration_localization";
            else:
                loc_res = multilateration_localization(node_id, nodes, filtered_db); 
                
        if localisation_method == 3:
            if report > 0:
                loc_type = "weight_centroid_localisation";
            else:
                loc_res = weight_centroid_localisation(node_id, nodes, filtered_db);
                
        if localisation_method == 4: # not working 
            if report > 0:
                loc_type = "trilateration_3D_students_not_working";
            else:
                loc_res = trilateration_3D_students(node_id, nodes, filtered_db);
                
        if localisation_method == 5:
            if report > 0:
                loc_type = "trilateration_3D_3points";
            else:
                loc_res = trilateration_3D_3points(node_id, nodes, filtered_db);
                 
        
        if localisation_method == 6:
            if report > 0:
                loc_type = "localization_2D_DCE";
            else:
                loc_res = localization_2D_DCE(node_id, nodes, filtered_db);     
                
        if localisation_method == 7:
            if report > 0:
                loc_type = "localization_minmaxshrink";
            else:
                loc_res = localization_minmaxshrink(node_id, nodes, filtered_db);  
        
        if localisation_method == 8:
            if report > 0:
                loc_type = "Least Squares";
            else:
                loc_res = Least_Squares(node_id, nodes, filtered_db);                  
        #-------------------------------------------------return localization result   
        #print time.clock() - start_time, "seconds for localisation"
                
        if report > 0:
            return loc_type, filter_type;
        if (loc_res!= 0): # [x,y,z] were returned from localization procedure
            x,y,z  = loc_res;
            #print "node", node_id," was localized here: ", loc_res, "real pos:", nodes[node_id].x_y_zreal;
            return [x, y, z];
        else:
            return 0;
    else: #if less then 3 measurments
        return 0;                    
        

if __name__ == '__main__':
    #conclusions = main()
    #data set in database: [[[1, 'm', 1, 0], ss, [x_uav, y_uav]]]
#------------------------------------------------this part is only for tests--------------------------------------------- 
    nodes = []
    one_nodes_database = [];
    p1 = point(0.81, 1.2)
    p2 = point(1.21, 0.69)
    p3 = point(0.87, 0.84)
    a = mNode(0, 1, 1, 1);
    a.database.append([1, -39.66, [0.81, 1.2, 0]]);
    a.database.append([1, -35.092, [1.21, 0.69, 0]]);
    a.database.append([1, -38.14, [0.87, 0.84, 0]]);      
    a.database.append([1, -35.092, [1.21, 0.69, 0]]);
    b = mNode(0, 1, 1, 1);
    c = mNode(0, 1, 1, 1);
    d = mNode(0, 1, 1, 1);
#    print ss_to_dist(-20)
#    print ss_to_dist(-30)
#    print ss_to_dist(-40)
#    print ss_to_dist(-50)
#    print ss_to_dist(-60)
#    print ss_to_dist(-70)
#    print ss_to_dist(-88)
#    print ss_to_dist(-101.3)
#    print ss_to_dist(-102.5)
#    print ss_to_dist(-105)
    b.database.append([1, -102.5, [50, 50, 50]]);
    b.database.append([1, -102.5, [0, 0, 50]]);
    b.database.append([1, -102.5, [50, 50, 0]]);    
    b.database.append([1, -102.5, [0, 0, 0]]); 
                     
    c.database.append([1, -88, [0, 0, 0]]);
    c.database.append([1, -88, [-30, -20, 0]]);
    c.database.append([1, -88, [-35, 10, 0]]);   
                     
    d.database.append([1, -101.25, [30, 0, 0]]);
    d.database.append([1, -88, [-30, -20, 0]]);
    d.database.append([1, -88, [-35, 10, 0]]); 
    d.database.append([1, -60, [-30, 10, 0]]); 
       
    nodes.append(d);
    #print nodes[0]
              
    #print SS_Filter(one_nodes_database);
    #print LastThree_Filter(one_nodes_database);                         
    print "centroid", centroid_localisation(0, nodes, [0,1,2]);     #[0.9633333333333334, 0.91, 0]  correct
    print "weighted centroid", weight_centroid_localisation(0, nodes, [0,1,2]);    
    print "trilateration", trilateration_localization(0, nodes, [0,1,2]);     # [1.0746879774866145,0.8306638880320697]  correct
    print "multilateration", multilateration_localization(0, nodes, [0,1,2]);     # [1.0746879774866145,0.8306638880320697]  correct
    print "trilateration 3d students", trilateration_3D_students(0, nodes, [0,1,2,3]); 
    print "trilateration 3d", trilateration_3D_3points(0, nodes, [0,1,2]);               
    #print "trilateration 3d dynamik", trilateration_dynamic_localization(0, nodes, [0,1,2,3]); # not working
    print "localization_2D_DCE", localization_2D_DCE(0, nodes, [0,1,2,3]);
    print "localization_3D_DCE", localization_3D_DCE(0, nodes, [0,1,2,3]);  
    print "localization_minmaxshrink", localization_minmaxshrink(0, nodes, [0,1,2]);      
                                                