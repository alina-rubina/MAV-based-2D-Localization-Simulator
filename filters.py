

from utils import *
from  BeanSphere import *


import numpy as np
import functools


def init_distanceArray(x , y): 
    fin_arr = []
    tmp_arr = []
    for i in range (0, x):
        tmp_arr = []
        tmp_arr = [-1] * x
        fin_arr.append(tmp_arr)
    return fin_arr 
    
def rotate(A,B,C):
    return (B[1]-A[1])*(C[2]-B[2])-(B[2]-A[2])*(C[1]-B[1])
    
#building fence around set of dots
#https://habrahabr.ru/post/144921/
def Grahamscan(Arr):
    n = len(Arr) # N dots
    P = range(n) # list of cells because we need them sorted
    A = []
    for i in Arr:
        A.append(i[2])
    for i in range(1, n):
        #print  A[P[i]][1],  A[P[i]]
        if A[P[i]][1] < A[P[0]][1]: # if P[i] dot is left from P[0]
            P[i], P[0] = P[0], P[i] # chnging them
    for i in range(2, n): # sorting by insertion
        j = i
        while j>1 and (rotate(A[P[0]],A[P[j-1]],A[P[j]])<0): 
            P[j], P[j-1] = P[j-1], P[j]
            j -= 1
    S = [P[0],P[1]] # stack
    for i in range(2,n):
        while rotate(A[S[-2]],A[S[-1]],A[P[i]])<0:
            del S[-1] # pop(S)
        S.append(P[i]) # push(S,P[i])
    return S

# take 4 dots with max distances    
def Perimeter_Filter(one_nodes_database):
    dotArray_coming = one_nodes_database;

    m = 4; # 4 dots needed for 3D, 3 for 2D
    #print "coming array has "+ str (len(dotArray_coming)) + "points";
    # searching for dots on boarder, they will make max perimeter and area
    BoarderDots = Grahamscan(dotArray_coming);
    n = len(BoarderDots); # number of dots
    if n < m:
        return BoarderDots;
    dotArray = [];
    for i in range(0, n):# taking in account only border points
        dotArray.append(dotArray_coming[BoarderDots[i]][2]) 
    #print "Grahamscan array has "+ str(n)   + "points";
                                      
    if n > 0 :
        n_y = n#  #for initiation of empty array    
    delta = (n - m + 1);
    #print "delta", delta
    # Calculate distances between each combination of anchors
    maxDistance=0.0;
    distanceArray = init_distanceArray(n, n_y);
    
    for i in range (0, delta): 
        
        for j in range (i+1, delta+1):
            
            if((distanceArray[i][j]) > -1):
                ij = distanceArray[i][j];
            else : 
                ij = float(getHaversineDistance(dotArray[i][1], dotArray[i][2],  dotArray[j][1], dotArray[j][2]));                    
                distanceArray[i][j] = ij;
                distanceArray[j][i] = ij;
            
            for k in range (j+1, delta+2): 
               
                if( (distanceArray[j][k])  > -1):
                    jk = distanceArray[j][k];
                else :
                    jk =float( getHaversineDistance(dotArray[j][1], dotArray[j][2],dotArray[k][1], dotArray[k][2]));                       
                    distanceArray[j][k]=jk;
                    distanceArray[k][j]=jk;
                
                for m in range (k+1, delta+3):
                        
                    if( (distanceArray[i][m]) > -1):
                        im = distanceArray[i][m];
                    else :
                        im = float(getHaversineDistance(dotArray[i][1], dotArray[i][2], dotArray[m][1], dotArray[m][2]));                        
                        distanceArray[i][m]=im;
                        distanceArray[m][i]=im;
                        
                        
                    if( (distanceArray[k][m]) > -1):
                        km = distanceArray[k][m];
                    else :
                        km = float(getHaversineDistance(dotArray[k][1], dotArray[k][2], dotArray[m][1], dotArray[m][2]));                        
                        distanceArray[k][m]=km;
                        distanceArray[m][k]=km;
                        
                        
                    #searching max perimeter 
                    distance = ij + jk + km + im;
                
                    if(distance > maxDistance):
                        maxDistance = distance;
                        element_i=i;
                        element_j=j;
                        element_k=k;
                        element_m=m;


    if maxDistance>0:
        resultArray = [];
        resultArray.append(element_i);
        resultArray.append(element_j);
        resultArray.append(element_k);
        resultArray.append(element_m);
        #print "resultArray", resultArray 
    
        return resultArray;  	
    else: # some problem with finding 4 dots on perimeter happen, so return all of them
        return BoarderDots;

#attempt to select only points with same SS for centroid localizatin
def JointClustering_Filter_Centroid(one_nodes_database,coming_treshould=0.3):
    #data set in database: [[[1, 'm', 1, 0], ss, [x_uav, y_uav]]]

    treshould = coming_treshould;
    if treshould > 1:
        return range(0, len(one_nodes_database)-1);

    ss_list = [x[1] for x in one_nodes_database];
    uniq_ss_list = list(set(ss_list));  # this will remove dublicates from ss list
    top_1_ss = sorted(np.asarray(uniq_ss_list))[-1:];  #this will select top3 SS values
    result_list = [];
    for i in range(len(ss_list)):  #add ID to result list if value in top_3_ss
        if abs(ss_list[i]- top_1_ss[0]) < treshould:         
            result_list.append(i);
    if len(result_list)>2:
        return result_list
    top_2_ss = sorted(np.asarray(uniq_ss_list))[-2:-1];     
    result_list = [];
    for i in range(len(ss_list)):  #add ID to result list if value in top_3_ss
        if abs(ss_list[i]- top_2_ss[0]) < treshould:         
            result_list.append(i);
    if len(result_list)>2:
        return result_list  
    if len(sorted(np.asarray(uniq_ss_list)))>2:
        top_3_ss = sorted(np.asarray(uniq_ss_list))[-3:-2];     
        result_list = [];
        for i in range(len(ss_list)):  #add ID to result list if value in top_3_ss
            if abs(abs(ss_list[i])- abs(top_3_ss[0])) < treshould:         
                result_list.append(i);
        if len(result_list)>2:
            return result_list                                        
    result = JointClustering_Filter_Centroid(one_nodes_database, treshould + 0.3);
    return result                               
         

def JointClustering_Filter(one_nodes_database):
    #data set in database: [[[1, 'm', 1, 0], ss, [x_uav, y_uav]]]
    ss_list = [x[1] for x in one_nodes_database];
    uniq_ss_list = list(set(ss_list));  # this will remove dublicates from ss list
    top_3_ss = sorted(np.asarray(uniq_ss_list))[-3:];  #this will select top3 SS values
    result_list = [];
    for i in range(len(ss_list)):  #add ID to result list if value in top_3_ss
        if ss_list[i] in top_3_ss:         
            result_list.append(i);
    #print uniq_ss_list;
    #print top_3_ss;
    #print result_list;
    return result_list;			

def LastThree_Filter(one_nodes_database):
    #data set in database: [[[1, 'm', 1, 0], ss, [x_uav, y_uav]]]
    max_ss = 5; # how many ss we should take
    ss_length = len(one_nodes_database);
    if ss_length > max_ss:
        ss_length = max_ss;
    id_list = range(len(one_nodes_database));  # all IDs are included
    result_list = id_list[-ss_length:]; #last 3 ID selected
    #print result_list
    return result_list;			

def SS_Filter(one_nodes_database):
    #data set in database: [[[1, 'm', 1, 0], ss, [x_uav, y_uav]]]
    max_ss = 6; # how many ss we should take
    ss_list = [x[1] for x in one_nodes_database]; #selecting only ss from incoming array
    ss_length = len(ss_list);
    if ss_length > max_ss:
        ss_length = max_ss;
    result_list = sorted(range(len(ss_list)), key=lambda i: ss_list[i])[-ss_length:];  # take ID of top 3 elements in sorted list
    #print result_list
    return result_list;
    
if __name__ == '__main__':
    #---------------------------------------------------------this is only for test purpose
    #conclusions = main()
    one_nodes_database = [];
    one_nodes_database.append([1,1,4]);
    one_nodes_database.append([1,1,4]);
    one_nodes_database.append([1,8,4]);
    one_nodes_database.append([1,7,4]);
    one_nodes_database.append([1,4,4]);
    one_nodes_database.append([1,4,4]);
    one_nodes_database.append([1,7,4]);
    one_nodes_database.append([1,4,4]);

    #print SS_Filter(one_nodes_database);
    #print LastThree_Filter(one_nodes_database);                         
    print JointClustering_Filter(one_nodes_database);                         
    
    