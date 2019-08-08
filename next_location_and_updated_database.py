from scipy.spatial import distance
import numpy as np

''''


'''

def next_location_and_updated_database(given_location,data_base):

    #distance_points=distance.euclidean(given_location , data_base )
    distance_points = distance.cdist(given_location , data_base , 'euclidean') #computes distance between given point and available square locations
    distance_points=distance_points.tolist()
    
    index_min = np.argmin(distance_points)
    index_min= index_min.tolist()                 # gives index of nearest square  
    
    distance_points=distance_points[0]
    
    #print data_base,"before update"
    
    next_location=data_base.pop(index_min)    #returns nearest square location ,and update database--using pop operation  
    
    #print next_location,"next location"
    #print data_base," after update"
    
    return next_location,data_base
  

#total=[[2,0,0],[3,0,0],[1,0,0],[4,0,0],[5,0,0],[6,0,0]]
#initial=[[0,0,0]]
#print next_location_and_updated_database(initial,total)    
