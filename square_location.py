
import math


def square_location(max_x,max_y,max_z,step_size):
    step_size=  int(step_size)
    
    p=( (max_x*2) / step_size ) 
 
    q=( (max_y*2) / step_size ) 

    result=[]

    for j in range(q):
        for i in range(p):
            if(j==0 or j%2==0):
                if( result==[] or ( result[-1][0] <=  ( max_x-(step_size/2) )  and result[-1][1] <= (max_y- (step_size/2)) ) ):
                    result.append ( [-max_x + (step_size/2) +(i*step_size) , -max_y + (step_size/2) + (j*step_size) ,0 ]) # horizontal right  
            else:     
                if( result==[] or ( result[-1][0] >=  ( -max_x+(step_size/2) )  and result[-1][1] <= (max_y- (step_size/2)) ) ):
                    result.append ( [max_x - (step_size/2) - (i*step_size) , -max_y + (step_size/2) + (j*step_size), 0 ]) # horizontal left
    return result

#g=square_location(200,200,0,30)
#print g 
#print len(g)             
