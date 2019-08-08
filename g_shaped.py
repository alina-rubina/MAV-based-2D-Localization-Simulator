##with step size = communication range 
## G-shaped trajectory , Hybrid trajectory 1 

import math
import itertools


def g_shaped(coord,d): #inputs in order of (co-ordinates,first valve(whose second value to be obtained),distance)
    
    
    global x_qc 
    x_qc=0
    global y_qc 
    y_qc=0
    global delta
    #d=.9*d
    s=d*math.sin((60*math.pi)/180)
    c=d*math.cos((60*math.pi)/180)
    delta= math.pow((d/2),.5)
    Result=[] # contains UAV co-ordinates(result)
    step=[]   
    
    def First_point(coord):  
        Result.append(coord)
        return Result     
    
    def Second_point(coord): 
        A=[coord[0]+ (c/2) , coord[1]-(c/2), coord[2] ]
        Result.append(A)     
        return Result 
 
    def Third_point(coord): 
        B=[coord[0] + c ,coord[1]  -c , coord[2] ]
        Result.append(B)     
        return Result
   
    def Fourth_point(coord): 
        C=[coord[0] +( c/2),coord[1] -c, coord[2] ]
        Result.append(C)        
        return Result
   
    def Fifth_point(coord): 
        C=[coord[0] ,coord[1] -c,coord[2] ]
        Result.append(C)        
        return Result
    
    def Sixth_point(coord): 
        D=[coord[0] -(c/2) ,coord[1] -c  ,coord[2] ]
        Result.append(D)  
        return Result  
    
    def Seventh_point(coord): 
        D=[coord[0] -c ,coord[1] -c  ,coord[2] ]
        Result.append(D)  
        return Result
      
    def Eight_point(coord): 
        E=[coord[0] -c  ,coord[1]-(c/2) ,coord[2] ]
        Result.append(E)  
        return Result
    
    def Ninth_point(coord):
        E=[coord[0] -c  , coord[1] ,coord[2] ]
        Result.append(E)  
        return Result
        
    def Tenth_point(coord): 
        F=[coord[0] -c ,coord[1] + (c/2) ,coord[2] ]
        Result.append(F)  
        return Result
    
    def Eleventh_point(coord):
        F=[coord[0] -c ,coord[1] + c ,coord[2] ]
        Result.append(F)  
        return Result
    
    def Twelveth_point(coord): 
        G=[coord[0]-(c/2)   ,coord[1] + c ,coord[2] ]
        Result.append(G)  
        return Result
        
    def Thirteenth_point(coord): 
        G=[coord[0]   ,coord[1] + c ,coord[2] ]
        Result.append(G)  
        return Result
        
    def Fourteenth_point(coord): 
        H=[coord[0] + (c/2)  ,coord[1] +c ,coord[2] ]
        Result.append(H)  
        return Result 
    
    def Fifteenth_point(coord): 
        H=[coord[0] +c  ,coord[1] +c ,coord[2] ]
        Result.append(H)  
        return Result            
    
    First_point(coord)
    #Second_point(coord) 
    Third_point(coord)
    #Fourth_point(coord)
    #Fifth_point(coord) 
    #Sixth_point(coord)
    Seventh_point(coord) 
    #Eight_point(coord)
    #Ninth_point(coord)
    #Tenth_point(coord)
    Eleventh_point(coord) 
    #Twelveth_point(coord)
    #Thirteenth_point(coord) 
    #Fourteenth_point(coord)
    Fifteenth_point(coord)
   
    x = iter(Result) #returns every element at a time
    #print x
    
  
    return x 
    
#w=g_shaped([20,20,10],2)
#print w.next()
#print w.next(),'final'
#print w.next(),'final'
#print w.next(),'final'
#print w.next(),'final'
#print w.next(),'final'
#print w.next(),'final'
#print w.next(),'final'
#print w.next(),'final'
#print w.next(),'final'
#print w.next(),'final'
#print w.next(),'final'
#print w.next(),'final'
#print w.next(),'final'
#print w.next(),'final'
#print w.next(),'final'
#print w.next(),'final'

    
