import math
import itertools
import numpy as np 

def circle(x_y_zinit_qc,x_max,y_max,z_max,radius): #(1.co-ordinates and 2.radius of circle depending on communication range)
    Result=[]    
    global x_qc 
    x_qc=0
    global y_qc 
    y_qc=0
      
    for t in range(1,(x_max/radius)+1):         
          
            th=np.arange(0,(2*math.pi),(2*math.pi)/(7*t)); #by this number 16 you make it smooth, you can change it
            r=radius*t; #these r,r1 are the corresponding radiuses in x and y direction
            r1=radius*t;
            x=r*np.cos(th);
            y=r1*np.sin(th);
            if((min(x)>=-x_max) and (max(x)<=x_max) and (min(y)>=-y_max) and (max(y)<=y_max) ):          
                for i in range(0, len(x)):
                    A=[x[i],y[i],0]
                    Result.append(A)
                    if(i==len(x)-1):
                        A=[x[0]+.001,y[0]+.001,0]
                        Result.append(A)
    
    #print Result
    xxLMAT=list(zip(*Result)[0])
    yyLMAT=list(zip(*Result)[1])
    

    
    ##This gives 2ND value on inserting first value,3RD value on inserting SECOND value,ETC..from the array list.
    for n in range(0,len(xxLMAT)):
            
           if (any([xxLMAT[n] == x_y_zinit_qc[0]]) and any([yyLMAT[n]==x_y_zinit_qc[1]])):
                if n == (len(xxLMAT)-1):
                    x_qc=xxLMAT[0];
                    y_qc=yyLMAT[0];
                else:  
             
                    x_qc=xxLMAT[n+1];
                    y_qc=yyLMAT[n+1];

    
    return [x_qc,y_qc,0]; #returns next element at a time
    
    
    
    
#w=circle([0,0,0],200,200,0,90)
#print '##############################'
#print w    
