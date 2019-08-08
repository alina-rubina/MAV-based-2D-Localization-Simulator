import math  
    
def triangle(x_y_zinit_qc, X_max, Y_max, Z_max, step):
    
    global x_qc 
    x_qc=0
    global y_qc 
    y_qc=0
    Z_max=0;
    angle= 60*(math.pi/180) # we consider angle in triangle as 60 degrees 
    x_delta=(step * math.cos(angle))
    y_delta=(step * math.sin(angle))
    
    
   
    Result=[] # contains UAV co-ordinates(result of triangular scan)
    Initial_point=[-X_max, -Y_max, Z_max] 
    Result.append(Initial_point) 
    
    
    
    def forward_triangle(Result): 
        # forms  triangle ,from left to right
        for i in range (0, int( ((X_max*2)/x_delta) + 1 ) ):
        
            if ((Result[-1][0] + (x_delta)*2) <= X_max  ):
                A=[Result[-1][0]+x_delta, Result[-1][1]+y_delta , Z_max]
                Result.append(A)    
            
            #if ((Result[-1][0] + (x_delta)) <= X_max  ):   
                B=[Result[-1][0]+(x_delta), Result[-1][1]-y_delta  , Z_max]
                Result.append(B)     
        return Result
    
    
    def vertical_strip(Result):
        #  distance between two triangles
       
        if(Result[-1][1] <= Result[-2][1]):            
            if ((Result[-1][1]+y_delta) <= Y_max):
                C=[Result[-1][0], Result[-1][1]+y_delta, Z_max]
                Result.append(C)   
        else:
             if ((Result[-1][1]+y_delta) <= Y_max):
                C=[Result[-1][0], Result[-1][1]+.001, Z_max]
                Result.append(C) 
    
        return Result   
   
    
    def reverse_triangle(Result):
       #  forms triangle ,from right to left
        for i in range (0, int( ((X_max*2)/x_delta) + 1 ) ):
            
            if ((Result[-1][0] -(x_delta)*2) >= -X_max ):
                D=[Result[-1][0]-x_delta, Result[-1][1]+y_delta , Z_max]
                Result.append(D)    
            
            #if ((Result[-1][0] - (x_delta)) >= -X_max  ):  
                E=[ Result[-1][0]-(x_delta), Result[-1][1]-y_delta , Z_max]
                Result.append(E)  
               
        return Result               
   
    
    
         
    for i in range (0,int(((Y_max*2)/y_delta) + 1 )): 
            
        if( Result[-1][1]+ y_delta <= Y_max ): 
            Result=forward_triangle(Result);          
                
             
        if(((Result[-1][1])+y_delta) <= Y_max):
            Result=vertical_strip(Result);
               
            
        if(((Result[-1][1])+y_delta) <= Y_max):
            Result=reverse_triangle(Result); 
          
        if(((Result[-1][1])+y_delta) <= Y_max):
            Result=vertical_strip(Result);
    
    xxLMAT=list(zip(*Result)[0])
    yyLMAT=list(zip(*Result)[1])
     
    #print Result
    
    ##This gives 2ND value on inserting first value,3RD value on inserting SECOND value,ETC..from the array list.
    for n in range(0,len(xxLMAT)):
            
           if (any([xxLMAT[n] == x_y_zinit_qc[0]]) and any([yyLMAT[n]==x_y_zinit_qc[1]])):
                if n == (len(xxLMAT)-1):
                    x_qc=xxLMAT[0];
                    y_qc=yyLMAT[0];
                else:  
             
                    x_qc=xxLMAT[n+1];
                    y_qc=yyLMAT[n+1];

    
    return [x_qc,y_qc,0];  
    
  
           
    
#triangle([1, 133, 0],200,200,0,45)
#print (hello)
                           
                





