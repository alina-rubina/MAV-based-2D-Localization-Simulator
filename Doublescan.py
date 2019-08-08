    
def DoubleScan(x_y_zinit_qc, X_max, Y_max, Z_max, r):
    global x_qc 
    x_qc=0
    global y_qc 
    y_qc=0
    global v
    v=0
    global h
    h=0
    #Result_y=[]
    gap=(r-.005); #this is done ,so that values dont repeat 
    Z_max=0;
   
    Result=[] # contains UAV co-ordinates(result of dual scan)
    Initial_point=[-X_max, -Y_max, Z_max] 
    Result.append(Initial_point) 
    
    
    
    def verticalup_long(Result): 
        # forms the longer vertical portion of ( from bottom to top ) vertical scan 
        for i in range (1,int(((Y_max*2)/r)+1)):
        
            if ( (Result[-1][1] <= Y_max-r) ):
                A=[Result[-1][0], Result[-1][1] + (r), Z_max]
                Result.append(A)    
        return Result
    
    
    def horizontalright_short(Result):
        # forms the shorter horizontal portion of ( from right to left ) vertical scan 
        if ( Result[-1][0]+r <= X_max):
            B=[Result[-1][0]+r, Result[-1][1],Z_max]
            Result.append(B)   
        return Result   
   
    
    def verticaldown_long(Result):
       # forms the longer vertical portion of ( from top to bottom ) vertical scan 
        for i in range (1,int(((Y_max*2)/r)+1)): 
            if ((Result[-1][1]) >= (-Y_max + r)):
                C=[Result[-1][0],Result[-1][1]-(r),Z_max]
                Result.append(C)
        return Result               
   
    
    def lefthorizontal_long(Result):
       # forms the longer horizontal portion of ( from left  to right ) horizontal scan 
        
        for i in range (1,int(((X_max*2)/gap)+1)): 
            if ((Result[-1][0]) >= (-X_max + gap)):
                D=[Result[-1][0]-gap,Result[-1][1],Z_max]
                Result.append(D)
        return Result 
    
    
    def righthorizontal_long(Result):
        # forms the longer horizontal portion of ( from right to left ) horizontal scan 
        
        for i in range (1,int(((X_max*2)/gap)+1)): 
            if ((Result[-1][0]) <= (X_max - gap)):
                E=[Result[-1][0]+gap,Result[-1][1],Z_max]
                Result.append(E)
        return Result 
    
    
    def verticalup_short(Result):
     
     #Going up,Horizontal scan
        if ((Result[-1][1]) <= (Y_max - gap)):
            F=[Result[-1][0], Result[-1][1]+gap, Z_max]
            Result.append(F)
        return Result 
    
    
    def verticaldown_short(Result):
     #Going down,Horizontal scan
        
         if ((Result[-1][1]) >= (-Y_max + gap)):
                G=[Result[-1][0], Result[-1][1]-gap, Z_max]
                Result.append(G)
         return Result
    
    
     
    
    
    while( ( ((Result[-1][0])+r) <=X_max) ):  #creates pattern up and many (right,down,right,up) = vertical scan 
        
        if(v==0):
            Result=verticalup_long(Result); #runs only one time,vertical up
            v=v+1;        
                  
        if(((Result[-1][0]+r) <= X_max )):  # right,verticaldown
            Result=horizontalright_short(Result);
            Result=verticaldown_long(Result);
            
        if(((Result[-1][0]+r) <= X_max )):  #right,verticalup 
            Result=horizontalright_short(Result);
            Result=verticalup_long(Result);
                
          
   #There are two possibilities 1) horizontal scan going up and 2) horizontal scan going down  
    
    if(Result[-1][1]>=0):# horizontal scan going down  
       
       
     
       
        while (((Result[-1][1])-gap) >= -Y_max):
            
            #creates pattern with left and many (down,right,down,left) = horizontal scan 
          
            if(h==0):  #runs only one time, horizontal right up
                Result=lefthorizontal_long(Result);          
                h=h+1;
                
            
            if(((Result[-1][1])+gap) >= -Y_max):
                Result=verticaldown_short(Result);
                Result=righthorizontal_long(Result);
                          
            
            if(((Result[-1][1])+gap) >= -Y_max):
                Result=verticaldown_short(Result);
                Result=lefthorizontal_long(Result);         
        
    else:      
    
          #creates pattern with left and many(up,right,up,left) = horizontal scan 
         
        while (((Result[-1][1])+gap) <= Y_max): #horizontal scan going up
            
            if(h==0): #runs only one time, horizontal left up
                Result=lefthorizontal_long(Result);          
                h=h+1;
             
            if(((Result[-1][1])+gap) <= Y_max):
                Result=verticalup_short(Result);
                Result=righthorizontal_long(Result);
            
            if(((Result[-1][1])+gap) <= Y_max):
                Result=verticalup_short(Result);
                Result=lefthorizontal_long(Result);     
     
          
  
    
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

    
    return [x_qc,y_qc,0];  
    
  
           
    
    

#hello= DoubleScan([1, 133, 0],200,200,0,45)
#print (hello)
                           
                





