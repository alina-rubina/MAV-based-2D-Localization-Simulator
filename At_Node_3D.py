#At Node 3D


       
       # packet from UAV - Source ID = 0 ,status= status of the node (from multilateration) (1=localised),NULoN = ignore
                    # Rx_packet = [0, Destination ID, yourstatus, NULoN]
      
       # packet from neighbour nodes - Source ID = ID of node, Destination ID = Broadcast ID
                    # Rx_packet= [nodeID, Broadcast destn., mystatus, NULoN]

       # packet to transmit from node , broadcast                     
                    # Tx_packet = [myID, Broadcast, mystatus,NULoN] 


class mNode(object):
    "class node"
    def __init__(self, myId, x, y, z):
        "K: float  [dBm],  pathLossExponent: float, referenceDistance: float [m]"
        self.myId = myId;
        self.x_y_zreal = [x,y,z];
        self.x_y_zestim = [0,0,0];
        self.mystatus = 0; #my status: 0-unlocalized, 1-localized
        self.NULoN = 0; # Number of Unlocalised Neighbours
        self.NULoN_UAV_view = 0; # Number of Unlocalised Neighbours that was seen previously by UAV in the last communication session
        self.nbr = []; #neighbors of this node incl
        self.database = []; #database of transfered messages extended with SS and current coordinates of the UAV
        self.visible_neigbords = [];
        self.my_neigbords = [];

                                                                                                                                               # Initialise mystatus = 0 , i.e. I am not localised
    def update_database(self, packet, ss, x_y_zuav): # data = [3, "bdcast", 1, 0]
        self.database.append([packet, ss, x_y_zuav]);
    
    def update_listofnbr(self, data): # data = [3, "bdcast", 1, 0]
        
        replaced = 0
        for i in range(len(self.nbr)):
            if self.nbr[i][0] == data[0]:
                self.nbr[i] = data
                replaced = 1
            
        if replaced == 0:
            self.nbr.append(data)
            
        #print ("my_updated_neighbour_list = ",self.nbr)//ec

    def packet_receive(self, packet): # data = [3, "bdcast", 1, 0]
        if packet[0] == 0 :                                                              #if Rx_packet is from UAV 
            if packet[1] == "myID":                                                      # to me #myID is taken from within the node itself
                if packet[2] == 1:                                                   # when I am localised after multilateration at UAV 
                    self.mystatus = 1
                    #print ("UAV told me that I am localised")//ec
        else:                                                                               #If Rx_packet is from neighbouring nodes
            #print ("I received a packet from neighbour node")//ec            self.update_listofnbr(packet)                                            #add the new packet to the list or update it,
            self.update_nulon()

    def update_nulon(self):
        self.NULoN = 0
        for i in range(len(self.nbr)) :                                                            #from the list calculate the number of Unlocalised neighbours
            if self.nbr[i][2] == 0:
                self.NULoN += 1
            
    def packet_transmit(self): # data = [3, "bdcast", 1, 0]
        packet = [];        
        packet.append(self.myId);
        packet.append("f");
        packet.append(self.mystatus);
        packet.append(self.NULoN);
        return packet;


#examples of self.nbr:
'''
nbr.append( [1, "bdcast", 1, 5] )
nbr.append( [2, "bdcast", 0, 6] )
nbr.append( [3, "bdcast", 1, 0] )

# for every Rx_packet
Rx_packet= random.choice([[0, "myID", 1 , 0],[4, "bdcast", 0 , 0] ,[3,"bdcast",1,6]])

print "Rx_pkt =",Rx_packet


Tx_packet = ['myID','to all', mystatus,NULoN]                                       # Tx_packet = [myID, Broadcast, mystatus,NULoN]

# After a set of node packets received , broadcast Tx_packet with updated values

print "Tx_packet=", Tx_packet
'''
