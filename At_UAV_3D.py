#At Node 3D


       
       # packet from UAV - Source ID = 0 ,status= status of the node (from multilateration) (1=localised),NULoN = ignore
                    # Rx_packet = [0, Destination ID, yourstatus, NULoN]
      
       # packet from neighbour nodes - Source ID = ID of node, Destination ID = Broadcast ID
                    # Rx_packet= [nodeID, Broadcast destn., mystatus, NULoN]

       # packet to transmit from node , broadcast                     
                    # Tx_packet = [myID, Broadcast, mystatus,NULoN] 

from At_Node_3D import *
class mUav(object):
    "class node"
    def __init__(self, uavId, step, steps, x, y, z):
        "K: float  [dBm],  pathLossExponent: float, referenceDistance: float [m]"        
        self.uavId = uavId;
        self.uav_current_coord = [x,y,z];
#        self.step = 50;
#        self.steps = 0;
        self.NULoN_UAV_view = 0; # Number of Unlocalised Neighbours that was seen previously by UAV in the last communication session
        self.database = []; #database of transfered messages extended with SS and current coordinates of the UAV
        self.route_length = 0;
        self.unlocalized_nodes = [];
                                                                                                                                               # Initialise mystatus = 0 , i.e. I am not localised
    def update_database(self, packet, ss, x_y_zuav): # data = [3, "bdcast", 1, 0]
        self.database.append([packet, ss, x_y_zuav]);
    
            

    def packet_transmit(self, packet): # data = [3, "bdcast", 1, 0]
        if packet[0] == 0 :                                                              #if Tx_packet is from UAV 
            if packet[1] == "myID":                                                      # to the node #myID is taken from within the node itself
                if packet[2] == 1:                                                   # when the Node localised after multilateration at UAV 
                    self.mystatus = 1
                    #print ("UAV told the Node that It is localised")
        else:                                                                               #If Rx_packet is from neighbouring nodes
            #print ("the Node received a packet from neighbour node")
            self.update_listofnbr(packet)                                            #add the new packet to the list or update it,
            self.update_nulon()

            
    def packet_receive(self): # data = [3, "bdcast", 1, 0]
        packet = [];        
        packet.append(self.myId);
        packet.append("f");
        packet.append(self.mystatus);
        packet.append(self.NULoN);
        return packet;
        
    def save_node_NULON(self,node,NULoN): # data = [3, "bdcast", 1, 0]
        node_found = 0;
        for i in range(len(self.unlocalized_nodes)):
            if self.unlocalized_nodes[i][0] == node:
                self.unlocalized_nodes[i][1] = NULoN;
                #sorting according to localised nodes with highest NULoN(Number of Unlocalised Neighbours)
                #self.unlocalised_nodes.reverse();
                node_found = 1;
        if node_found == 0:
            self.unlocalized_nodes.append([node,NULoN])
        self.unlocalized_nodes.sort();
            
    def move(self,node,NULoN): # data = [3, "bdcast", 1, 0]
        node_found = 0;
        for i in range(len(self.unlocalized_nodes)):
            if self.unlocalized_nodes[i][0] == self.unlocalized_nodes[node]:
                self.unlocalized_nodes[i][0] = NULoN;
                node_found = 1;
        if node_found == 0:
            self.unlocalized_nodes.append([node,NULoN])
    
    def printyourself(self): # data = [3, "bdcast", 1, 0]
        print "self.uavId", self.uavId
        print "self.uav_current_coord",self.uav_current_coord
        print "self.NULoN_UAV_view",self.NULoN_UAV_view # Number of Unlocalised Neighbours that was seen previously by UAV in the last communication session
        #print "self.database",self.database #database of transfered messages extended with SS and current coordinates of the UAV
        #print "self.route_length",self.route_length
        #print "I know about unlocalized_nodes",self.unlocalized_nodes
                
                
                
             
            
            
        



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
