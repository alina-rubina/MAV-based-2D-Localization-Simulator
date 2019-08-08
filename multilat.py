import numpy as np
from dsmisc import *

#  (c) Dominik Schulz, TU Ilmenau, FG Kommunikationsnetze
#
#  Classes for multilateration

class PropagationModel(object):
    "Abstracts a propagation model. This is currently only used to compute distances."
    
    def __init__(self, K, pathLossExponent, referenceDistance=1):
        "K: float  [dBm],  pathLossExponent: float, referenceDistance: float [m]"
        self.K = K;
        self.PL = pathLossExponent;
        self.d0 = referenceDistance;
        
    def getDistance(self, rssi):
        "Return the estimated distance given a certain RSSI."
        return self.d0 * 10**( (self.K - rssi) / (10*self.PL));
        

       

class MultilaterationBase(object):
    "Base multilateration class"
    
    def __init__(self, dims, propModel, windowSize=np.inf, minRssi=-np.inf):
        "Initialize some basic stuff"        
        #print " init MBase was called"
        if (dims < 1) or (not isinstance(propModel, PropagationModel)) or (windowSize < 1):
            raise ValueError        
        self._dims = dims
        self._changed = False        
        self._currentPosition = np.asmatrix(np.ones((dims,1))*np.NaN)
        self.propModel = propModel        
        self.timeStamps = RingBufferedMatrix(windowSize, 1)      
        self.refPositions = RingBufferedMatrix(windowSize, dims)
        self.rssi = RingBufferedMatrix(windowSize, 1)
        self.minRssi = minRssi

    def __iadd__(self, value):
        "Convenience operator (+=) to add new data to the multilateration. See append() method."
        self.append(value[0], value[1], value[2])
        return self
        
    def index_val(self):
        "Convenience function that returns how much data has been put into the multilateration." 
        a = self.timeStamps
        return a.bla

    def __len__(self):
        "Convenience function that returns how much data has been put into the multilateration."
        return len(self.timeStamps)

    @property
    def changed(self):
        return self._changed
        
    def isValid(self, timeStamp, position, rssi, distance):
        "Defines whether a set of data is taken into account or not. May be overwritten."
        return rssi >= self.minRssi

    def append(self, timeStamp, position, rssi):
        "timeStamp: float, positions: float [m], rssi: float [dBm]"
        #print "append MB was called " + str(self._bla.bla)      
        if len(position) != self._dims:
            raise ValueError
        distance = self.propModel.getDistance(rssi)  
        #print distance
        if self.isValid(timeStamp, position, rssi, distance):            
            position = np.mat(position)
            if (position.shape[1] > 1) and (position.shape[0] == 1):
                pass
            elif (position.shape[0] > 1) and (position.shape[1] == 1):
                position = position.T
            else:
                raise ValueError
            self.update(timeStamp, position, rssi, distance)
            self._changed = True            
        return self._changed
        
    def update(self, timeStamp, position, rssi, distance):
        "Here you can update and store whatever you want on each new set of data. Overwrite this"
        #print " update MBase was called"
        self.timeStamps += timeStamp
        self.refPositions += position
        self.rssi += rssi
            

    def estimatePosition(self):
        "Is used to retrieve the estimated position. The actual estimation is done in calcPosition()"
        if self.changed:
            self._currentPosition = self.calcPosition()           
            self._changed = False;             
        return self._currentPosition

    def calcPosition(self):
        "Overwrite this to calculate the position based on the measured values."
        pass



class Multilateration(MultilaterationBase):
    "Normal multilateration class"
    
    def __init__(self, dims, propModel, windowSize=np.inf, minRssi=-np.inf, iterCount=1):
        #print " init M was called"
        super(Multilateration, self).__init__(dims, propModel, windowSize, minRssi)
        if iterCount < 1:
            raise ValueError
        self._iterCount = iterCount
        self.A = RingBufferedMatrix(windowSize, 2)

    def update(self, timeStamp, position, rssi, distance):
        #print " update M was called"
        # We also store ones here so we do not have to create a vector of ones later on
        super(Multilateration, self).update(timeStamp, position, rssi, distance)
        self.A += [np.sum(np.power(position,2)) - pow(distance, 2), 1] 
        #print np.sum(np.power(position,2)) - pow(distance, 2)

    def calcPosition(self):
        #print "normal   ML"
        B = np.linalg.lstsq(self.refPositions(), self.A())[0]
        q = B[:, 0]
        r = B[:, 1]
        gamma = np.roots([float(r.T*r), 2*sum(self.A()[:,0]) - 4, float(q.T*q)])
        #print gamma
        if len(gamma)==1:
            #print "condit 0 worked"
            return 0.5*(q + gamma[0]*r)            
        if (gamma[0] < 0) and (not np.iscomplex(gamma[1])):
            #print "condit 1 worked"
            return 0.5*(q + gamma[1]*r)
        elif gamma[1] < 0 and (not np.iscomplex(gamma[0])):
            #print "condit 2 worked"
            return 0.5*(q + gamma[0]*r)
        else:
            #print "condit else worked"
            # take the real part - just to be sure
            gamma = np.real(gamma)
            p0 = 0.5*(q + gamma[0]*r)
            p1 = 0.5*(q + gamma[1]*r)
            for i in range(self._iterCount - 1):
                gamma[0] = p0.T*p0
                gamma[1] = p1.T*p1
                p0 = 0.5*(q + gamma[0]*r)
                p1 = 0.5*(q + gamma[1]*r)
                #print p0
                #print p1
            tmp0 = np.sum(abs(gamma[0] + self.A()[:,0] - 2*self.refPositions() * p0))            
            tmp1 = np.sum(abs(gamma[1] + self.A()[:,0] - 2*self.refPositions() * p1))
            if tmp0 <= tmp1:
                return p0
            else:
                return p1


class SimpleMultilateration(MultilaterationBase):
    "Simplified multilateration class that does not need to compute the norm of the position to be estimated"
    
    def __init__(self, dims, propModel, windowSize=np.inf, minRssi=-np.inf):
        "Initialize some basic stuff"        
        #super(SimpleMultilateration, self).__init__(dims, propModel, windowSize, minRssi)
        super(SimpleMultilateration, self).__init__(dims, propModel, windowSize, minRssi)
        self.A = RingBufferedMatrix(windowSize, 1)
        self.meanB = AveragedMatrix(windowSize, dims)
        self.meanNormBSqr = AveragedMatrix(windowSize, 1)
        self.meanDSqr = AveragedMatrix(windowSize, 1)
        

    def update(self, timeStamp, position, rssi, distance):
        # We also store ones here so we do not have to create a vector of ones later on
        super(SimpleMultilateration, self).update(timeStamp, position, rssi, distance)
        self.A += np.sum(np.power(position,2)) - pow(distance, 2) 
        self.meanB = position
        self.meanNormBSqr += position*position.T
        self.meanDSqr += distance**2
        

    def calcPosition(self):
        print "simple  ML"
        o = np.asmatrix(np.ones((len(self), 1), dtype=np.float))
        B = self.refPositions() - o * self.meanB
        C = self.A() + o *(self.meanDSqr.mean   - self.meanNormBSqr.mean)
        return 0.5*np.linalg.lstsq(B, C)[0]


class HybridMultilateration(MultilaterationBase):
    "Combination of Multilateration and SimpleMultilateration"
    
    def __init__(self, dims, propModel, windowSize=np.inf, minRssi=-np.inf, iterCount=1):
        "iterCount ==> iterations in calculating the position"
        #super(SimpleMultilateration, self).__init__(dims, propModel, windowSize, minRssi)
        super(HybridMultilateration, self).__init__(dims, propModel, windowSize, minRssi)
        self.A = RingBufferedMatrix(windowSize, 2)
        self.meanB = AveragedMatrix(windowSize, dims)
        self.meanNormBSqr = AveragedMatrix(windowSize, 1)
        self.meanDSqr = AveragedMatrix(windowSize, 1)
        self.iterCount = iterCount
        

    def update(self, timeStamp, position, rssi, distance):
        # We also store ones here so we do not have to create a vector of ones later on
        super(HybridMultilateration, self).update(timeStamp, position, rssi, distance)
        self.A += [np.sum(np.power(position,2)) - pow(distance, 2), 1]
        self.meanB = position
        self.meanNormBSqr += position*position.T
        self.meanDSqr += distance**2
        

    def calcPosition(self):
        print "hybrid ML"
        o = np.asmatrix(np.ones((len(self), 1), dtype=np.float))
        B = self.refPositions() - o * self.meanB
        C = self.A()[:,0] + o *(self.meanDSqr.mean   - self.meanNormBSqr.mean)
        p = 0.5*np.linalg.lstsq(B, C)[0]
        B = np.linalg.lstsq(self.refPositions(), self.A())[0]
        q = B[:, 0]
        r = B[:, 1]
        for i in range(self.iterCount):
            p = 0.5*(q + float(p.T*p)*r)
        return p