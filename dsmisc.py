import numpy as np

#  (c) Dominik Schulz, TU Ilmenau, FG Kommunikationsnetze
#
#  Helper classes for multilateration

class RingBufferedMatrix(object):
    "Special matrix that acts as a ring buffer along the rows"
    def __init__(self, maxRows, cols):
        #print " init Ring was called"
        "Create empty matrix"
        self._val = np.matrix(np.empty((0, cols)))
        self._maxRows = maxRows            
        self.idx = 0

    def __call__(self):
        #print " just Ring was called"
        return self._val

    def append(self, value):
        "Append a new row to the matrix. If the matrix is full, the oldest data will be overwritten"
        #print " append Ring was called"
        curRows = self._val.shape[0]      
        if curRows < self._maxRows:
            self._val.resize((curRows+1, self._val.shape[1]), refcheck=False)
        #if isinstance(value, list):
        if not isinstance(value, np.matrix):
            value = np.mat(value)        
        self._val[self.idx, :] = value
        if (not self._maxRows== np.inf):
            self.idx = (self.idx + 1) % self._maxRows
        else:
            self.idx = (self.idx + 1)



    def __len__(self):
        return self._val.shape[0]

    def __iadd__(self, a):
        #print " iadd Ring was called"
        self.append(a)
        return self
        

    @property
    def maxRows(self):
        return self._maxRows

    @property
    def first(self):
        return self._val[(self.idx + 1) % self._val.shape[0], :]

    @property
    def last(self):
        return self._val[self.idx, :]
        
 
 
class AveragedMatrix(RingBufferedMatrix):
    "This class works as a ring buffer but also tracks the mean along the rows"
    def __init__(self, maxRows, cols): 
        super(AveragedMatrix, self).__init__(maxRows, cols)
        self._mean = np.mat(0.)
                
    def append(self, value):
        dataLen = len(self)
        if dataLen > 0:
            if dataLen < self._maxRows:
                self._mean = (dataLen*self._mean + value)/(dataLen+1)
                super(AveragedMatrix, self).append(value)        
            else:
              self._mean = self._mean + (value - self.last)/dataLen
              super(AveragedMatrix, self).append(value)        
                
        else:
            super(AveragedMatrix, self).append(value)
            self._mean = self._val[0, :]

        
    @property
    def mean(self):
        return self._mean
            