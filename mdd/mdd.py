import numpy as np
from mgd.mgd import Mgd

class Mdd :
    def __init__(self,data):
        
        self.data = data
        mgd = Mgd(self.data)
        self.t01 = mgd.calculate_transform(self.data[0])
        self.t12 = mgd.calculate_transform(self.data[1])
        self.t23 = mgd.calculate_transform(self.data[2])
        self.t34 = mgd.calculate_transform(self.data[3])
        self.t45 = mgd.calculate_transform(self.data[4])
        self.t45[:3,:3] = self.t45[:3,:3] @ np.array([[0,0,-1],[-1,0,0],[0,1,0]])
        self.t56 = mgd.calculate_transform(self.data[5])

    def Jacobienne(self):
          O = []
          Z = []

          T = np.eye(4)

          Ts = [self.t01, self.t12, self.t23, self.t34, self.t45, self.t56] 
          for Ti in Ts:
            T = T @ Ti
            O.append(T[0:3, 3])
            Z.append(T[0:3, 2])

          Oe = O[-1]

          Jg = np.zeros((6, 6))

          for i in range(6):
              zi = Z[i]
              oi = O[i]
              Jg[0:3, i] = np.cross(zi, Oe - oi) 
              Jg[3:6, i] = zi                     

          return Jg

    def MDD(self, qpoint):
        Jg = self.Jacobienne()
        Xpoint = Jg @ qpoint
        return Xpoint      
