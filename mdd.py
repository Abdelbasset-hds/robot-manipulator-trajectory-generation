import numpy as np

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
    Jg = self.mdd()
    Xpoint = Jg @ qpoint
    return Xpoint      
