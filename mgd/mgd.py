import numpy as np
import configuration as cf

class Mgd :
    def __init__(self,data):

        self.data = data
        self.t06 = None
        self.update_transform()
        
    def update_transform(self):
        self.t01 = self.calculate_transform(self.data[0])
        self.t12 = self.calculate_transform(self.data[1])
        self.t23 = self.calculate_transform(self.data[2])
        self.t34 = self.calculate_transform(self.data[3])
        self.t45 = self.calculate_transform(self.data[4])
        self.t45[:3,:3] = self.t45[:3,:3] @ np.array([[0,0,-1],[-1,0,0],[0,1,0]])
        self.t56 = self.calculate_transform(self.data[5])
        
        self.__t06 = self.t01@self.t12@self.t23@self.t34@self.t45@self.t56
        
        
    def calculate_transform(self,params):
        a,alpha,r,teta = params
        matrice = np.array([[np.cos(teta),-np.sin(teta),0,a],
                            [np.cos(alpha)*np.sin(teta),np.cos(alpha)*np.cos(teta),-np.sin(alpha),-r*np.sin(alpha)],
                            [np.sin(alpha)*np.sin(teta),np.sin(alpha)*np.cos(teta),np.cos(alpha),r*np.cos(alpha)],
                            [0,0,0,1]])
        return matrice
    
    def set_angles(self,angles) :
        for i in range(len(angles)) :
            self.data[i][-1] = angles[i]
        self.update_transform()
        
    def get_oriontation(self) :
        return self.__t06[:3,:3]
    
    def get_matrice_homogene(self):
        return self.__t06
    
    def get_position(self):
        return self.__t06[:3,3]
    
    def get_all_homog_matrices(self):
        return [self.t01,self.t12,self.t23,self.t34,self.t45,self.t56]