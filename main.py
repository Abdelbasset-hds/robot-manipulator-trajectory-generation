from mgd import Mgd
import numpy as np
import configuration as cf

def main() :
        
    mgd_instance = Mgd(cf.data)    
    matrice_homogene = mgd_instance.get_matrice_homogene()
    print(matrice_homogene)
    print("*******************************")
    print(mgd_instance.get_position())
    print("*******************************")
    matrice_homogene = mgd_instance.get_matrice_homogene()
    print(matrice_homogene)
    mgd_instance.set_angles([np.pi,np.pi,np.pi,np.pi,np.pi,np.pi])
    print("*******************************")
    print(mgd_instance.get_position())
    
if __name__ == "__main__" :
    main()