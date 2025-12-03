from mgd import Mgd
from simulation import UR3Simulator
import numpy as np
import configuration as cf
import os
import time
import pybullet as p
import pybullet_data
from simulation import control

def main() :
    

    sim = UR3Simulator("/simulation/ur_description/urdf/ur3_robot.urdf")
    angles_init = [0, 0, 0, 0, 0, 0]
    sim.set_joint_angles(angles_init)
    while True:
        sim.step_simulation()
        

    mgd_instance = Mgd(cf.data)    
    mgd_instance.set_angles([0, 0., 0., 0, 0, 0 ])
    print(mgd_instance.get_matrice_homogene())
    print("*******************************")
    print(f"position : {mgd_instance.get_position()}")
    print(f"angles : {mgd_instance.get_oriontation()}")
    
if __name__ == "__main__" :
    main()