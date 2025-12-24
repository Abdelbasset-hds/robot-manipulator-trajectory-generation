import os
import numpy as np
import matplotlib.pyplot as plt
import pybullet as p

import configuration as cf
from mgd.mgd import Mgd
from trajectoire import traj

from simulation import UR3Simulator
from simulation import control
from simulation import affiche  

def main():
    urdf_path = "simulation/ur_description/urdf/ur3_robot.urdf"
    
    if not os.path.exists(urdf_path):
        print(f"ERREUR : Fichier introuvable {urdf_path}")
        return

    print("Démarrage du simulateur...")
    sim = UR3Simulator(urdf_path, dt=0.001)
    
    print("Génération de la trajectoire...")
    res = traj(O=(0.320, -0.028, 0.250), R=0.150, V=0.05)
    
    t_arr = res['t']
    q_ref_list = res['q']
    
    print("Positionnement initial...")
    sim.set_joint_angles(q_ref_list[0])
    
    print(f"Exécution du mouvement ({len(t_arr)} pas)...")
    
    X_reel_list = []
    X_ref_list = []
    verificateur = Mgd(cf.data)

    for i in range(len(t_arr)):
        sim.set_joint_angles(q_ref_list[i])
        
        q_reel, _, _ = control.getJointStates(sim.robot, sim.control_joints)
        
        verificateur.set_angles(q_reel)
        X_reel_list.append(verificateur.get_position())
        
        verificateur.set_angles(q_ref_list[i])
        X_ref_list.append(verificateur.get_position())

    print("Simulation terminée.")


    print("Affichage de la loi de mouvement (via affiche.py)...")
    affiche.afficheloi_mvt(res['s'][0], res['s'][1], res['s'][2], res['t'], res['commutation'])

    print("Affichage de la comparaison 3D...")
    X_reel = np.array(X_reel_list)
    X_ref = np.array(X_ref_list)
    
    fig = plt.figure(figsize=(10, 6))
    ax = fig.add_subplot(1, 1, 1, projection='3d')
    ax.plot(X_ref[:,0], X_ref[:,1], X_ref[:,2], 'b--', label='Consigne (Théorique)')
    ax.plot(X_reel[:,0], X_reel[:,1], X_reel[:,2], 'r', linewidth=2, label='Réel (Simulé)')
    ax.set_title("Validation Trajectoire 3D")
    ax.set_xlabel('X'); ax.set_ylabel('Y'); ax.set_zlabel('Z')
    ax.legend()
    plt.show()

if __name__ == "__main__":
    main()
