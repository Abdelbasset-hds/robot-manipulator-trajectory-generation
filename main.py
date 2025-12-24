import os
import time
import numpy as np
import pybullet as p
import matplotlib.pyplot as plt


from simulation import UR3Simulator

import configuration as cf
from mgd.mgd import Mgd
from trajectoire import traj 

def get_joint_states_main(simulator):
    """
    Lit les angles réels du robot.
    J'ai recréé cette fonction ici car 'control.py' n'est pas directement 
    accessible via l'objet simulator.
    """
    angles = []
    # On utilise la liste control_joints définie dans ton fichier simulation.py
    for joint_index in simulator.control_joints:
        state = p.getJointState(simulator.robot, joint_index)
        angles.append(state[0])     # Position
    return angles

def main():
    # --- A. CONFIGURATION ---
    # Chemin vers l'URDF (adapter si besoin selon ton dossier)
    urdf_path = "simulation/ur_description/urdf/ur3_robot.urdf"
    
    # Vérification dossier
    if not os.path.exists(urdf_path):
        print(f"ATTENTION: Vérifie le chemin: {urdf_path}")

    # Lancement du simulateur (Ton fichier simulation.py s'occupe de tout)
    print("Démarrage du simulateur...")
    sim = UR3Simulator(urdf_path, dt=0.001)
    
    # --- B. CALCUL TRAJECTOIRE ---
    print("Calcul de la trajectoire...")
    # Appel de ta fonction trajectoire
    res = traj(O=(0.320, -0.028, 0.250), R=0.150, V=0.05)
    
    t_arr = res['t']
    q_ref_list = res['q']
    
    # --- C. INITIALISATION ---
    print("Placement initial...")
    # On place le robot au premier point
    sim.set_joint_angles(q_ref_list[0])
    
    # --- D. BOUCLE D'EXÉCUTION ---
    print(f"Exécution de la trajectoire ({len(t_arr)} points)...")
    
    X_reel_list = []
    X_ref_list = []
    
    # Outil pour vérifier la position Cartésienne
    verificateur = Mgd(cf.data)

    for i in range(len(t_arr)):
        # 1. Commande : On envoie l'angle désiré
        # ATTENTION : Ton fichier control.py fait déjà 100 pas de simulation ici !
        # Le mouvement sera donc lent et fluide, c'est normal avec ton code actuel.
        sim.set_joint_angles(q_ref_list[i])
        
        # Note : On ne fait PAS sim.step_simulation() ici car set_joint_angles le fait déjà.

        # 2. Validation : On regarde où est vraiment le robot
        q_actuel = get_joint_states_main(sim)
        
        # Calcul position réelle (via MGD)
        verificateur.set_angles(q_actuel)
        X_reel_list.append(verificateur.get_position())
        
        # Calcul position théorique (via MGD avec consigne)
        verificateur.set_angles(q_ref_list[i])
        X_ref_list.append(verificateur.get_position())

    print("Fin du mouvement. Affichage des courbes...")
    
    # --- E. AFFICHAGE RÉSULTATS ---
    X_reel = np.array(X_reel_list)
    X_ref = np.array(X_ref_list)
    
    # Comparaison 3D
    fig = plt.figure(figsize=(10, 6))
    ax = fig.add_subplot(1, 1, 1, projection='3d')
    ax.plot(X_ref[:,0], X_ref[:,1], X_ref[:,2], 'b--', label='Consigne')
    ax.plot(X_reel[:,0], X_reel[:,1], X_reel[:,2], 'r', linewidth=2, label='Réel')
    ax.set_title("Validation Trajectoire (Consigne vs Réel)")
    ax.set_xlabel('X'); ax.set_ylabel('Y'); ax.set_zlabel('Z')
    ax.legend()
    plt.show()

if __name__ == "__main__":
    main()
