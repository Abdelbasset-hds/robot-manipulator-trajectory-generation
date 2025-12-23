import numpy as np

def traj(O=(0.320, -0.028, 0.250), R=0.150, V=0.05, Te=0.001):
    Ox, Oy, Oz = O

    # --- 1. CALCUL DES PARAMÈTRES DE TEMPS (V.1) ---
    # Calcul des distances pour chaque phase du cercle
    dist_AB = (np.pi * R) / 2
    dist_BC = (np.pi * R) / 2
    dist_CA = np.pi * R
    
    # Calcul des temps et accélérations (Profil trapézoïdal)
    t1 = (2 * dist_AB) / V
    a1 = V / t1
    t2 = t1 + (dist_BC / V)
    tf = t2 + (2 * dist_CA) / V
    a3 = -V / (tf - t2)
    
    # Création du vecteur temps
    t = np.arange(0, tf + Te, Te)
    
    # --- 2. GÉNÉRATION DE L'ABSCISSE CURVILIGNE s(t) ---
    s = np.zeros_like(t)
    sd = np.zeros_like(t)
    sdd = np.zeros_like(t)
    
    for i in range(len(t)):
        ti = t[i]
        if ti <= t1:
            s[i]   = 0.5 * a1 * ti**2
            sd[i]  = a1 * ti
            sdd[i] = a1
        elif ti <= t2:
            tau = ti - t1
            s[i]   = dist_AB + V * tau
            sd[i]  = V
            sdd[i] = 0.0
        else:
            tau = ti - t2
            s[i]   = (dist_AB + dist_BC) + V * tau + 0.5 * a3 * tau**2
            sd[i]  = V + a3 * tau
            sdd[i] = a3

    # --- 3. TRAJECTOIRE DANS L'ESPACE OPÉRATIONNEL X(t) (V.3) ---
    phi_t = s / R
    
    # Coordonnées cartésiennes au cours du temps
    x_t = Ox + R * np.sin(phi_t)
    y_t = Oy * np.ones_like(t)
    z_t = Oz + R * np.cos(phi_t)
    
    # Vitesses cartésiennes au cours du temps
    vx_t = np.cos(phi_t) * sd
    vy_t = np.zeros_like(t)
    vz_t = -np.sin(phi_t) * sd

    # --- 4. PRÉPARATION DE L'ESPACE ARTICULAIRE (V.4) ---
   
    q_t = []   
    qd_t = []  
    qdd_t = [] 

    for i in range(len(t)):
        # Pour l'instant, on met des listes vides de 6 éléments
        q_t.append([0.0] * 6)
        qd_t.append([0.0] * 6)
        qdd_t.append([0.0] * 6)

    return t, q_t, qd_t, qdd_t
