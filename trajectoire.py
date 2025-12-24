import numpy as np
import configuration as cf
from mgd.mgd import Mgd 
from mdd import Jacobienne 

class MonRobot(Mgd):
    def Jacobienne(self):

        return Jacobienne(self)

def traj(O=(0.320, -0.028, 0.250), R=0.150, V=0.05, Te=0.001):
    Ox, Oy, Oz = O
    
    dist_AB = (np.pi * R) / 2
    dist_BC = (np.pi * R) / 2
    dist_CA = np.pi * R
    
    t1 = (2 * dist_AB) / V
    a1 = V / t1
    t2 = t1 + (dist_BC / V)
    tf = t2 + (2 * dist_CA) / V
    a3 = -V / (tf - t2)
    
    t = np.arange(0, tf + Te, Te)
    
    s_list, sd_list, sdd_list = [], [], []
    x_list, vx_list, ax_list = [], [], []
    z_list, vz_list, az_list = [], [], []
    v_outil_list = []
    q_list, qd_list = [], []


    robot = MonRobot(cf.data)
    q_actuel = np.array([0.0, -1.57, 1.57, 0.0, 0.0, 0.0]) 
    
    for ti in t:
        # A. Loi de mouvement s(t)
        if ti <= t1:
            s, sd, sdd = 0.5*a1*ti**2, a1*ti, a1
        elif ti <= t2:
            tau = ti - t1
            s, sd, sdd = dist_AB + V*tau, V, 0.0
        else:
            tau = ti - t2
            s = (dist_AB + dist_BC) + V*tau + 0.5*a3*tau**2
            sd, sdd = V + a3*tau, a3

        phi = s / R
        x = Ox + R * np.sin(phi)
        z = Oz + R * np.cos(phi)
        vx = np.cos(phi) * sd
        vz = -np.sin(phi) * sd
        ax = -(1/R)*np.sin(phi)*(sd**2) + np.cos(phi)*sdd
        az = -(1/R)*np.cos(phi)*(sd**2) - np.sin(phi)*sdd
        
        robot.set_angles(q_actuel)
        

        J = robot.Jacobienne() 
        
        v_desiree = np.array([vx, 0, vz, 0, 0, 0])
        qd_actuel = np.linalg.pinv(J) @ v_desiree
        q_actuel = q_actuel + qd_actuel * Te

        s_list.append(s); sd_list.append(sd); sdd_list.append(sdd)
        x_list.append(x); vx_list.append(vx); ax_list.append(ax)
        z_list.append(z); vz_list.append(vz); az_list.append(az)
        v_outil_list.append(np.sqrt(vx**2 + vz**2))
        q_list.append(q_actuel)
        qd_list.append(qd_actuel)

    return {
        "t": t, "commutation": (t1, t2, tf),
        "s": (s_list, sd_list, sdd_list),
        "x": (x_list, vx_list, ax_list),
        "z": (z_list, vz_list, az_list),
        "v_outil": v_outil_list,
        "q": q_list, "qd": qd_list
    }
