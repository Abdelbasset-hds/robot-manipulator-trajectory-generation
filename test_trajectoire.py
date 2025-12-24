import matplotlib.pyplot as plt
import numpy as np
from trajectoire import traj

print("--- TEST VISUALISATION ---")
# On lance le calcul
res = traj(O=(0.320, -0.028, 0.250), R=0.150, V=0.05)
t = res["t"]
t1, t2, tf = res["commutation"]

# --- FONCTION D'AFFICHAGE DES TEMPS ---
def plot_commutation(ax):
    for ti in [t1, t2]:
        ax.axvline(x=ti, color='k', linestyle='--', alpha=0.5)

# --- TRACÉ DES COURBES ---
# 1. Loi de mouvement
plt.figure()
plt.suptitle("Loi de mouvement s(t)")
for i, label in enumerate(["s(t)", "vitesse", "accel"]):
    ax = plt.subplot(3, 1, i+1)
    ax.plot(t, res["s"][i])
    plot_commutation(ax)
    ax.set_ylabel(label); ax.grid(True)
plt.show()

# 2. Articulaire
q_vals = np.array(res["q"])
plt.figure()
plt.suptitle("Positions Articulaires q(t)")
for i in range(6):
    ax = plt.subplot(3, 2, i+1)
    ax.plot(t, q_vals[:, i])
    plot_commutation(ax)
    ax.set_ylabel(f"q{i+1}"); ax.grid(True)
plt.tight_layout()
plt.show()

# 3. Trajectoire 3D
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot(res["x"][0], [0.320]*len(t), res["z"][0]) 
ax.set_title("Trajectoire 3D")
plt.show()
