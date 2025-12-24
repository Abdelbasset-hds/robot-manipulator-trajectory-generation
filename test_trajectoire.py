import matplotlib.pyplot as plt
import numpy as np
from trajectoire import traj

print("--- TEST VISUALISATION ---")

O_test = (0.320, -0.028, 0.250)
res = traj(O=O_test, R=0.150, V=0.05)

t = res["t"]
t1, t2, tf = res["commutation"]

def plot_commutation(ax):
    for ti in [t1, t2]:
        ax.axvline(x=ti, color='k', linestyle='--', alpha=0.5)



plt.figure(figsize=(10, 8))
plt.suptitle("Loi de mouvement s(t)")
labels_s = ["Position s(t)", "Vitesse s'(t)", "Accélération s''(t)"]
for i in range(3):
    ax = plt.subplot(3, 1, i+1)
    ax.plot(t, res["s"][i])
    plot_commutation(ax)
    ax.set_ylabel(labels_s[i]); ax.grid(True)
plt.show()

q_vals = np.array(res["q"])
qd_vals = np.array(res["qd"])
plt.figure(figsize=(12, 10))
plt.suptitle("Positions et Vitesses Articulaires")

for i in range(6):
    ax = plt.subplot(6, 2, 2*i+1)
    ax.plot(t, q_vals[:, i])
    plot_commutation(ax)
    ax.set_ylabel(f"q{i+1}")
    ax.grid(True)
    if i==0: ax.set_title("Positions [rad]")
    
    ax = plt.subplot(6, 2, 2*i+2)
    ax.plot(t, qd_vals[:, i], 'orange')
    plot_commutation(ax)
    ax.set_ylabel(f"dq{i+1}")
    ax.grid(True)
    if i==0: ax.set_title("Vitesses [rad/s]")
    
plt.tight_layout()
plt.show()

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

Y_fixe = [O_test[1]] * len(t) 

ax.plot(res["x"][0], Y_fixe, res["z"][0], label="Trajectoire")
ax.scatter(O_test[0], O_test[1], O_test[2], c='r', marker='o', label='Centre O')

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title("Trajectoire 3D (Espace Opérationnel)")
ax.legend()
plt.show()
