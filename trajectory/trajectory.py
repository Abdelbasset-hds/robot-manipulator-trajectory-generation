import numpy as np 
from simulation import simulation 

class Trajectory_point_to_point : 
    def __init__(self,sim) : 
        self.sim = sim 
        self.trajectry = list() 
        
    
    def trajectoir_espace_articulation(self, config_init, config_target, kv, ka):

        config_init = np.array(config_init, dtype=float)
        config_target = np.array(config_target, dtype=float)

        dq = np.abs(config_target - config_init)
        n_joints = len(dq)

        self.trajectry.clear()

        # --- Compute trapezoid timing ---
        ta = kv / ka                  # accel time
        da = 0.5 * ka * ta**2         # accel distance

        tc = (dq - 2*da) / kv         # cruise time per joint
        tc = np.maximum(tc, 0)        # if negative -> triangular

        tf = 2*ta + tc                # total time per joint

        temps = np.max(tf)            # synchronize all joints

        # --- Discretization ---
        dt = 0.01
        t = np.arange(0, temps+dt, dt)
        N = len(t)

        q  = np.zeros((N, n_joints))
        dqv = np.zeros((N, n_joints))
        ddq = np.zeros((N, n_joints))

        # --- Trajectory for each joint ---
        for j in range(n_joints):

            q0 = config_init[j]
            qf = config_target[j]
            sign = np.sign(qf - q0)      # direction

            taj = ta
            tcj = tc[j]
            tfj = tf[j]
            daj = da

            for i, ti in enumerate(t):

                if ti < taj:
                    # Acceleration
                    ddq[i, j] = sign * ka
                    dqv[i, j] = sign * ka * ti
                    q[i, j]   = q0 + sign * 0.5 * ka * ti**2

                elif ti < taj + tcj:
                    # Cruise
                    ddq[i, j] = 0
                    dqv[i, j] = sign * kv
                    q[i, j]   = q0 + sign * (daj + kv*(ti - taj))

                elif ti < tfj:
                    # Deceleration
                    t_dec = tfj - ti
                    ddq[i, j] = -sign * ka
                    dqv[i, j] = sign * ka * t_dec
                    q[i, j]   = qf - sign * 0.5 * ka * t_dec**2

                else:
                    q[i, j] = qf
                    dqv[i, j] = 0
                    ddq[i, j] = 0

        # save q for execution
        for qi in q:
            self.trajectry.append(qi)

        return t, q, dqv, ddq




            
    def execute_trajectory(self) : 
        for angle in self.trajectry : 
            self.sim.set_joint_angles(angle, kp=1.0, kv=1.0)