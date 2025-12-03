import os
import time
import pybullet as p
import pybullet_data
from . import control

class UR3Simulator:
    def __init__(self, urdf_path, dt=0.001):
        self.dt = dt
    # Connexion à PyBullet
        p.connect(p.GUI)
        p.setGravity(0, 0, -9.81)
        p.setPhysicsEngineParameter(fixedTimeStep=self.dt, numSubSteps=1)
        p.resetDebugVisualizerCamera(cameraDistance=1.0, cameraYaw=50, cameraPitch=-35, cameraTargetPosition=[0,0,0])
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    
        # Charger le robot
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.loadURDF("plane.urdf")
        p.setAdditionalSearchPath(os.getcwd())
        self.robot = p.loadURDF(urdf_path, basePosition=[0,0,0], baseOrientation=[0,0,0,1],
                                flags=p.URDF_USE_INERTIA_FROM_FILE, useFixedBase=True)
        
        # Joints contrôlables
        nj = p.getNumJoints(self.robot) - 1
        self.control_joints = list(range(1, nj-2))
        self.end_eff_index = self.control_joints[-1]
    
    def set_joint_angles(self, angles, kp=1.0, kv=1.0):
        control.setJointPosition(self.robot, self.control_joints, angles, kp=kp, kv=kv)
    
    def get_tool_pose(self):
        link_state = p.getLinkState(self.robot, self.end_eff_index)
        return list(link_state[0]), list(link_state[1])  # position, quaternion
    
    def step_simulation(self):
        p.stepSimulation()
        time.sleep(self.dt)
        
            
    
