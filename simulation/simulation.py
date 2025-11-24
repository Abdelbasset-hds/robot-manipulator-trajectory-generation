###############################
# Import the necessary modules
###############################
import os
os.getcwd() 
import os.path
# The PyBullet physics simulation library
import pybullet as p
import pybullet_data

# Numpy for numerical calculations and manipulations
import numpy as np
import math
import time
# Matplotlib to create the necessary plots
import matplotlib.pyplot as plt
#################################
import control
import affiche

# paramètres pour Pybullet
# the step time
dt = 0.001 # fréquence du simulateur
# Connet to pybullet and setup simulation parameters.
p.connect(p.GUI)
p.setGravity(0, 0, -0.81) #9.81
p.setPhysicsEngineParameter(fixedTimeStep= dt, numSubSteps=1)
# Zoom onto the robot.
p.resetDebugVisualizerCamera(1.0, 50, -35, (0., 0., 0.))
# Disable the gui controller as we don't use them.
p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)

#p.setRealTimeSimulation(1)

# récupérer le chemin du répertoire courant
path = os.getcwd()
print("Le répertoire courant est : " + path)
# récupérer le nom du répertoire courant
repn = os.path.basename(path)
print("Le nom du répertoire est : " + repn)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf")  # Load the ground plane
#
p.setAdditionalSearchPath(repn)

# Chargement du modèle urdf du robot

urdf_path = "/simulation/ur_description/urdf/ur3_robot.urdf"
robot= p.loadURDF(urdf_path, [0,0,0], [0,0,0,1],flags=p.URDF_USE_INERTIA_FROM_FILE, useFixedBase=True)
# we can now query some information about the robot
nj = p.getNumJoints(robot)
print('the robot has: ' + str(nj) + ' joints\n')
nj=nj-1
print("mais",nj, "joints articulaires\n")
print('the joint names are:')
for i in range(nj+1):
    print(p.getJointInfo(robot, i)[1].decode('UTF-8'))
control_joints = list(range(1, nj-2))   
print("Robot controlable joint",control_joints)
end_eff_index = control_joints[-1]
print("EE= ",end_eff_index)

 # function to solve forward kinematics
def solveForwardPositionKinematics(robot_id, joint):
    print('Forward position kinematics')
    # get end-effector link state
    eeState = p.getLinkState(robot_id, joint)
    link_trn, link_rot, com_trn, com_rot, frame_pos, frame_rot = eeState
#    eePose = list(link_trn) + list(p.getEulerFromQuaternion(link_rot))
    eePose = list(link_trn) + list(link_rot)
    print('End-effector pose:', eePose)
    return eePose

############################
#Init du robot
angle = np.array([1.5707, 0., -0., -1.570796326, -3.14159, -1.57079658])
control.setJointPosition(robot, control_joints, angle, kp=1.0, kv=1.0)
time.sleep(10*dt)

frame_base=[]
frame_base= affiche.draw_coordinate_frame([0,0,0],p.getQuaternionFromEuler([0, 0, 0]), length=0.5)

# Obtenir la position et l'orientation du link associé au joint
link_state = p.getLinkState(robot, 7)

# La pose est donnée par :
pos = link_state[0]  # Position (x, y, z)
orn = link_state[1]  # Orientation (quaternion [x, y, z, w])
ma= np.array(p.getMatrixFromQuaternion(orn))
#print(type(ma))
pp= np.array([pos[0],pos[1],pos[2]])
print(pp)
mat=np.array([ma[0:3],ma[3:6],ma[6:9]])
print(mat)

outil = solveForwardPositionKinematics(robot, 7)
print("Pose Outil=",outil)
# La pose est donnée par :
pos = outil[0:3]  # Position (x, y, z)
ori = outil[3:7]  # Orientation (quaternion [x, y, z, w])

qinv = list(p.calculateInverseKinematics(robot, 7,pos, ori))
print(" qi sol du MGI pour UR3=", qinv, "type=",type(qinv))

control.setJointPosition(robot, control_joints, qinv, kp=1.0, kv=1.0)
outil = solveForwardPositionKinematics(robot, 7)
print("Pose 2 Outil=",outil)

p.setRealTimeSimulation(0)   # désactive le mode temps réel

while True:
    p.stepSimulation()       # avance la physique d’un pas
    time.sleep(0.01)  