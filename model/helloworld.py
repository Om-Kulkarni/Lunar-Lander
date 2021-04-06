import pybullet as p
import pybullet_data
from pybullet_object_models import ycb_objects
import numpy as np
from math import sin,cos,pi 

import os, inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0, parentdir)



import time
import math as m

# Scaling factors for force and torque
ALPHA = 3
BETA = 3
alpha = 27.122*pi / 180

r1 = 125.998 
r2 = 180.20 / 2

#radius = 125.988 mm
#height = 180.20 mm

DURATION = 10000

# ------------------------ #
# --- Setup simulation --- #
# ------------------------ #

# Create pybullet GUI
physics_client_id = p.connect(p.GUI)
p.resetDebugVisualizerCamera(1.8, 120, -50, [0.0, -0.0, -0.0])
p.resetSimulation()
p.setPhysicsEngineParameter(numSolverIterations=150)
sim_timestep = 1.0/240
p.setTimeStep(sim_timestep)
p.setRealTimeSimulation(0)

# Load plane contained in pybullet_data
p.loadURDF(os.path.join(pybullet_data.getDataPath(), "plane.urdf"))

# Set gravity for simulation. For easy movement without restriction
p.setGravity(0, 0, -9.8)

urdf_path = r"Lunar_Lander.urdf"
robotID = p.loadURDF(urdf_path, useFixedBase=True, basePosition=[0, 0, 0.5])

# Final POSITION
# gemId = p.loadURDF("duck_vhacd.urdf", [
#                    2, 2, 1],  p.getQuaternionFromEuler([0, 0, 0]))


position, orientation = p.getBasePositionAndOrientation(robotID)


# 6 DOF Forces and Torques
F_x = p.addUserDebugParameter('F_x', -2, 2, 0)
F_y = p.addUserDebugParameter('F_y', -2, 2, 0)
F_z = p.addUserDebugParameter('F_z', -2, 2, 0)

T_x = p.addUserDebugParameter('T_x', -2, 2, 0)
T_y = p.addUserDebugParameter('T_y', -2, 2, 0)
T_z = p.addUserDebugParameter('T_z', -2, 2, 0)

#Might want to use applyExternalForce(objectUniqueID = robotID,
#                                      linkIndex = -1, 
#                                      forceObj = [F_x,F_y,F_z],
#                                      posObj = [Cm_x,Cm_y,Cm_z],
#                                      flags = LINK_FRAME,
#                                      physicsClientId = physics_client_id)

F111 = 0
F112 = 0
F121 = 0
F122 = 0
F131 = 0
F132 = 0
F141 = 0
F142 = 0
F211 = 0
F212 = 0
F221 = 0
F222 = 0
F231 = 0
F232 = 0
F241 = 0
F242 = 0
F_T = 0

for i in range(DURATION):
#while True:

    p.stepSimulation()
    time.sleep(1./240.)

    user_F_x = p.readUserDebugParameter(F_x)
    user_F_y = p.readUserDebugParameter(F_y)
    user_F_z = p.readUserDebugParameter(F_z)

    user_T_x = p.readUserDebugParameter(T_x)
    user_T_y = p.readUserDebugParameter(T_y)
    user_T_z = p.readUserDebugParameter(T_z)

    #gemPos, gemOrn = p.getBasePositionAndOrientation(gemId)
    robotPos, robotOrn = p.getBasePositionAndOrientation(robotID)

    F_x = -F122*sin(alpha) + F121*sin(alpha) + F142*sin(alpha) - F141*sin(alpha) - F222*sin(alpha) + F221*sin(alpha) + F242*sin(alpha) - F241*sin(alpha)
    F_y = F111*sin(alpha) - F112*sin(alpha) - F131*sin(alpha) + F132*sin(alpha) + F211*sin(alpha) - F212*sin(alpha) - F231*sin(alpha) + F232*sin(alpha)
    F_z = (- F111*cos(alpha) - F112*cos(alpha) - F121*cos(alpha) - F122*cos(alpha) - F131*cos(alpha) - F132*cos(alpha) - F141*cos(alpha) - F142*cos(alpha) + 
            F211*cos(alpha) + F212*cos(alpha) + F221*cos(alpha) + F222*cos(alpha) + F231*cos(alpha) + F232*cos(alpha) + F241*cos(alpha) + F242*cos(alpha) + F_T) 

    T_x = ( -F111*sin(alpha)*r2 + F112*sin(alpha)*r2 + F121*cos(alpha)*r2 + F122*cos(alpha)*r2 + F131*sin(alpha)*r2 - F132*sin(alpha)*r2 - F141*cos(alpha)*r2 - F142*cos(alpha)*r2 
    + F211*sin(alpha)*r2 - F212*sin(alpha)*r2 - F221*cos(alpha)*r2 - F222*cos(alpha)*r2 - F231*sin(alpha)*r2 + F232*sin(alpha)*r2 + F241*cos(alpha)*r2 + F242*cos(alpha)*r2) 
    T_y = (F111*cos(alpha)*r2 + F112*cos(alpha)*r2 + F121*sin(alpha)*r2 - F122*sin(alpha)*r2 - F131**cos(alpha)*r2 - F132*sin(alpha)*r2 - F141*sin(alpha)*r2 + F142*sin(alpha)*r2 
    - F211*cos(alpha)*r2 + - F212*cos(alpha)*r2 - F221*sin(alpha)*r2 + F222*sin(alpha)*r2 + F231*cos(alpha)*r2 + F232*sin(alpha)*r2 + F241*sin(alpha)*r2 - F242*sin(alpha)*r2)  
    T_z = (F111*sin(alpha)*r1 - F112*sin(alpha)*r1 + F121*sin(alpha)*r1 - F122*sin(alpha)*r1 + F131*sin(alpha)*r1 - F132*sin(alpha)*r1 + F141*sin(alpha)*r1 - F142*sin(alpha)*r1 
    + F211*sin(alpha)*r1 - F212*sin(alpha)*r1 + F221*sin(alpha)*r1 - F222*sin(alpha)*r1 + F231*sin(alpha)*r1 - F232*sin(alpha)*r1 + F241*sin(alpha)*r1 - F242*sin(alpha)*r1)

    # force = ALPHA * (np.array(gemPos) - np.array(robotPos))
    force = (np.array([F_x, F_y, F_z]))
    torque = (np.array([T_x, T_y, T_z]))

    p.applyExternalForce(objectUniqueId=robotID, linkIndex=-1,
                         forceObj=force, posObj=robotPos, flags=p.LINK_FRAME) #Might have to chance to link frame

    p.applyExternalTorque(objectUniqueId=robotID, linkIndex=-1,
                        torqueObj=torque, flags=p.LINK_FRAME) #Might have to change to link frame

    print('Applied force vector = {}'.format(force))
    print('Applied force magnitude = {}'.format(np.linalg.norm(force)))

    print('Applied torque vector = {}'.format(torque))
    print('Applied torque magnitude = {}'.format(np.linalg.norm(torque)))

p.disconnect()
