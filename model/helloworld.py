import pybullet as p
import pybullet_data
from pybullet_object_models import ycb_objects
import numpy as np

import os, inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0, parentdir)



import time
import math as m

# Scaling factors for force and torque
ALPHA = 3
BETA = 3

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
robotID = p.loadURDF(urdf_path, basePosition=[0, 0, 0.5])

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

    # force = ALPHA * (np.array(gemPos) - np.array(robotPos))
    force = (np.array([F_x, F_y, F_z]))
    torque = (np.array([T_x, T_y, T_z]))

    p.applyExternalForce(objectUniqueId=robotID, linkIndex=-1,
                         forceObj=force, posObj=robotPos, flags=p.WORLD_FRAME)

    p.applyExternalTorque(objectUniqueId=robotID, linkIndex=-1,
                        torqueObj=torque, flags=p.WORLD_FRAME)

    print('Applied force vector = {}'.format(force))
    print('Applied force magnitude = {}'.format(np.linalg.norm(force)))

    print('Applied torque vector = {}'.format(torque))
    print('Applied torque magnitude = {}'.format(np.linalg.norm(torque)))

p.disconnect()
