import pybullet as p
import os
import math

class lunarModule:

    def __init__(self,client):

        self.client = client
        f_name = os.path.join(os.path.dirname(__file__), 'lm.urdf')
        self.car = p.loadURDF(fileName=f_name,
                              basePosition=[0, 0, 0.1],
                              physicsClientId=client)

        #-----Constants and parameters-------#

        # mass lost per timestep of 50ms
        self.dm = 0.25

        # Fixed Thrust values
        self.mainThrust = 100.0
        self.sideThrust = 2.5

        # Number of timesteps
        self.duration = 1000.0

        # lengths of lm and angles of side thrusters
        self.alpha = 27.122 * math.pi/180
        self.l = 125.998
        self.r = 180.20 / 2

        # thrsuters to force matrix
        self.ttof = np.array()

        # thrusters to torque matrix
        self.ttot = np.array()

    def get_ids(self):
        return self.lm, self.client

    def apply_action(self, action):

        # Action is (num of thrusters)x1 vector of boolean values
        # Each thruster is either on or off

        # Convert action into a force and torque on COM
        force = ttof * action
        torque = ttot * action

        # Apply the force and torque
        p.applyExternalForce(objectUniqueId=self.lm, linkIndex=-1,
                         forceObj=force, posObj=robotPos, flags=p.LINK_FRAME) #Might have to chance to link frame

        p.applyExternalTorque(objectUniqueId=self.lm, linkIndex=-1,
                        torqueObj=torque, flags=p.LINK_FRAME) #Might have to change to link frame


    def get_observation(self):
        # Get the position and orientation of the lm in the simulation
        #orientation is in Quaternion
        pos, ang = p.getBasePositionAndOrientation(self.lm, self.client)

        # Get the velocity of the lm
        vel = p.getBaseVelocity(self.lm, self.client)[0]

        # Concatenate position, orientation, velocity
        observation = (pos + ang + vel)

        return observation                
