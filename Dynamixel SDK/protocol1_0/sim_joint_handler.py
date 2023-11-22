#     |\__/,|   (`\
#   _.|o o  |_   ) )
# -(((---(((--------

import math
from emerge_classes import *

SIM = Sim_setup(False)
HANDLERS = SIM.joint_handlers()
OBJ_HANDLERS = SIM.obj_handlers()

class JointHandlerSim():
    def __init__(self, client, sim):
        self.client = client # client = RemoteAPIClient('localhost', PORT_CONECTION)
        self.sim = sim # sim = self.client.getObject('sim')
        
        # Method needed ariables
        self.exact_rad = 1/180*math.pi
        self.joint_ids = []
        self.num_joints = 0
        self.obj_ids = []
        self.obj_pos_relative_to = self.sim.handle_world

    def getObjectPosition(self, object):
        x, y, z = self.sim.getObjectPosition(object, self.obj_pos_relative_to)
        return x, y, z

    def getJointPosition(self, joint):
        position = self.sim.getJointPosition(joint)
        return position

    # Moves the module until it gets to a final position in a range
    def setJointTargetPosition(self, joint, target_position_rad):
        self.sim.setJointTargetPosition(joint, target_position_rad)
        self.client.step()

        act_pos = self.getJointPosition(joint)
        counter = 0
        diff = act_pos - target_position_rad

        while abs(diff) > self.exact_rad or counter < 5:
            self.sim.setJointTargetPosition(joint, target_position_rad)
            self.client.step()

            new_pos = self.getJointPosition(joint)

            if round(act_pos, 3) == round(new_pos, 3):
                counter += 1
            else:
                counter = counter
        
            act_pos = new_pos
            diff = new_pos - target_position_rad

    # Moves to initial position
    def setJointInitialPosition(self, joint, initial_position_rad):
        self.sim.setJointTargetPosition(joint, initial_position_rad)
        self.sim.setJointPosition(joint, initial_position_rad)

    def connectEMERGE(self):
        self.client.setStepping(True)
        #Joints
        HANDLERS.J0 = self.sim.getObject("/J0")
        HANDLERS.J1 = self.sim.getObject("/J1")
        HANDLERS.J2 = self.sim.getObject("/J2")

        #Setting obj handlers
        OBJ_HANDLERS.p = self.sim.getObject("/p")
    
        #Stores the different id for every joint
        self.joint_ids = list(HANDLERS.__dict__.values())
        self.num_joints = len(self.joint_ids)
        self.obj_ids = list(OBJ_HANDLERS.__dict__.values())
        
    
    def disconnectEMERGE(self):
        pass
    
    # Allows to start testing with the robot
    # Starts simulation (not the same as for robot since the order of use should be the same and makes more sense for the robot)
    def loadEMERGE(self):
        self.sim.startSimulation()
        self.client.step()

    def unloadEMERGE(self):
        self.sim.stopSimulation()