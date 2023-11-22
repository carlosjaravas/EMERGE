#     |\__/,|   (`\
#   _.|o o  |_   ) )
# -(((---(((--------

import math
from dynamixel_sdk import *                    # Uses Dynamixel SDK library
from emerge_classes import *

MODULE = EMERGE_setup()
RAM_ADDR = MODULE.RAM_ADDR()
EEPROM_ADDR = MODULE.EEPROM_ADDR()
RAM_BYTES = MODULE.RAM_BYTES()
EEPROM_BYTES = MODULE.EEPROM_BYTES()

class JointHandlerEMERGE():
    def __init__(self, portHandler, packetHandler):
        self.portHandler = portHandler # Get methods and members of PortHandler: PortHandler(DEVICENAME)
        self.packetHandler = packetHandler # Get methods and members of Protocol1PacketHandler: PacketHandler(PROTOCOL_VERSION)
        
        # Robot depending values
        self.joint_min_position = 205
        self.joint_max_position = 819
        self.exact_rad = 1/180*math.pi
        self.max_num_joint = 12        
        
        # Method needed variables
        self.joint_ids = []
        self.num_joints = 0       
    
    # Gives a the position of the motor as an int
    def rad2AX(self, rad):
        offset = 15/18*math.pi
        rel = 512/offset
        AX_pos = (rad + offset)*rel
        return round(AX_pos)
    
    # Gives a the position of the motor in rad
    def AX2rad(self, AX_pos):
        offset = 15/18*math.pi
        rel = 512/offset
        rad = AX_pos/rel - offset
        return rad

    def enableJointMovement(self, joint):
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, joint, RAM_ADDR.TORQUE_ENABLE, 1)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            pass

    def disbleJointMovement(self, joint):
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, joint, RAM_ADDR.TORQUE_ENABLE, 0)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            pass

    def setJointMinTargetPosition(self, joint):
        dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, joint, EEPROM_ADDR.CW_ANGLE_LIMIT, self.joint_min_position)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            pass
    
    def setJointMaxTargetPosition(self, joint):
        dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, joint, EEPROM_ADDR.CCW_ANGLE_LIMIT, self.joint_max_position)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            pass

    def getObjectPosition(self, object): # To be defined using cameras
        #x, y, z = self.sim.getObjectPosition(object, self.obj_pos_relative_to)
        #return x, y, z
        pass

    # Functions to move joints
    def getJointPosition(self, joint):
        dxl_present_position, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, joint, RAM_ADDR.PRESENT_POSITION)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            pass
        position = self.AX2rad(dxl_present_position)
        return position
    
    # Moves the module to a position
    def setJointAngularPosition(self, joint, target_position_rad):
        target_position = self.rad2AX(target_position_rad)
        dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, joint, RAM_ADDR.GOAL_POSITION, target_position)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            pass

    # Moves the module until it gets to a final position in a range    
    def setJointTargetPosition(self, joint, target_position_rad):
        self.setJointAngularPosition(joint, target_position_rad)

        act_pos = self.getJointPosition(joint)
        counter = 0
        diff = act_pos - target_position_rad

        while abs(diff) > self.exact_rad or counter < 5:
            self.setJointAngularPosition(joint, target_position_rad)

            new_pos = self.getJointPosition(joint)

            if round(act_pos, 3) == round(new_pos, 3):
                counter += 1
            else:
                counter = counter
        
            act_pos = new_pos
            diff = new_pos - target_position_rad

    # Moves to initial position
    def setJointInitialPosition(self, joint, initial_position_rad):
        self.setJointTargetPosition(self, joint, initial_position_rad)
        
    # Pings a Dynamixel using its ID to get the model number
    def getJointStatus(self, joint): 
        # Get Dynamixel model number, for AX-12A should be 12
        dxl_model_number, dxl_comm_result, dxl_error = self.packetHandler.ping(self.portHandler, joint)
        if dxl_comm_result != COMM_SUCCESS:
            #print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            #print("[ID:%03d] ping Failed." % (joint))
            return False
        elif dxl_error != 0:
            #print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            #print("[ID:%03d] ping Failed." % (joint))
            return False
        else:
            #print("[ID:%03d] ping Succeeded. Dynamixel model number : %d" % (joint, dxl_model_number))
            print("[ID:%03d] ping Succeeded." % (joint))
            return True
    
    # Pings all joints in a range and updates the list with the ones available
    def getJointList(self):
        self.joint_ids = []
        for joint in range(1,self.max_num_joint+1):
            isJoint = self.getJointStatus(joint)
            if isJoint:
                self.joint_ids.append(joint)
        self.num_joints = len(self.joint_ids)
    
    # Connects to the EMERGE Modules
    def connectEMERGE(self):
        # Open port
        if self.portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
        # Set port baudrate
        if self.portHandler.setBaudRate(MODULE.BAUDRATE):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate") 

    def disconnectEMERGE(self):
        pass

    # Allows to start testing with the robot
    def loadEMERGE(self):
        self.connectEMERGE()
        self.getJointList()
        for joint in self.joint_ids:
            self.setJointMinTargetPosition(joint)
            self.setJointMaxTargetPosition(joint)
            self.enableJointMovement(joint)

    def unloadEMERGE(self):
        for joint in self.joint_ids:
            self.disbleJointMovement(joint)
        self.portHandler.closePort()